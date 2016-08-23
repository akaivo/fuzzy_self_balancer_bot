/*
 * mikroprotsessorid_l6put66.c
 *
 * Created: 19.05.2016 21:10:41
 *  Author: A
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

#define dataHigh() PORTE |= (1<<PE4)
#define dataLow() PORTE &= ~(1<<PE4)
#define clockHigh() PORTE |= (1<<PE3)
#define clockLow() PORTE &= ~(1<<PE3)
#define latchHigh() PORTB |= (1<<PB7)
#define latchLow() PORTB &= ~(1<<PB7)
#define segHigh() PORTD |= (1<<PD4)
#define segLow() PORTD &= ~(1<<PD4)

#define startSPI()	(PORTB &= ~(1<<SS))
#define endSPI() (PORTB |= (1<<SS))


#define SS PB0
#define SPICLOCK PB1
#define MOSI PB2
#define READ 0x80
#define MULTIPLE 0x40
#define TEMP_CFG_REG 0x1F
#define TEMP_EN (1<<6)
#define CTRL_REG4 0x23
#define HR (1<<3)
#define BDU (1<<7)
#define ST1 (1<<2)
#define ST0 (1<<1)
#define CTRL_REG1 0x20
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define ONE_HERTZ 0x10// 0001 0000
#define TEN_HERTZ 0x20// 0010 0000
#define FOUR_HUNDRED_HERTZ 0x70
#define LOW_POWER_ENABLE (1<<3)
#define X_AXIS 0x01
#define Y_AXIS 0x02
#define Z_AXIS 0x04

volatile uint8_t hex_digits[] = {0b11111100,0b01100000,0b11011010,0b11110010,0b01100110,0b10110110,0b10111110,0b11100000,0b11111110,0b11110110,0b11101110,0b00111110,0b10011100,0b01111010,0b10011110,0b10001110};
volatile uint8_t seg_display_elements[2];
volatile uint8_t current_number = 0;

uint8_t SPI_transfer(char cData);

float float_min(float a, float b){
	if(a>b){
		return b;
	}
	return a;
}
float float_max(float a, float b){
	if(a<b){
		return b;
	}
	return a;
}
int16_t abs(int16_t input){
	if(input<0){
		return -input;
	}
	return input;
}

void JTAG_off(){
	uint8_t temp = MCUCR;
	temp |= (1<<JTD);
	MCUCR = temp;
	MCUCR = temp;
}
void init_pins()
{
	//servo pins
	DDRC |= (1<<PC6);//servo b
	DDRB |= (1<<PB5);//servo a
	
	DDRA = 0xFF; //LEDs
	
	//7seg display
	DDRE |= (1<<PE4)|(1<<PE3);//data&clock
	DDRB |= (1<<PB7);//latch
	DDRD |= (1<<PD4);//segment chooser
	
	//joystick
	DDRF &= ~((1<<PF6)|(1<<PF4)) ;	//center-down input
	PORTF |= (1<<PF6)|(1<<PF4);		//center-down pull-up
}
void init_SPI_and_accelerometer()
{
	PRR0 &= ~(1<<PRSPI); //SPI enable (SPI power reduction off)
	DDRB |= (1<<SS)|(1<<SPICLOCK)|(1<<MOSI);//SPI(PB=SS,PB1=CLOCK,PB2=MOSI) output pins
	PORTB |= (1<<SS);//slave select high - no transmission
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA);//SPI enable,master,idle clock high, drive on fall - read on rise
	SPSR = (1<<SPI2X);//SPI speed f_osc/2 - 1MHz SPI @ 2MHz processor
	
	startSPI();
	SPI_transfer(CTRL_REG1);
	SPI_transfer(FOUR_HUNDRED_HERTZ|X_AXIS|Y_AXIS|Z_AXIS);
	endSPI();
	
	startSPI();
	SPI_transfer(TEMP_CFG_REG);
	SPI_transfer(TEMP_EN);
	endSPI();
	
	startSPI();
	SPI_transfer(CTRL_REG4);
	SPI_transfer(HR|BDU);
	endSPI();
}
void init_UART(){
	UBRR1 = 0;							//baud rate= 256000bps @ 2MHz. manual pg. 198
	UCSR1A = (1<<U2X1);					//double speed
	UCSR1B = (1<<TXEN1);				//transmit enable
	UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);	//default
}
void init_timers()
{
	//timer 0 - 7 segment display
	TCCR0A = 0;						//default
	TCCR0B = (1<<CS01)|(1<<CS00);	//prescaler 64
	TIMSK0 = (1<<TOIE0);			//overflow interrupt
	
	//timer 1 - servo 1 PWM
	ICR1 = 40000;								//20ms~50Hz @2MHz & no prescaler
	TCCR1A = (1<<WGM11)|(1<<COM1A1);			//fast pwm, TOP=ICR1, update at TOP; set on TOP, clear on compare match
	TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS00);	//fast pwm, no prescaler
	OCR1A = 2960;
	
	//timer 3 - servo 2 PWM
	ICR3 = 40000;								//20ms~50Hz @2MHz & no prescaler
	TCCR3A = (1<<WGM31)|(1<<COM3A1);			//fast pwm, TOP=ICR3, update at TOP; set on TOP, clear on compare match
	TCCR3B = (1<<WGM32)|(1<<WGM33)|(1<<CS00);	//fast pwm, no prescaler
	OCR3A = 2987;
}
void init_adc()
{
	ADMUX = (1<<MUX1)|(1<<REFS0);				//channel 2
	ADCSRB = 0;									//auto trigger --> free running
	ADCSRA = (1<<ADEN)|(1<<ADSC)|(1<<ADATE);	//ADC enable,start conversion, auto trigger
    
}

void write_next_byte(uint8_t serial_data){//data to 7seg display
	
	for(uint8_t i = 0;i<8;i++){
		clockLow();
		if(serial_data&0b00000001){
			dataHigh();
			} else {
			dataLow();
		}
		clockHigh();
		clockLow();
		
		serial_data = serial_data>>1;
	}
	latchLow();
	latchHigh();
}
void send_UART_word(char sone[]){		
	uint16_t k = 0;
	uint16_t len = strlen(sone);
	while (k<len){						//to string end
		while( !(UCSR1A & (1<<UDRE1)) );//wait until UDR1 free
		UDR1 = sone[k];					//send
		k++;							//next character
	}
}

void servo_1(int16_t speed){
	OCR1A = 2997 + speed;//2997 - 1. servo stop value
}
void servo_2(int16_t speed){
	OCR3A = 2982 + speed;//2982 - 2. servo stop value
}

void led_indicator(int16_t acc_x){
	uint8_t led = 1;
	if(acc_x<0){
		acc_x = abs(acc_x)/64;
		for (int8_t i = 0;i<8;i++){
			if(i<acc_x){
				PORTA |= led;
			} else {
				PORTA &= ~led;
			}
			led = led<<1;
		}
	} else {
		acc_x = acc_x/64;
		for (int8_t i = 7;i>=0;i--){
			if(i>7-acc_x){
					PORTA |= led;
				} else {
					PORTA &= ~led;
				}
			}
			led = led<<1;
		}
	
}
void display_adc_value(uint16_t input){
	seg_display_elements[0] = input & 0xF;
	seg_display_elements[1] = input >> 4;
}


uint8_t SPI_transfer(char address)
{
	/* Start transmission */
	SPDR = address;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));
	/*return received MISO byte*/
	return SPDR;
}

/*------fuzzy membership functions----------*/
float triangle_membership(int16_t input,int16_t left,int16_t center,int16_t right){
	if(input<=left || input>=right){
		return 0;
	}
	if(input<center){
		return (float)(input-left)/(center-left);
	}
	return (float)(right-input)/(right-center);
}
float trapezoid_membership(int16_t input,int16_t left,int16_t center_left,int16_t center_right,int16_t right){
	if(input<=left || input>=right){
		return 0;
	}
	if(input<center_left){
		return (float)(input-left)/(center_left-left);
	}
	if(input>center_right){
		return (float)(right-input)/(right-center_right);
	}
	return 1;
}


int main(void)
{	
	JTAG_off();
	init_pins();
	init_SPI_and_accelerometer();
	init_UART();
	init_timers();
	init_adc();
    
	sei();
	segHigh();
	
	char number[20];// for UART output
	
	uint8_t balance_off = 1;
	uint16_t adc_divided = 0;
	int16_t acc_x = 0;
	int16_t acc_y = 0;
	int16_t servo_kiirus = 0;	
	
	/*-----------------fuzzy variables start----------------*/
	//input acceleration Z - zero, P - positive, N - negative, S,L - small,large
	float bZ = 0;
	float bP = 0;
	float bPL = 0;
	float bN = 0;
	float bNL = 0;
	
	
	//input speed
	float is_Z = 0;
	float is_P = 0;
	float is_N = 0;
	
	//output speed
	int16_t speed_PS = 15;
	int16_t speed_NS = -15;
	int16_t speed_P = 30;
	int16_t speed_N = -30;
	int16_t speed_PL = 40;
	int16_t speed_NL = -40;
	int16_t speed_PLL = 160;
	int16_t speed_NLL = -160;
	
	float weight_rule1 = 0;
	float weight_rule2 = 0;
	float weight_rule3 = 0;
	float weight_rule4 = 0;
	float weight_rule5 = 0;
	float weight_rule6 = 0;
	float weight_rule7 = 0;
	float weight_rule8 = 0;
	float weight_rule9 = 0;
	/*---------------fuzzy variables end--------------------*/
	
	
	while(1)
    {	
		if(~PINF & (1<<PF6)){//joystick center - turn balancing off
			balance_off = 1;
		}
		if(~PINF & (1<<PF4)){//joystick down - turn balancing on
			balance_off = 0;
		}
		
		startSPI();
		SPI_transfer(OUT_X_L|READ|MULTIPLE);			//start x low 
		acc_x = SPI_transfer(0) | (SPI_transfer(0)<<8);	//x_low,x_high
		acc_y = SPI_transfer(0) | (SPI_transfer(0)<<8);	//y_low,...
		endSPI();
		
		if(balance_off){		//change balance only when stopped
			adc_divided = (1023 - ADC)/4;
			}
			
		acc_x = acc_x/16;		//accelerator gives left adjusted data with 12 bit precision(sign bit + 11 bits number => range ~ +-1024)
		acc_x = acc_x - 450 - 128 + adc_divided;//robot is off,balance by default. adjust center of mass
		
		led_indicator(acc_x);
		display_adc_value(adc_divided);
		//itoa(servo_kiirus,number,10);
		//send_UART_word(number); send_UART_word("\r\n");
		
		/*-------fuzzy inputs--------*/
		bZ = triangle_membership(acc_x,-30,0,30);
		bP = triangle_membership(acc_x,0,30,100);
		bPL = trapezoid_membership(acc_x,30,100,2000,2001);
		bN = triangle_membership(acc_x,-100,-30,0);
		bNL = trapezoid_membership(acc_x,-2001,-2000,-100,-30);
		
		is_Z = triangle_membership(servo_kiirus,-15,0,15);
		is_P = trapezoid_membership(servo_kiirus,5,100,500,501);
		is_N = trapezoid_membership(servo_kiirus,-501,-500,-100,-5);
		
		/*------fuzzy rules------(min means fuzzy AND)*/
		weight_rule1 = float_min(bPL,is_P);
		weight_rule2 = float_min(bNL,is_N);
		weight_rule3 = float_min(bP,is_P);
		weight_rule4 = float_min(bN,is_N);
		weight_rule5 = float_min(bPL,is_Z);
		weight_rule6 = float_min(bNL,is_Z);
		weight_rule7 = float_min(bP,is_Z);
		weight_rule8 = float_min(bN,is_Z);
		weight_rule9 = float_min(bZ,is_Z);
		
		/*-----inference(Sugeno singleton)--------*/
		servo_kiirus = (int16_t)(
			(weight_rule1*speed_PLL + 
			 weight_rule2*speed_NLL + 
			 weight_rule3*speed_PL + 
			 weight_rule4*speed_NL + 
			 weight_rule5*speed_P + 
			 weight_rule6*speed_N + 
			 weight_rule7*speed_PS + 
			 weight_rule8*speed_NS)/
			(weight_rule1 + weight_rule2 + weight_rule3 + weight_rule4 + weight_rule5 + weight_rule6 + weight_rule7 + weight_rule8 + weight_rule9)
			);//weighted average of rules aka. defuzzification
		
		if(abs(acc_y)>11000 || balance_off){
			servo_1(0);
			servo_2(0);
		} else {
			servo_1(-servo_kiirus);
			servo_2(servo_kiirus);
		}
    }
}

ISR(TIMER0_OVF_vect){
	write_next_byte(0);
	PORTD = (PORTD & ~(1<<PD4)) | (current_number << PD4);
	current_number = !bcurrent_number;
	write_next_byte(hex_digits[seg_display_elements[current_number]]);
}