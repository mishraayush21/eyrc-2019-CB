/*
*	Team Id		:	eYRC#2158
*	Author List	:	Ayush Mishra , Raj Krishnan , Prashant Singh , Ashish Maurya
*	Filename	:	task5-main.c
*	Theme		:	Construct-O-Bot (CB)
*	Functions	:	pick(void), place(void), main(), Task5(int []), min_cost_pick_place(int [],int),  min_cost_pick_place(int []),
*					 zig_zag_line(unsigned char), move_source_dest(int,int), command_traversal(int),
*					node_detector(unsigned char , unsigned char , unsigned char ),forward_wls(unsigned char), left_turn_wls(void), right_turn_wls(void),
*					create_graph(void), findpath(int,int), traversal_command(int), white_line_follower(unsigned char), white_forward_wls(void),
*					white_node_detector(unsigned char , unsigned char , unsigned char ), wall_follower(unsigned char ), adjust_left(void), adjust_right(void),
*					white_left(void), white_right(void)
*	
*	Global variables : int white_node_flag,int flag_u,int flag_v,int Start,int flag_special,int shortest_dist,int path[MAX],char command[MAX],int house_type,
*					   int alter_pick_place , int current_orientation ,int flag_adj , unsigned char left_sensor_value , unsigned char middle_sensor_value ,
*					   unsigned char right_sensor_value , unsigned char node , unsigned char temp_node_counter ,volatile unsigned long int shaftcountleft ,
*					   volatile unsigned long int shaftcountRight , char adj[MAX][MAX], int vertices, int H[6], int S, int P, int B, int C, int E, int G,
*					   int sc,int bc, int gc, int pc, int ec, int cc , int house_rise[6] 
*/


#define F_CPU 14745600
#define minwall_follower 120
#define maxwall_follower 140
#define MAX 65
#define TEMP 0
#define PERM 1
#define infinity 99
#define cutoff 40 

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

int white_node_flag = 0;
int flag_u = 0;
int flag_v = 0;
volatile int Start = 52;
int flag_special = 0;
int shortest_dist; 
int path[MAX];
char command[MAX];
int house_type=0;
int alter_pick_place = 0;
int current_orientation = 2;
int flag_adj = 0;
unsigned char left_sensor_value ;
unsigned char middle_sensor_value ;
unsigned char right_sensor_value ;
unsigned char node = 0 , temp_node_counter = 0; 
volatile unsigned long int shaftcountleft = 0;
volatile unsigned long int shaftcountRight = 0;



unsigned char ADC_Conversion(unsigned char) ;
unsigned char ADC_value , ADC_reading ;

void lcd_init(void);
void lcd_port_config(void);
void lcd_print(char row, char coloumn, unsigned int value, int digits) ;

 int B= 43, S = 33, G = 47, P = 19, C = 29, E = 15;
/*
B => Address of Brick	S => Address of  Sand	C => Address of  Cement 
P => Address of  Paint	G => Address of  Gravel	E => Address of  Electrical fittings 
*/  

int H[6] = {0, 35, 41, 21, 27, 10}; //Stores the address of houses form H1 to H5
int bc=2,sc=2,gc=2,pc=2,cc=2,ec=2;
/*
bc => brick count	sc => sand count	gc => gravel count
pc => paint count	cc => cement count	ec => electrical fittings count
*/


char adj[MAX][MAX]; //Adjacency matrix for graph implementation , char type used to avoid memory overflow
int vertices;	//Number of vertices in graph

/***********************************************************************************************************************************************************/

//  ALL HARDWARE LIKE LCD , BUZZER ,SERVO MOTORS , DC MOTORS , PROXIMITY SENSORS , PINS etc INITIALIZED

void adc_pin_config (void)
{
	DDRF = 0x00;
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

void sensor_init(void)
{
	DDRF = 0x00 ;
	PORTF = 0x00 ;
}

void adc_init()
{   // ADC pin initialization 
	ADCSRA = 0x00;
	ADCSRB = 0x00;		
	ADMUX = 0x20;		
	ACSR = 0x80;
	ADCSRA = 0x86;		
}

unsigned char ADC_Conversion(unsigned char Ch)
{   // ADC conversion function
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		
	while((ADCSRA&0x10)==0);
	a=ADCH; 
	ADCSRA = ADCSRA|0x10; 
	ADCSRB = 0x00;
	return a;
}
//initalize port to use encoder interrupts 
void left_encoder_pin_config (void)//Position Encoder
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 5 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 5 pin
}

void print_sensor(char row , char column , unsigned char channel)
{
	ADC_value = ADC_Conversion(channel) ;
	lcd_print(row,column,ADC_value,3) ;
}
// FOR LCD PINS

// LCD HEADER FILE NOT DIRECTLY USED DUE TO ERRORS PRODUCED DUE TO HEADER FILE INCLUSION



#define RS 0
#define RW 1
#define EN 2
#define lcd_port PORTC

#define sbit(reg,bit)	reg |= (1<<bit)	
#define cbit(reg,bit)	reg &= ~(1<<bit)		
// Functions for LCD
void init_ports();
void lcd_reset();
void lcd_init();
void lcd_wr_command(unsigned char);
void lcd_wr_char(char);
void lcd_line1();
void lcd_line2();
void lcd_string(char*);

unsigned int temp;
unsigned int unit;
unsigned int tens;
unsigned int hundred;
unsigned int thousand;
unsigned int million;

//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();
}

//Function to Reset LCD
void lcd_set_4bit()
{
	_delay_ms(1);

	cbit(lcd_port,RS);				
	cbit(lcd_port,RW);				
	lcd_port = 0x30;			
	sbit(lcd_port,EN);				
	_delay_ms(5);	
	cbit(lcd_port,EN);				

	_delay_ms(1);

	cbit(lcd_port,RS);				
	cbit(lcd_port,RW);				
	lcd_port = 0x30;				
	sbit(lcd_port,EN);			
	_delay_ms(5);					
	cbit(lcd_port,EN);				

	_delay_ms(1);

	cbit(lcd_port,RS);			
	cbit(lcd_port,RW);				
	lcd_port = 0x30;				
	sbit(lcd_port,EN);			
	_delay_ms(5);					
	cbit(lcd_port,EN);				

	_delay_ms(1);

	cbit(lcd_port,RS);				
	cbit(lcd_port,RW);			
	lcd_port = 0x20;			
	sbit(lcd_port,EN);				
	_delay_ms(1);			
	cbit(lcd_port,EN);			

	
}

//Function to Initialize LCD
void lcd_init()
{
	_delay_ms(1);

	lcd_wr_command(0x28);			//LCD 4-bit mode.
	lcd_wr_command(0x01);
	lcd_wr_command(0x06);
	lcd_wr_command(0x0E);
	lcd_wr_command(0x80);
	
}


//Function to Write Command on LCD
void lcd_wr_command(unsigned char cmd)
{
	unsigned char temp;
	temp = cmd;
	temp = temp & 0xF0;
	lcd_port &= 0x0F;
	lcd_port |= temp;
	cbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
	
	cmd = cmd & 0x0F;
	cmd = cmd<<4;
	lcd_port &= 0x0F;
	lcd_port |= cmd;
	cbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
}

//Function to Write Data on LCD
void lcd_wr_char(char letter)
{
	char temp;
	temp = letter;
	temp = (temp & 0xF0);
	lcd_port &= 0x0F;
	lcd_port |= temp;
	sbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);

	letter = letter & 0x0F;
	letter = letter<<4;
	lcd_port &= 0x0F;
	lcd_port |= letter;
	sbit(lcd_port,RS);
	cbit(lcd_port,RW);
	sbit(lcd_port,EN);
	_delay_ms(5);
	cbit(lcd_port,EN);
}


//Function to bring cursor at home position
void lcd_home()
{
	lcd_wr_command(0x80);
}


//Function to Print String on LCD
void lcd_string(char *str)
{
	while(*str != '\0')
	{
		lcd_wr_char(*str);
		str++;
	}
}

//Position the LCD cursor at "row", "column".

void lcd_cursor (char row, char column)
{
	switch (row) {
		case 1: lcd_wr_command (0x80 + column - 1); break;
		case 2: lcd_wr_command (0xc0 + column - 1); break;
		case 3: lcd_wr_command (0x94 + column - 1); break;
		case 4: lcd_wr_command (0xd4 + column - 1); break;
		default: break;
	}
}

//Function To Print Any input value upto the desired digit on LCD
void lcd_print (char row, char coloumn, unsigned int value, int digits)
{
	unsigned char flag=0;
	if(row==0||coloumn==0)
	{
		lcd_home();
	}
	else
	{
		lcd_cursor(row,coloumn);
	}
	if(digits==5 || flag==1)
	{
		million=value/10000+48;
		lcd_wr_char(million);
		flag=1;
	}
	if(digits==4 || flag==1)
	{
		temp = value/1000;
		thousand = temp%10 + 48;
		lcd_wr_char(thousand);
		flag=1;
	}
	if(digits==3 || flag==1)
	{
		temp = value/100;
		hundred = temp%10 + 48;
		lcd_wr_char(hundred);
		flag=1;
	}
	if(digits==2 || flag==1)
	{
		temp = value/10;
		tens = temp%10 + 48;
		lcd_wr_char(tens);
		flag=1;
	}
	if(digits==1 || flag==1)
	{
		unit = value%10 + 48;
		lcd_wr_char(unit);
	}
	if(digits>5)
	{
		lcd_wr_char('E');
	}
	
}


void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F; //set direction of the PORTA 3 to PORTA 0 pins as output
	PORTA = PORTA & 0xF0; // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM
}
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5address1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}
// Function for robot velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}
//Function to initialize ports
//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 			// removing upper nibbel as it is not needed
	PortARestore = PORTA; 			// reading the PORTA's original status
	PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
	PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
	PORTA = PortARestore; 			// setting the command to the port
}


void forward (void) //both wheels forward
{   
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{   

	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{   
	
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void stop (void) //hard stop
{
	motion_set(0x00);
}
void port1_init()
{
	motion_pin_config();
	left_encoder_pin_config();
	right_encoder_pin_config();
	
	
}
void left_position_encoder_interrupt_init(void)
{
	
	cli();
	EICRB = EICRB|0x02;//(INT4)
	EIMSK = EIMSK|0x10;
	sei();
	
}
void Right_position_encoder_interrupt_init(void)
{
	
	cli();
	EICRB = EICRB|0x08;//(INT5)
	EIMSK = EIMSK|0x20;
	sei();
	
}
ISR(INT5_vect)
{
	
	shaftcountRight++;
	
}
ISR(INT4_vect)
{
	
	shaftcountleft++;
	
}
void left_encoder_pin(void)
{
	
	DDRE = DDRE & 0xEF;
	PORTE = PORTE | 0x10; //(PORTE pin 4)
}
void Right_encoder_pin(void)
{
	
	DDRE = DDRE & 0xDF;
	PORTE = PORTE | 0x20; //(PORTE pin 5)
}
void linear_distance(unsigned int Distance)
{
	
	float Reqdshaftcount = 0;
	unsigned long int ReqdshaftcountInt = 0;
	Reqdshaftcount = Distance/.390;
	ReqdshaftcountInt = (unsigned long int) Reqdshaftcount;
	shaftcountRight = 0;
	while(1)
	{
		if(shaftcountRight > ReqdshaftcountInt)
		{
			break;
		}
	}

	stop();
}
void forward_mm(unsigned int DistanceInMM)
{
	
	forward();
	linear_distance(DistanceInMM);
}
void back_mm(unsigned int DistanceInMM)
{
	
	back();
	linear_distance(DistanceInMM);
}
void right_mm(unsigned int DistanceInMM)
{
	
	right();
	linear_distance(DistanceInMM);
}
void left_mm(unsigned int DistanceInMM)
{
	
	left();
	linear_distance(DistanceInMM);
}


void init_devices (void)
{   cli();
	lcd_port_config();
	lcd_set_4bit();
	adc_pin_config();
	adc_init();
	lcd_init();
	motion_pin_config();
	port_init();
	port1_init();
    left_position_encoder_interrupt_init();
	Right_position_encoder_interrupt_init();
	timer5_init(); 
	devices();
	port3_init();
	sei();
}

void servo1(void)
{
	DDRB  = DDRB | 0x20;  // PORTB 5 pin output
	PORTB = PORTB | 0x20; //PORTB 5 pin to logic 1
}
void servo2(void)
{
	DDRB  = DDRB | 0x40;  // PORTB 6 pin output
	PORTB = PORTB | 0x40; // PORTB 6 pin to logic 1
}

void timer1(void)
{
	TCCR1B = 0x00;
	TCNT1H = 0xFC;
	TCNT1L = 0x01;
	OCR1AH = 0x03;
	OCR1AL = 0xFF;
	OCR1BH = 0x03;
	OCR1BL = 0xFF;

	ICR1H  = 0x03;
	ICR1L  = 0xFF;
	TCCR1A = 0xAA;
	
	TCCR1B = 0x0C;
}

void servo_1(unsigned char degrees)                   // for arm  
{
	float PositionofServo = 0;
	PositionofServo = ((float)degrees / 1.86) + 35.0;
	OCR1AH = 0x00;
	OCR1AL = (unsigned char) PositionofServo;
}
void servo_2(unsigned char degrees)                 //for gripper
{
	float PositionofServo = 0;
	PositionofServo = ((float)degrees / 1.86) + 35.0;
	OCR1BH = 0x00;
	OCR1BL = (unsigned char) PositionofServo;
}

void servo_1_off(void)
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}
void servo_2_off(void)
{
	OCR1BH = 0x03;
	OCR1BL = 0xFF; //Servo 2 off
}


void devices (void)
{
	cli();
	servo1();
	servo2();
	timer1();
	sei();
}


void buzzer_pin_config (void)
{
	DDRC = DDRH | 0x08;		     //Setting PORTC 3 as output
	PORTH = PORTH & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

void port3_init (void)
{
	buzzer_pin_config();
}

void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINH;
	port_restore = port_restore | 0x08;
	PORTH = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINH;
	port_restore = port_restore & 0xF7;
	
	PORTH = port_restore;
}


/***********************************************************************************************************************************************************/




/*
* Function Name: pick(void)
* Input		 :	void
* Output	 :	void
* Logic		 :	Picks the Construction material block after aligning it. 
* Example Call : pick();
*/
void pick(void)
	{	
		if(flag_adj == 0  )
		{
			adjust_left();
			
			stop();
			velocity(255,255);
			
		}
		else if(flag_adj == 1 )
		{
			adjust_right();
			
			stop();
			velocity(255,255);
			
		}
	
		
		servo_2(20);
		_delay_ms(500);
		servo_1(90);
		_delay_ms(1400);
		servo_1_off();
		servo_2(70);
		_delay_ms(1000);
		servo_1(0);
		_delay_ms(1800);
		servo_1_off();            
	back_mm(40);                  // after pick we move bot in back
	}
	
/*
* Function Name: place(void)
* Input		 :	void
* Output	 :	void
* Logic		 :	Places the Construction material block after aligning it according to type of house i.e., low or high rise or House 5 
* Example Call : pick();
*/
void place(void)
{
	if(flag_adj == 0 && flag_u != 1 && flag_special != 1)
	{
		adjust_left();
		
		stop();
		velocity(255,255);
		
	}
	else if(flag_adj == 1 && flag_u != 1 && flag_special != 1)
	{
		adjust_right();
		
		stop();
		velocity(255,255);
		
	}
	
	if(flag_u != 1)
	{
		if(house_type == 0)//If house is not H5 and house type is low
		{
			
			back_mm(80);
			_delay_ms(100);
			servo_1(55);
			_delay_ms(1000);
			servo_2(30);                 // gripper open , places the object
			_delay_ms(1000);
			servo_1(0);
			_delay_ms(1000);
			forward_mm(35);

			servo_1_off();
			servo_2_off();
		}
		else if(house_type == 1)//If house is not H5 and house type is high
		{
			
			if(flag_special != 1)
			back_mm(120);
			else
			back_mm(85);
			_delay_ms(100);
			servo_1(28);
			_delay_ms(1000);
			servo_2(0);                 // gripper open , places the object
			_delay_ms(1000);
			servo_1(0);
			_delay_ms(1000);
			
			forward_mm(43);

			servo_1_off();
			servo_2_off();
			
		}
		
		
	}
	else
	{
		if(house_type == 0)//If house is  H5 and house type is low
		{
			
			back_mm(80);
			_delay_ms(100);
			servo_1(55);
			_delay_ms(1000);
			servo_2(30);                 // gripper open , places the object
			_delay_ms(1000);
			servo_1(0);
			_delay_ms(1000);
			forward_mm(35);

			servo_1_off();
			servo_2_off();
		}
		
		else if(house_type == 1)//If house is H5 and house type is high
		{
			
		back_mm(115);
		_delay_ms(100);
		servo_1(28);
		_delay_ms(1000);
		servo_2(0);                 // gripper open , places the object
		_delay_ms(1000);
		servo_1(0);
		_delay_ms(700);
		forward_mm(70);
		_delay_ms(30);
		servo_1_off();
		servo_2_off();
		right();
		_delay_ms(55);
		
		}
	}
}


 //ONLY CONFIGURTION OF BLOCKS AND HIGH-RISE/LOW-RISE NEEDS TO BE MODIFIED FOR TRAVERSAL AND PICK PLACE ACCORDING TO ANY CONFIGURATION PROVIDED.
 int house_rise[6] = {0, 1, 1, 1, 1 , 1 };//House rise passed as given 



/*
* Function Name: main()
* Input		 :	void
* Output	 :  void
* Logic		 : This function passes configuration array to Task5()
*				
* Example Call :main()
*/
 int main(void)
 {
	init_devices();
	servo_1(0);                   
	_delay_ms(1000);
	servo_1_off();
	int config[10] = {S, B, B, G, P, S, C, G, E, P};//Array passed according to given configurations
	
	Task5(config);
	
	buzzer_on();
	_delay_ms(5000);
	buzzer_off();
	
	return(0);
	
}

/*
* Function Name: Task5()
* Input		 :	config array
* Output	 :	
* Logic		 : This function calls min_cost_pick_place_H5() made for house 5 and then it calls
*				min_cost_pick_place() for pick-place execution in other houses
* Example Call :Task5(int config[10]);
*/
void Task5(int config[10])
{
	int counter = 0;//Counter variable to count number of blocks to be placed except House 5
	
	if(config[0] != 0 || config[1] != 0)
		min_cost_pick_place_H5(config);// For house 5 
	 
	 if(config[0] == 0 && config[1] == 0)
		Start = 52;
	
	for(int i = 2 ; i < 10 ; i++)
		if(config[i] != 0 )
			counter++;
	
	min_cost_pick_place(config,counter);// For house 1,2,3 and 4
}



/*
* Function Name:  min_cost_pick_place(int config[],int k)
* Input		 :	  config array, frequency of non-zero elements in config array except for House 5
* Output	 :	
* Logic		 : This function checks the minimum cost pick-place execution available from the current location of 
*				bot and makes recursive call to itself by considering the Start position as its current position
*				and then calls move_source_dest().  
* Example Call :min_cost_pick_place(int config[],int k);
*/
void min_cost_pick_place(int config[],int k)
{   create_graph();
	
	int key =0;
	int temp_cost,count,min_cost = 1000, address1,address2;
	
	if(k > 0  )
	{
		for(int i = 2 ; i < 10 ; i++)//Minimum among all possible paths is found and then stored temporarily  
		{   
			temp_cost = 0;
			if(config[i]!=0)
			{
				count = findpath(Start,config[i]);
				
				temp_cost = shortest_dist;
				
				count = findpath(config[i],H[(11-i)/2]);
				temp_cost += shortest_dist;
				
				
				if(temp_cost < min_cost)
				{
					min_cost = temp_cost;
					address1 = config[i];
					address2 = H[(11-i)/2];
					key = i;
					
				}
				
				
			}
			
		}
		
		
		if(address1 == B)
		move_source_dest( Start, (address1+(int)pow(-1,bc-- )));
		
		
		
		else if(address1 == S)
		move_source_dest( Start,(address1+ (int)pow(-1,sc-- )));
		
		
		else if(address1 == G)
		move_source_dest( Start,(address1+ (int)pow(-1,gc--) ));
		
		else if(address1 == E)
		move_source_dest( Start, address1+(int)pow(-1,ec-- ));
		
		
		else if(address1 == C)
		move_source_dest( Start, address1+(int)pow(-1,cc-- ));
		
		else if(address1 == P)
		move_source_dest( Start,address1+ (int)pow(-1,pc-- ));
		
		//Moved from current position to CM position	
		
		
		
		move_source_dest(address1,address2);//Moved from CM position to correct house position 
		
	
		if(address2 == 35)
		Start = 36;
		else if(address2 == 41)
		Start = 40;
		else if(address2 == 21)
		Start = 22;
		else if(address2 == 27)
		Start = 26;
		else if(address2 == 10)
		Start = 3;
		
		config[key] = 0;//the CM which is placed is made 0 in config array
		
		min_cost_pick_place(config,k-1);//recursive call with changed Start
										
		
		
	}
	
}


/*
* Function Name: min_cost_pick_place_H5(int config[10])
* Input		 :	config array
* Output	 :
* Logic		 : This function compares the cost of performing pick-place operation in House 5 and gets the order of operation
*              in such a way that overall cost is minimum and then it calls move_source_dest()
* Example Call :min_cost_pick_place_H5(int config[10]);
*/

void min_cost_pick_place_H5(int config[10])//only for House 5 pick-place
{  
	
	
	int count,temp_cost_1,temp_cost_2;
	create_graph();
	if(config[0] != 0 && config[1] != 0)//To place two CM in H5
	{
		count = findpath(Start,config[0]);
		temp_cost_1 = shortest_dist;
		count = findpath(config[0],H[5]);
		temp_cost_1 += shortest_dist;
		
		count = findpath(H[5],config[1]);
		temp_cost_1 += shortest_dist;
		count = findpath(config[1],H[5]);
		temp_cost_1+= shortest_dist;
		
		count = findpath(Start,config[1]);
		temp_cost_2 = shortest_dist;
		count = findpath(config[1],H[5]);
		temp_cost_2 += shortest_dist;
		
		count = findpath(H[5],config[0]);
		temp_cost_2 += shortest_dist;
		count = findpath(config[0],H[5]);
		temp_cost_2+= shortest_dist;
		int p =3,address1,address2;
		
		if(temp_cost_1 < temp_cost_2)//Order of pick-place cost is compared
		{
			address1 = config[0];
			address2 = config[1];
		}
		else
		{
			address1 = config[1];
			address2 = config[0];
		}
		

		
		if(address1 == B)
		move_source_dest( Start, (address1+(int)pow(-1,bc-- )));
		
		
		
		else if(address1 == S)
		move_source_dest( Start,(address1+ (int)pow(-1,sc-- )));
		
		
		else if(address1 == G)
		move_source_dest( Start,(address1+ (int)pow(-1,gc--) ));
		
		else if(address1 == E)
		move_source_dest( Start, address1+(int)pow(-1,ec-- ));
		
		
		else if(address1 == C)
		move_source_dest( Start, address1+(int)pow(-1,cc-- ));
		
		else if(address1 == P)
		move_source_dest( Start,address1+ (int)pow(-1,pc-- ));
		
		//Moved from current position to  CM position
		
		
		move_source_dest(address1,H[5]);//Moved from CM position to correct house position
		
		Start = 3;//Address next to H5
		
		if(address2 == B)
		move_source_dest( Start, (address2+(int)pow(-1,bc-- )));
		
		
		
		else if(address2 == S)
		move_source_dest( Start,(address2+ (int)pow(-1,sc-- )));
		
		
		else if(address2 == G)
		move_source_dest( Start,(address2+ (int)pow(-1,gc--) ));
		
		else if(address2 == E)
		move_source_dest( Start, address2+(int)pow(-1,ec-- ));
		
		
		else if(address2 == C)
		move_source_dest( Start, address2+(int)pow(-1,cc-- ));
		
		else if(address2 == P)
		move_source_dest( Start,address2+ (int)pow(-1,pc-- ));
		//Moved from current position to CM position
		
		
		move_source_dest(address2,H[5]);//Moved from CM position to correct house position
		
		
		
	}
	else if((config[0] != 0 && config[1] == 0)||(config[0] == 0 && config[1] != 0))//To place only one CM in H5 
	{
		int address1;
		if(config[0] != 0)
			 address1 = config[0];
		else
			 address1 = config[1];
		
		if(address1 == B)
		move_source_dest( Start, (address1+(int)pow(-1,bc-- )));
		
		else if(address1 == S)
		move_source_dest( Start,(address1+ (int)pow(-1,sc-- )));
		
		
		else if(address1 == G)
		move_source_dest( Start,(address1+ (int)pow(-1,gc--) ));
		
		else if(address1 == E)
		move_source_dest( Start, address1+(int)pow(-1,ec-- ));
		
		
		else if(address1 == C)
		move_source_dest( Start, address1+(int)pow(-1,cc-- ));
		
		else if(address1 == P)
		move_source_dest( Start,address1+ (int)pow(-1,pc-- ));
		
		
		move_source_dest(address1,H[5]);
		
		
	}
	
	Start = 3;
}




/*
* Function Name: check_left_right(void)
* Input		 :	
* Output	 :
* Logic		 : This function checks for black line available at the left and then at right until a particular limit of Shaft Count.
*			   This function has been used in zig-zag line following. 
* Example Call :check_left_right();
*/
void check_left_right(void)//used in case of zig zag line
{	shaftcountRight = 0;
	shaftcountleft = 0;
	int cutoff2 = 30;
	int flag_align = 0;
	while(1){//For finding line on left
		left_sensor_value = ADC_Conversion(3);	//Getting data of Left WL Sensor
		middle_sensor_value = ADC_Conversion(2);	//Getting data of Center WL Sensor
		right_sensor_value = ADC_Conversion(1);	//Getting data of Right WL Sensor

		if((left_sensor_value<cutoff2 && middle_sensor_value > cutoff2 && right_sensor_value < cutoff2)   )
		{	
			flag_align=1;//if bot is aligned h =1
			stop();
			break;	
		}
		if(shaftcountRight > 190 || shaftcountleft > 190)
		{
			//if left turn limit is reached
			stop();
			break;	
		}
		velocity(150,150);
		left();
		
		
	
	}
	
	if(flag_align != 1)//if bot is not aligned and no black line is found on left
	{
	while(1){
		
		left_sensor_value = ADC_Conversion(3);	//Getting data of Left WL Sensor
		middle_sensor_value = ADC_Conversion(2);	//Getting data of Center WL Sensor
		right_sensor_value = ADC_Conversion(1);	//Getting data of Right WL Sensor

		if((left_sensor_value<cutoff2 &&middle_sensor_value > cutoff2 && right_sensor_value < cutoff2) 
		|| ( shaftcountleft > 800 || shaftcountRight > 800 ))
		{
			stop();
		
		break;	
		}
		velocity(150,150);
		right();
		
		
	
	}
	}
	
}	


/*
* Function Name: zig_zag_line(unsigned char node)
* Input		 :
* Output	 :
* Logic		 : This function follows black line until any of sensors is on black line. When all sensors are on white 
*				check_left_right(void) is called which checks and aligns bot on left or right. 
* Example Call :zig_zag_line('1');
*/
void zig_zag_line(unsigned char node)
 {	int cutoff2 =28;
	 temp_node_counter = 0;	//initialized to 0 so that it can iterate till number of nodes to go forward
    
	
     
	 while (1)
	 {
		 if ( temp_node_counter <= (int)node - 49)	//Since node is of 'char' type (int)node converts it to ascii value and 49 is ascii code of '0' which is subtracted

		 {

			 left_sensor_value = ADC_Conversion(3);
			 middle_sensor_value = ADC_Conversion(2);
			 right_sensor_value = ADC_Conversion(1);
			 

			 //Sensor values are assigned
			 node_detector(left_sensor_value, middle_sensor_value,right_sensor_value);
			 
			 // node_detector function is called in each iteration to check for a node
			 // node_detector increases node_counter if node is encountered

		    		 if ( ( left_sensor_value < cutoff2 && middle_sensor_value > cutoff2 && right_sensor_value < cutoff2  ) || ( left_sensor_value > cutoff2 && middle_sensor_value > cutoff2 && right_sensor_value < cutoff2  )
					  ||  ( left_sensor_value < cutoff2 && middle_sensor_value > cutoff2 && right_sensor_value > cutoff2  )  )
		    		 {	velocity(255,255);
			    		 forward();
		    		 }
					
		    		 else if ( right_sensor_value > cutoff2 )
		    		 {
			            velocity(200,200);
						soft_right();
						
			

		    		 }

		    		 else if ( left_sensor_value > cutoff2 )
		    		 {  
						velocity(200,200);
						soft_left();
			    	   

		    		 }
		    		 
				
		 
			
			
	     	 if( left_sensor_value < cutoff2 && middle_sensor_value< cutoff2 && right_sensor_value < cutoff2 )
			 {		
				 // If all sensors are on white 
				 forward();
				 _delay_ms(21);
				 	
					 check_left_right();
			 }
			 
			
		 }

		 else
		 {    
			 velocity(255,255);          // It executes when bot has moved by the number of nodes specified
			 break;
		 }
	 }
 }	
	
 
/*
* Function Name: move_source_dest(int source, int dest)
* Input		 :
* Output	 :
* Logic		 : This function moves the bot from source to destination using the shortest path by calling findpath().
*				Then it makes call to pick() and place() alternately. 
* Example Call :move_source_dest(52, 10);
*/
 void move_source_dest(int source, int dest)
 {
	
	create_graph() ;
	 int i,command_len;
	
	
	int count;
		
		 
		 count = findpath(source,dest);//path array generated in reverse order
      
	  
	  
	  
	  if(dest == 41)
		house_type = house_rise[2];
	  else if(dest == 21)
		house_type = house_rise[3];
	 else if(dest == 27)
		 house_type = house_rise[4];
	 else if(dest == 35)
		 house_type = house_rise[1];
	  else if(dest == 10)
		 house_type = house_rise[5]; 
	  
	  
	  
	
			int start = 0;
			int end = count;
			 int temp; 
            while (start < end) 
            { 
             temp = path[start];    
             path[start] = path[end]; 
             path[end] = temp; 
             start++; 
            end--; 
            } 
       //path array is made correct by reversing it again
		
        command_len = traversal_command(count);//Command array generated
		
		

		flag_u = 0;
		 command_traversal(command_len);
		 
		 
		 stop();
		 _delay_ms(1000);//to stop the bot before performing pick or place
		 
		 
		//alternate pick place is called
		 if(alter_pick_place == 0)
		{
			alter_pick_place = 1;
			pick();
			
		  }
		 else
		 {
			 alter_pick_place = 0;
			 place();
			 
		 }
		
				
	}
		 
		 
/*
* Function Name: command_traversal(int j)
* Input		 :	Length of command array 
* Output	 :
* Logic		 :	This function makes call to forward_wls(), right_turn_wls(), left_turn_wls(), wall_follow(), zig_zag_line(),
*				 white_line_follower() and white_forward_wls() as per the command array.
* Example Call :command_traversal(5);
*/
void command_traversal(int command_len)
		 {	
			 white_node_flag = 0;
			 flag_adj = 2;// flag for adjust left and right
			 flag_v = 0;//flag for case V
			

			for (int i = 0; i < command_len-1; i++)
			{
				
				switch (command[i])
				{
					case 'F':
						forward_wls('1');
						flag_special = 0;
						break;
						
					case 'R':
						flag_adj = 0;
						
						if(command[i+1] == 'V')
						{
							white_right();
							continue;
						}
						
						right_turn_wls();
						
						break;
						
					case 'L':
						flag_adj = 1;
						
						if(command[i+1] == 'V')
						{
							white_left();
							continue;
						}
						
						left_turn_wls();
						
						break;
						
					case 'Z':
						zig_zag_line('1');
						flag_special = 1;
						break;
					
					case 'U'://Case when bot follows black line then white line
						back_mm(30);
						
						if(flag_adj == 0)
							adjust_left();
						else if(flag_adj == 1)
							adjust_right();
						
						white_forward_wls();
						flag_u= 1;
					
					if(command[i-1] == 'R' && command[i+1] == 'R')
					{
						
					white_right();
					command_len--;
					}
					else if(command[i-1] == 'L' && command[i+1] == 'L')
					{
						white_left();
						command_len--;
					}
					
				
						break;
				
				case 'V':	//Case when bot follows white line then black line
				
					flag_v =1;
					
				
					white_line_follower('1');
				
					forward_wls('1');
						break;
					
					case 'W':
						flag_special = 1;
						wall_follower('1');
						break;
					
					case 'A'://for 180 degree turn
						
						
						
						right_turn_wls();
			
						right_turn_wls();
					
					default:
						break;
					
					
				}
				
	
			}
	
		
	
		
 }
 
 
/*
* Function Name: node_detector( unsigned char l, unsigned char m, unsigned char r)
* Input		 :	left_sensor_value, middle_sensor_value and right_sensor_value
* Output	 :
* Logic		 :	This function checks whether all sensors are on black or any of the two sensors are on black  and then bot is
*				moved forward from the node. It also increments the value of node and temp_node_counter. 
* Example Call :node_detector(left_sensor_value, middle_sensor_value, right_sensor_value)
*/
 void node_detector( unsigned char l, unsigned char m, unsigned char r)
 {   
	 if (  (l > cutoff+40 && m > cutoff+40 && r > cutoff+40) || (m > cutoff+40 && r > cutoff+40) || (m > cutoff+40 && l > cutoff+40) )
	 { 
		node++ ;
		temp_node_counter++ ;
		forward();
		_delay_ms(150);       
		stop();
		 
		 
	 }
 }
 
 
/*
  *
  * Function Name: forward_wls
  * Input: node
  * Output: void
  * Logic: Uses white line sensors to go forward by the number of nodes specified
  * Example Call: forward_wls(2); //Goes forward by two nodes
  *
  */
 void forward_wls(unsigned char node)
 {
	 temp_node_counter = 0;	//initialized to 0 so that it can iterate till number of nodes to go forward

	
	 while (1)
	 {
		 if ( temp_node_counter <= (int)node - 49)	//Since node is of 'char' type (int)node converts it to ascii value and 49 is ascii code of '0' which is subtracted

		 {

			 left_sensor_value = ADC_Conversion(3);
			 middle_sensor_value = ADC_Conversion(2);
			 right_sensor_value = ADC_Conversion(1);
			 

			 //Sensor values are assigned
			 node_detector(left_sensor_value, middle_sensor_value,right_sensor_value);
			 
			 // node_detector function is called in each iteration to check for a node
			 // node_detector increases node_counter if node is encountered

			 if ( left_sensor_value < cutoff && middle_sensor_value > cutoff && right_sensor_value < cutoff )//As 255 value represents black line
			 {    
				 forward();
				 
			 }

			 else if ( right_sensor_value >  cutoff ) // To align the bot accordingly if it deviates from its path
			 {
				 soft_right();

			 }

			 else if ( left_sensor_value > cutoff) // To align the bot
			 {
				 soft_left();

			 }

		 }

		 else
		 {              // It executes when bot has moved by the number of nodes specified
			 break;
		 }
	 }
	 
	
 }

/*
*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn left until black line is encountered
* Example Call: left_turn_wls(); //Turns right until black line is encountered
*
*/

 void left_turn_wls(void)
 {	
	 left();                         //    jerk in provided to increase the accuracy of left turn 
	 _delay_ms(250);
    
	 while (1  )
	 {
		 left_sensor_value = ADC_Conversion(3);
		

		 if ( left_sensor_value > cutoff   ) // As soon as left sensor finds a black line it breaks
		 {
			stop();
			velocity(255,255);
			 break;
		 }

		
		 left();

	 }

 }
 
 
 /*
 *
 * Function Name: right_turn_wls
 * Input: void
 * Output: void
 * Logic: Uses white line sensors to turn right until black line is encountered
 * Example Call: right_turn_wls(); //Turns right until black line is encountered
 *
 */
 void right_turn_wls(void)
 {		
	 right();                //  jerk in provided to increase the accuracy of right turn 
	 _delay_ms(250);
	 
	 
	 while (1)
	 {
		
		 right_sensor_value = ADC_Conversion(1);

		 if ( right_sensor_value > cutoff ) // As soon as right sensor finds a black line it breaks
		 {
            stop();
			velocity(255,255);
			 break;
		 }


		 right();

	 }

 }



/*
*
* Function Name: create_graph()
* Input: void
* Output: void
* Logic:	Creates the adjacency matrix for implementation of Dijkstra's algo. Also it assigns specific weight to
*			 all edges of the graph.   
* Example Call: create_graph()
*/
void create_graph()
{
	//Whole arena is implemented as graph with 63 vertices as a matrix of 9 rows and & columns
	int wt =1;//weights of particular edges of graph
	int wt2 = 2;
	int wt4 = 4;


	vertices = 63;

adj[1][3]=wt2;
adj[3][5]=wt2;
adj[3][10]=wt;
adj[1][15]=wt;
adj[5][19]=wt;
adj[14][15]=wt;
adj[15][16]=wt;
adj[18][19]=wt;
adj[19][20]=wt;
adj[15][22]=wt;
adj[21][22]=wt;
adj[19][26]=wt;
adj[22][26]=wt4+1;
adj[26][27]=wt;
adj[22][29]=wt;
adj[26][33]=wt;
adj[28][29]=wt;
adj[29][30]=wt;
adj[32][33]=wt;
adj[33][34]=wt;
adj[29][36]=wt;
adj[33][40]=wt;
adj[35][36]=wt;
adj[36][40]=wt4;
adj[40][41]=wt;
adj[36][43]=wt;
adj[40][47]=wt;
adj[42][43]=wt;
adj[43][44]=wt;
adj[46][47]=wt;
adj[47][48]=wt;
adj[43][57]=wt;
adj[47][61]=wt;
adj[52][59]=wt;
adj[57][59]=wt2;
adj[59][61]=wt2;


adj[3][1]=wt2;
adj[5][3]=wt2;
adj[10][3]=wt;
adj[15][1]=wt;
adj[19][5]=wt;
adj[15][14]=wt;
adj[16][15]=wt;
adj[19][18]=wt;
adj[20][19]=wt;
adj[22][15]=wt;
adj[22][21]=wt;
adj[26][19]=wt;
adj[26][22]=wt4+1;
adj[27][26]=wt;
adj[29][22]=wt;
adj[33][26]=wt;
adj[29][28]=wt;
adj[30][29]=wt;
adj[33][32]=wt;
adj[34][33]=wt;
adj[36][29]=wt;
adj[40][33]=wt;
adj[36][35]=wt;
adj[40][36]=wt4;
adj[41][40]=wt;
adj[43][36]=wt;
adj[47][40]=wt;
adj[43][42]=wt;
adj[44][43]=wt;
adj[47][46]=wt;
adj[48][47]=wt;
adj[57][43]=wt;
adj[61][47]=wt;
adj[59][52]=wt;
adj[59][57]=wt2;
adj[61][59]=wt2;

}

/*
*
* Function Name: findpath(int s,int d)
* Input: address of source and destination vertices
* Output: length of path array created
* Logic:	Creates the path array which stores address of vertices to which bot should be moved in order to 
*			follow the shortest path using Dijkstra's algorithm
* Example Call: findpath(52,10)
*/

int findpath(int s,int d)//Dijkstra algorithm to generate path
{	int state[2*MAX][3];
	int i,min,count=0,current,newdist,u,v;
	shortest_dist=0;
	
	for(i=1;i<=vertices;i++)
	{
		state[i][0]= 0;
		state[i][1] = infinity;
		state[i][2] = TEMP;
	}

	
	state[s][0]=0;
	state[s][1] = 0;
	state[s][2]= PERM;

	current=s;
	while(current!=d)
	{
		for(i=1;i<=vertices;i++)
		{
			if ( adj[current][i] > 0 && state[i][2] == TEMP )
			{
				newdist=state[current][1] + adj[current][i];
				/*Checks for Relabeling*/
				if( newdist < state[i][1] )
				{
					state[i][0] = current;
					state[i][1] = newdist;
					
				}
			}
		}
		min=infinity;
		current=0;
		for(i=1;i<=vertices;i++)
		{
			if(state[i][2] == TEMP && state[i][1] < min)
			{
				min = state[i][1];
				current=i;
				
			}
		}

	if(current==0)
		return 0;
		
		state[current][2]=PERM;
	}
	while( current!=0 )
	{
		count++;
		path[count]=current;
		current=state[current][0];
	}

	
	for(i=count;i>1;i--)
	{
		u=path[i];
		v=path[i-1];
		shortest_dist+= adj[u][v];
	}
	return (count);

}

/*
*
* Function Name:  traversal_command(int count)
* Input:  length of path array
* Output: length of command array created
* Logic:	Creates the command array which stores the command such as 'F' for forward , 'R' for right,etc. Moreover orientation
*			of bot is considered in order to find whether right or left or 180 degree turn is required.   
* Example Call: traversal_command(12)
*/
int traversal_command(int count)
{
	
	int required_orientation = current_orientation;
	int j = 0;
	for (int i = 0; i < count-1 ; i++) {
		
		if ((int)abs(path[i + 1] - path[i]) % 7 == 0 && path[i] > path[i + 1])
		required_orientation = 0;
		else if ((int)abs(path[i + 1] - path[i]) % 7 == 0 && path[i] < path[i + 1])
		required_orientation = 2;
		else if (path[i] < path[i + 1])
		required_orientation = 1;
		else if (path[i + 1] < path[i])
		required_orientation = 3;
		
		
		if (current_orientation > required_orientation)
		{
			if (current_orientation - required_orientation == 1)
			command[j++] = 'L';
			else if (current_orientation - required_orientation == 2)
			command[j++] = 'A';
			else
			command[j++] = 'R';

		}
		else if (current_orientation < required_orientation)
		{
			if (required_orientation - current_orientation == 3)
			command[j++] = 'L';
			else if (required_orientation - current_orientation == 2)
			command[j++] = 'A';
			else
			command[j++] = 'R';


		}
		
		if ((path[i + 1] == 26 && path[i] == 22) || (path[i + 1] == 22 && path[i] == 26))
		command[j++] = 'Z';

		else if ((path[i + 1] == 36 && path[i] == 40) || (path[i + 1] == 40 && path[i] == 36))
		command[j++] = 'W';
		else if ((path[i + 1] == 1 && path[i] == 3) || (path[i + 1] == 5 && path[i] == 3))
		command[j++] = 'V';
		else if ((path[i + 1] == 3 && path[i] == 1) || (path[i + 1] == 3 && path[i] == 5))
		command[j++] = 'U';

		else{
			command[j++] = 'F';
			
			
		}
		
		
		current_orientation = required_orientation;
		
		
		
	}


	return(j);
}



/*
*
* Function Name:  white_line_follower(unsigned char)
* Input:  node
* Output: void
* Logic:	Moves the bot by specified	number of nodes in case of white line following segment.
* Example Call: white_line_follower('1');
*/

void white_line_follower(unsigned char node)
{   temp_node_counter = 0;
	int Cutoff1 = 80;
	shaftcountleft = 0;
	 shaftcountRight = 0;
	while(1)
	{
			left_sensor_value = ADC_Conversion(3);	//Getting data of Left WL Sensor
			middle_sensor_value = ADC_Conversion(2);	//Getting data of Center WL Sensor
			right_sensor_value = ADC_Conversion(1);	//Getting data of Right WL Sensor
			
		
		if ( temp_node_counter <= (int)node - 49  )
		{
			
			
		
				white_node_detector(left_sensor_value,middle_sensor_value,right_sensor_value);
				// calling node detector function

			if( left_sensor_value > Cutoff1 && middle_sensor_value < Cutoff1 && right_sensor_value > Cutoff1)
			{
				
				velocity(250,250);
				forward();
				
			}
			
			else if(left_sensor_value > Cutoff1 && middle_sensor_value > Cutoff1 && right_sensor_value < Cutoff1 )
			{  
				soft_right();
				
			}

			else if(left_sensor_value < Cutoff1 && middle_sensor_value > Cutoff1 && right_sensor_value > Cutoff1  )
			{  
				soft_left();
				
			}
		}
		else{
			
			velocity(255,255);
			break;
		}
		
		

	}
}


/*
* Function Name: white_node_detector( unsigned char , unsigned char , unsigned char )
* Input		 :	left_sensor_value, middle_sensor_value and right_sensor_value
* Output	 :
* Logic		 :	This function checks whether all sensors are on white or any of the two sensors are on white  and then bot is
*				moved forward from the node. It also increments the value of node and temp_node_counter.
* Example Call :white_node_detector(left_sensor_value, middle_sensor_value, right_sensor_value)
*/
void white_node_detector(unsigned char left_sensor_value,unsigned char middle_sensor_value,unsigned char right_sensor_value)
{
	int Cutoff1 = 40;
	
	if ((( left_sensor_value < Cutoff1 && middle_sensor_value < Cutoff1)|| (  middle_sensor_value < Cutoff1 && right_sensor_value < Cutoff1)
	|| (left_sensor_value < Cutoff1 && right_sensor_value<Cutoff1) ) && (shaftcountleft >1150 || shaftcountRight > 1150))
	{   
		if(white_node_flag == 0)
			node ++ ;
		
		temp_node_counter++;
		
		white_node_flag = 1;
		forward_mm(55);
		
		stop();	
	}
	
}

/*
*
* Function Name:  wall_follower(unsigned char )
* Input:  node
* Output: void
* Logic:	Makes the bot follow black line until all sensors are on white and  then wall following segment starts. For the wall 
*			proximity sensor is used, when distance between wall and bot is increased, it is decreased according to soft turns and vice-versa. 
* Example Call: wall_follower('1');
*/	
void wall_follower(unsigned char node)
{
	int i=0;
	temp_node_counter = 0;	//initialized to 0 so that it can iterate till number of nodes to go forward

	unsigned char left_sensor_value, middle_sensor_value, right_sensor_value, right_wall,left_wall;


	while (1)
	{
		if (temp_node_counter <= (int)node - 49)	//Since node is of 'char' type (int)node converts it to ascii value and 49 is ascii code of '0' which is subtracted

		{

			left_sensor_value = ADC_Conversion(3);
			middle_sensor_value = ADC_Conversion(2);
			right_sensor_value = ADC_Conversion(1);
			right_wall = ADC_Conversion(9);
			left_wall = ADC_Conversion(10);

			//Sensor values are assigned

			node_detector(left_sensor_value, middle_sensor_value, right_sensor_value);
			// node_detector function is called in each iteration to check for a node
			// node_detector increases node_counter if node is encountered

			if ((left_sensor_value < cutoff && middle_sensor_value > cutoff && right_sensor_value < cutoff))//As 255 value represents black line
			{

				forward();
			}

			else if ((right_sensor_value > cutoff)) // To align the bot accordingly if it deviates from its path
			{
				soft_right();
			}

			else if (left_sensor_value > cutoff) // To align the bot
			{
				soft_left();
			}
			
			else if ((left_sensor_value < cutoff && middle_sensor_value < cutoff && right_sensor_value < cutoff))
			{
				
				if ( (  right_wall >= minwall_follower && right_wall <= maxwall_follower) )
				{
					forward();
				}
				else if (right_wall > maxwall_follower)
				{
					//Wall follower alignment
					soft_right();
				}
				else if (right_wall < minwall_follower )
				 {
					//Wall follower alignment
					soft_left();
				}
				else if ( (right_wall < 85)  )
				{
					// wall is end 
					
					return(0);
					
					
					
					
				}

			}
			

		}

		else
		{ // It executes when bot has moved by the number of nodes specified
			break;
		}
	}



}


	
/*
*
* Function Name:  adjust_left()
* Input:  void
* Output: void
* Logic:	Adjusts the bot for accurate pick and place on left after right turn.
* Example Call: adjust_left()
*/
void adjust_left()
{  
	forward_mm(6);
	
	while(1)
	{
		
		left_sensor_value = ADC_Conversion(3);	//Getting data of Left WL Sensor
		middle_sensor_value = ADC_Conversion(2);	//Getting data of Center WL Sensor
		right_sensor_value = ADC_Conversion(1);	//Getting data of Right WL Sensor

		if( left_sensor_value< cutoff+11 && middle_sensor_value > cutoff+20 && right_sensor_value < cutoff+20   ) //this condition will check weather robot is on black line or not and if it is on black line than it will break the loop
		{
			break;
			stop();
		}
		velocity(125,125);
		left();//this will take left turn until it find black line
		

	}

}


/*
*
* Function Name:  adjust_right()
* Input:  void
* Output: void
* Logic:	Adjusts the bot for accurate pick and place on right after left turn.
* Example Call: adjust_right()
*/
void adjust_right()
{  
	forward_mm(5);
	
	while(1)
	{
		

		left_sensor_value = ADC_Conversion(3);	//Getting data of Left WL Sensor
		middle_sensor_value = ADC_Conversion(2);	//Getting data of Center WL Sensor
		right_sensor_value = ADC_Conversion(1);	//Getting data of Right WL Sensor

		if(  left_sensor_value< cutoff+20 && middle_sensor_value > cutoff+20 && right_sensor_value < cutoff+17 )//this condition will check weather robot is on black line or not and if it is on black line than it will break the loop
		{	stop();
			break;
		}
		velocity(125,125);
		right(); //it will take right turn until it find black line

	}
	
	
	}

	
	/*
	*
	* Function Name: white_left()
	* Input:  void
	* Output: void
	* Logic:	Makes left turn for place in House 5
	* Example Call: white_left()
	*/
	void white_left(void)
	{
		left();                                 //  jerk in provided to increase the accuracy of left  turn 
		_delay_ms(300);
	
	
		while (1  )
		{
			left_sensor_value = ADC_Conversion(3);
			

			if ( left_sensor_value < 50   ) // As soon as left sensor finds a black line it breaks
			{
				stop();
				
				break;
				
			}

			velocity(170,170);
			left();

		}
		
	}
	
	
	/*
	*
	* Function Name: white_right()
	* Input:  void
	* Output: void
	* Logic:	Makes right turn for place in House 5
	* Example Call: white_right()
	*/
	void white_right(void)
	{	
		right();                          //  jerk in provided to increase the accuracy of right turn 
		_delay_ms(300);
		
		
		while (1)
		{
			
			right_sensor_value = ADC_Conversion(1);

			if ( right_sensor_value < 67   ) // As soon as left sensor finds a black line it breaks
			{
				stop();
				
				break;
			}

			velocity(170,170);
			right();

		}
		
		//adjust_white_left();
		left();
		_delay_ms(199);
	
	//right_mm(140);
	}
	
	
	/*
	*
	* Function Name: white_forward_wls()()
	* Input:  void
	* Output: void
	* Logic:	Makes the bot follow black line until the white line following segment starts.
	* Example Call: white_forward_wls()
	*/
	void white_forward_wls()// for black line following  
	{
		
		temp_node_counter = 0;	//initialized to 0 so that it can iterate till number of nodes to go forward

		 shaftcountleft = 0;
		 shaftcountRight = 0;
		while (1)
		{
				left_sensor_value = ADC_Conversion(3);
				middle_sensor_value = ADC_Conversion(2);
				right_sensor_value = ADC_Conversion(1);
				

				//Sensor values are assigned
				//node_detector(l, m,r);
				
				// node_detector function is called in each iteration to check for a node
				// node_detector increases node_counter if node is encountered
				if((left_sensor_value > cutoff+30 && middle_sensor_value < cutoff+30 && right_sensor_value > cutoff+30)||(left_sensor_value < cutoff+30 && middle_sensor_value > cutoff+30 && right_sensor_value > cutoff+30)
				||(left_sensor_value > cutoff+30 && middle_sensor_value > cutoff+30 && right_sensor_value < cutoff+30) && flag_v != 1 && (shaftcountleft > 150 || shaftcountRight > 150))
				{	
					//when white following segment comes
						//left();
						//_delay_ms(30);
						
						stop();
						
						//buzzer_on();
						white_line_follower('1');
						
						break;
					}
				
				
				// while  white line not come it follow black line
				
				if ( left_sensor_value < cutoff && middle_sensor_value > cutoff && right_sensor_value < cutoff )//As 255 value represents black line
				{
					forward();
					
				}

				else if ( right_sensor_value >  cutoff ) // To align the bot accordingly if it deviates from its path
				{
					soft_right();

				}

				else if ( left_sensor_value > cutoff) // To align the bot
				{
					soft_left();

				}

		}
		
		
	}
	
