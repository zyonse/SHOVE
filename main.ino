/*Alexander Wilkinson - SHOVE MCP
  INPUT PINS:
     19 - RC 1 - INT2 - PD2
     18 - RC 2 - INT3
      2 - RC 5 - INT4
      3 - RC 6 - INT5
     A0 - Battery Voltage Divider
  OUTPUT PINS:
      6 - Left Motor Speed - OC4A - PH3
      8 - Left Motor Direction - PH5
      7 - Right Motor Speed - OC4B - PH4
      9 - Right Motor Direction - PH6
     23 - Low battery LED - PA1
     
  RC Controller Settings:
    Mix Mode - B
    CH Mix - Yes
    Servo Reverser
      1 - Normal
      2 - Reverse
      3 - Normal
      4 - Normal
  RC Channel Functions:
      RC1  - Channels 1 and 2 in mixed mode
      RC2       - Reads the state of x and y Right stick
      RC5  - 3 state switch - 1000 high, 1500 middle, 2000 low
      RC6  - pot - 990 left -> 1985 right
  **When controller disconnects after use it seems to output a neutral stick
      so no need to measure timeout
*/
#define ABS(x) (x<0?-(x):x)

//RC Vars
uint16_t rc1, rc2, rc5, rc6;
#define RCS 5 //number of samples for averaging rc readings
uint16_t rc1b[RCS], rc2b[RCS]; //rolling buffer for rc recordings
uint8_t rc1i = 0, rc2i = 0; //buffer index
void initRC();
uint8_t ctrlConn; //is the controller connected
#define MIDDLE 1490
#define DEADZONE 50 //radius of the deadzone

int16_t sL, sR; //set values from rc
float oL, oR; //OUTPUT Values -1000 -> 1000 -- 0 is stop

//Motor Right PWM on OC4B = PIN 7 //Motor Left PWM on OC4A = PIN6   
void initMotors();    //Motor takes a PWM signal for magnitude       
                      //   and a boolean value for the direction

/*void initADC(); //Voltage divider = 5V - R1 - A0 - R2 - 480V
uint16_t batt; //10 bit ADC reading - R1=810k R2=270k
#define LOW_BAT_VAL 608 // Call 11.9V low bat - 1023*(11.9*(R2/(R1+R2)))/5*/

//get an expected relationship between pwm and rpm for calculations
int16_t MAX_PWM; //Max output pwm
#define PWM_UPPER 800 //Range for max pwm output
#define PWM_LOWER 100
#define MAX_ACC 0.0028f //Max change in pwm per 4us
float MAX_INC; //Value for store max change for loop

void setup() {  
  //Timer for loop timing
  TCCR5A = 0;           //normal port operations
  TCCR5B = (1<<CS51) | (1<<CS50);   //clk/64 prescaler

  /*DDRA |= (1<<PA1); //Output for low battery led
  PORTA &= ~(1<<PA1); */

  initMotors();
  initRC();
  //initADC();
}

void loop() {
  rc1 = 0; //average last 5 recordings of rc width
  rc2 = 0;
  for (uint8_t i = 0; i < RCS; i ++) {
    rc1 += rc1b[i];
    rc2 += rc2b[i]; 
  }
  rc1 /= RCS;
  rc2 /= RCS;
  MAX_PWM = map(rc6, 990, 1985, PWM_LOWER, PWM_UPPER);

  if ((rc1 < MIDDLE + DEADZONE) && (rc1 > MIDDLE - DEADZONE)) //implement dead zone for sticks
    sL = 0;
  else
    sL = map(rc1, 2100, 900, -MAX_PWM, MAX_PWM); //Map incoming signals to set point PWM Values

  if ((rc2 < MIDDLE + DEADZONE) && (rc2 > MIDDLE - DEADZONE))
    sR = 0;
  else
    sR = map(rc2, 2100, 900, -MAX_PWM, MAX_PWM);
  
  MAX_INC = MAX_ACC*TCNT5; //Calculate maximum change in pwm val
  TCNT5 = 0; //Reset loop timer

  if (ABS(sL - oL) < MAX_INC) //max change in set point
    oL = sL;
  else 
    oL += (sL>oL ? MAX_INC : -MAX_INC);
    
  if (ABS(sR - oR) < MAX_INC) //max change in set point
    oR = sR;
  else 
    oR += (sR>oR ? MAX_INC : -MAX_INC);

  if (ABS(oL) > 1000)
    oL = oL>0?1000:-1000;
  if (ABS(oR) > 1000)
    oR = oR>0?1000:-1000;

  if (!ctrlConn) { //if controller not connected
    oL = 0;
    oR = 0;
  }
    
  if (oL >= 0) { //Set left motor speed and direction
    OCR4A = oL;
    PORTH &= ~(1<<PH5); //Clr left dir
  }
  else {
    OCR4A = -oL;
    PORTH |= (1<<PH5); //Set left dir
  }
  if (oR >= 0) { //set right motor speed and direction
    OCR4B = oR;
    PORTH &= ~(1<<PH6); //Clr right dir
  }
  else {
    OCR4B = -oR;
    PORTH |= (1<<PH6); //Set right dir
  }  

  /*
  if (batt<LOW_BAT_VAL) //if battery below threshold
    PORTA |= (1<<PA1); //turn on low battery indicator
  */

  /*  
  //read switch - rc5
  if (rc5 < 1250) //Top Position

  else if (rc5 < 1750) //Middle Position

  else //Bottom Position
  */
}

//setup PWM for motors - OC4A and OC4B using PWM mode 14 - freqency = ~244Hz
void initMotors() {
  DDRH |= (1<<PH3) | (1<<PH4) | (1<<PH5) | (1<<PH6);  //Turn on output pins
  TCCR4A = 0; //reset registers
  TCCR4B = 0;
  TCCR4A |= (1<<COM4A1) | (1<<COM4B1) | (1<<WGM41);
  TCCR4B |= (1<<WGM43) | (1<<WGM42) | /*(1<<CS41) |*/ (1<<CS40); 
  ICR4 = 1000; //set max of 1000
  OCR4A = 0; //initialize 0% duty cycle
  OCR4B = 0;
}

//Setup external interrupts
void initRC() {
  DDRD &= ~((1<<PD2) | (1<<PD3)); //INT 2 and 3 as input
  DDRE &= ~((1<<PE4) | (1<<PE5)); //INT 4 and 5 as input
  //Initialize Timer for Reading - tick every 0.5us
  TCCR1A = 0;           //normal port operations
  TCCR1B = (1<<CS11);   //clk/8 prescaler
  //Initialize External Interrupts
  EICRA = (1<<ISC20) | (1<<ISC31); //INT2 Any Edge, INT3 Falling Edge
  EICRB = (1<<ISC41) | (1<<ISC51); //INT4 INT5 Falling Edge
  EIMSK = (1<<INT2) | (1<<INT3) | (1<<INT4) | (1<<INT5); //Enable external interrupt pins
  ctrlConn = 0; //Set RC as disconnected at start
  sei(); //Set interrupt bit
}

ISR(INT2_vect) { //RC1
  if (PIND&(1<<PD2))
    TCNT1 = 0;
  else {
    rc1b[++rc1i<RCS?rc1i:(rc1i=0)] = TCNT1 >> 1;
    ctrlConn = 1;
  }
}

ISR(INT3_vect) { //RC2
  rc2b[++rc2i<RCS?rc2i:(rc2i=0)] = TCNT1 >> 1;
}

/*
ISR(INT4_vect) { //RC5
  rc5 = TCNT1 >> 1;
} 
*/

ISR(INT5_vect) { //RC6
  rc6 = TCNT1 >> 1;
}

/*
void initADC(void) {
  ADMUX = (1 << REFS0);  //Set the reference to AVCC and channel 0
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //set timing to clk/128
  ADCSRA |= (1 << ADIE) | (1 << ADEN); //enable adc and adc interrupt
  ADCSRA |= (1 << ADSC); //start ADC conversion
}

ISR(ADC_vect) { //Adc read interrupt
  batt = 0;
  batt = ADCL;       //get the lower 8-bits
  batt |= ADCH << 8;     //get the upper 2-bits
  ADCSRA |= (1<<ADSC);      //start ADC conversion
}
*/
