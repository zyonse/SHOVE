/*Alexander Wilkinson - SHOVE MCP
  INPUT PINS:
     19 - rcChannel1 - INT2 - PD2
     18 - rcChannel2 - INT3
      2 - rcSwitch   - INT4
      3 - rcPot      - INT5
  OUTPUT PINS:
      6 - Left Motor Speed - OC4A - PH3
      8 - Left Motor Direction - PH5
      7 - Right Motor Speed - OC4B - PH4
      9 - Right Motor Direction - PH6
     22 - Walkie Talkie PTT - PA0
     
  RC Controller Settings:
    Mix Mode - B
    CH Mix - Yes
    Servo Reverser
      1 - Normal
      2 - Reverse
      3 - Normal
      4 - Normal
  RC Channel Functions:
      rcChannel1 - Channels 1 and 2 in mixed mode
      rcChannel2 - Reads the state of x and y Right stick
      rcSwitch   - 3 state switch - 1000 high, 1500 middle, 2000 low
      rcPot      - pot - 990 left -> 1985 right
  **When controller disconnects after use it seems to output a neutral stick
      so no need to measure timeout
*/
#define ABS(x) (x < 0 ? -(x) : x)

//RC Vars
uint16_t rcChannel1, rcChannel2, rcSwitch, rcPot;
#define RC_SAMPLE_SIZE 5 //number of samples for averaging rc readings
uint16_t rcChannel1Buffer[RC_SAMPLE_SIZE], rcChannel2Buffer[RC_SAMPLE_SIZE]; //rolling buffer for rc recordings
uint8_t rcChannel1Index = 0, rcChannel2Index = 0; //buffer index
void initRC();
uint8_t controllerConnected; //is the controller connected
#define RC_MIDDLE 1490
#define RC_DEADZONE 50 //radius of the deadzone

int16_t leftCommand, rightCommand; //set values from rc
float leftOutput, rightOutput; //OUTPUT Values -1000 -> 1000 -- 0 is stop

//Motor Right PWM on OC4B = PIN 7 //Motor Left PWM on OC4A = PIN6   
void initMotors();    //Motor takes a PWM signal for magnitude       
                      //   and a boolean value for the direction

//get an expected relationship between pwm and rpm for calculations
int16_t MAX_PWM; //Max output pwm
#define PWM_UPPER 800 //Range for max pwm output
#define PWM_LOWER 100
#define MAX_ACCELERATION 0.0028f //Max change in pwm per 4us
float MAX_INC; //Value for store max change for loop

void setup() {  
  //Timer for loop timing
  TCCR5A = 0;           //normal port operations
  TCCR5B = (1 << CS51) | (1 << CS50);   //clk/64 prescaler

  DDRA = 0x00; //Set up PORTA to be input with no pullup resistors
  PORTA = 0x00;

  initMotors();
  initRC();
}

void gatherRCData() {
  rcChannel1 = 0; //average last 5 recordings of rc width
  rcChannel2 = 0;
  for (uint8_t i = 0; i < RC_SAMPLE_SIZE; i++) {
    rcChannel1 += rcChannel1Buffer[i];
    rcChannel2 += rcChannel2Buffer[i]; 
  }
  rcChannel1 /= RC_SAMPLE_SIZE;
  rcChannel2 /= RC_SAMPLE_SIZE;
  MAX_PWM = map(rcPot, 990, 1985, PWM_LOWER, PWM_UPPER);
}

void computeMotorSpeed() {
  if ((rcChannel1 < RC_MIDDLE + RC_DEADZONE) && (rcChannel1 > RC_MIDDLE - RC_DEADZONE)) { //implement dead zone for sticks
    leftCommand = 0;
  } else {
    leftCommand = map(rcChannel1, 2100, 900, -MAX_PWM, MAX_PWM); //Map incoming signals to set point PWM Values
  }

  if ((rcChannel2 < RC_MIDDLE + RC_DEADZONE) && (rcChannel2 > RC_MIDDLE - RC_DEADZONE)) {
    rightCommand = 0;
  } else {
    rightCommand = map(rcChannel2, 2100, 900, -MAX_PWM, MAX_PWM);
  }
  
  MAX_INC = MAX_ACCELERATION * TCNT5; //Calculate maximum change in pwm val
  TCNT5 = 0; //Reset loop timer
}

void updateMotorOutputs() {
  if (ABS(leftCommand - leftOutput) < MAX_INC) { //max change in set point
    leftOutput = leftCommand;
  } else {
    if (leftCommand > leftOutput) {
      leftOutput += MAX_INC;
    } else {
      leftOutput -= MAX_INC;
    }
  }
    
  if (ABS(rightCommand - rightOutput) < MAX_INC) { //max change in set point
    rightOutput = rightCommand;
  } else {
    if (rightCommand > rightOutput) {
      rightOutput += MAX_INC;
    } else {
      rightOutput -= MAX_INC;
    }
  }

  if (ABS(leftOutput) > 1000) {
    if (leftOutput > 0) {
      leftOutput = 1000;
    } else {
      leftOutput = -1000;
    }
  }
  
  if (ABS(rightOutput) > 1000) {
    if (rightOutput > 0) {
      rightOutput = 1000;
    } else {
      rightOutput = -1000;
    }
  }

  if (!controllerConnected) { //if controller not connected
    leftOutput = 0;
    rightOutput = 0;
  }
    
  if (leftOutput >= 0) { //Set left motor speed and direction
    OCR4A = leftOutput;
    PORTH &= ~(1 << PH5); //Clr left dir
  }
  else {
    OCR4A = -leftOutput;
    PORTH |= (1 << PH5); //Set left dir
  }
  if (rightOutput >= 0) { //set right motor speed and direction
    OCR4B = rightOutput;
    PORTH &= ~(1 << PH6); //Clr right dir
  }
  else {
    OCR4B = -rightOutput;
    PORTH |= (1 << PH6); //Set right dir
  }  
}

void checkSwitch() {
  //read switch - rcSwitch
  if (rcSwitch < 1250) { //Top Position
    DDRA &= ~(1 << PA0); //Release Button - Shove Recieving
  } else { //Middle or Bottom Position
    DDRA |= (1 << PA0); //Press Button - Shove Transmitting
  }
}

void loop() {
  gatherRCData();
  computeMotorSpeed();
  updateMotorOutputs();
  checkSwitch();
}

//setup PWM for motors - OC4A and OC4B using PWM mode 14 - freqency = ~244Hz
void initMotors() {
  DDRH |= (1 << PH3) | (1 << PH4) | (1 << PH5) | (1 << PH6);  //Turn on output pins
  TCCR4A = 0; //reset registers
  TCCR4B = 0;
  TCCR4A |= (1 << COM4A1) | (1 << COM4B1) | (1 << WGM41);
  TCCR4B |= (1 << WGM43) | (1 << WGM42) | /*(1 << CS41) |*/ (1 << CS40); 
  ICR4 = 1000; //set max of 1000
  OCR4A = 0; //initialize 0% duty cycle
  OCR4B = 0;
}

//Setup external interrupts
void initRC() {
  DDRD &= ~((1 << PD2) | (1 << PD3)); //INT 2 and 3 as input
  DDRE &= ~((1 << PE4) | (1 << PE5)); //INT 4 and 5 as input
  //Initialize Timer for Reading - tick every 0.5us
  TCCR1A = 0;           //normal port operations
  TCCR1B = (1 << CS11);   //clk/8 prescaler
  //Initialize External Interrupts
  EICRA = (1 << ISC20) | (1 << ISC31); //INT2 Any Edge, INT3 Falling Edge
  EICRB = (1 << ISC41) | (1 << ISC51); //INT4 INT5 Falling Edge
  EIMSK = (1 << INT2) | (1 << INT3) | (1 << INT4) | (1 << INT5); //Enable external interrupt pins
  controllerConnected = 0; //Set RC as disconnected at start
  sei(); //Set interrupt bit
}

ISR(INT2_vect) { // Interrupt for rcChannel1
  if (PIND & (1 << PD2)) {
    TCNT1 = 0;
  } else {
    rcChannel1Index++;
    if (rcChannel1Index >= RC_SAMPLE_SIZE) {
      rcChannel1Index = 0;
    }
    rcChannel1Buffer[rcChannel1Index] = TCNT1 >> 1;
    controllerConnected = 1;
  }
}

ISR(INT3_vect) { // Interrupt for rcChannel2
  rcChannel2Index++;
  if (rcChannel2Index >= RC_SAMPLE_SIZE) {
    rcChannel2Index = 0;
  }
  rcChannel2Buffer[rcChannel2Index] = TCNT1 >> 1;
}

ISR(INT4_vect) { // Interrupt for rcSwitch
  rcSwitch = TCNT1 >> 1;
} 

ISR(INT5_vect) { // Interrupt for rcPot
  rcPot = TCNT1 >> 1;
}
