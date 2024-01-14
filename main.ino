#define ABS(x) (x<0?-(x):x)

uint16_t rc1, rc2, rc5, rc6;
#define RCS 5
uint16_t rc1b[RCS], rc2b[RCS];
uint8_t rc1i = 0, rc2i = 0; 
void initRC();
uint8_t ctrlConn;
#define MIDDLE 1490
#define DEADZONE 50

int16_t sL, sR;
float oL, oR;
 
void initMotors();
int16_t MAX_PWM;
#define PWM_UPPER 800
#define PWM_LOWER 100
#define MAX_ACC 0.0028f
float MAX_INC;

void setup() {  
  TCCR5A = 0;
  TCCR5B = (1<<CS51) | (1<<CS50); 

  initMotors();
  initRC();
}

void loop() {
  rc1 = 0;
  rc2 = 0;
  for (uint8_t i = 0; i < RCS; i ++) {
    rc1 += rc1b[i];
    rc2 += rc2b[i]; 
  }
  rc1 /= RCS;
  rc2 /= RCS;
  MAX_PWM = map(rc6, 990, 1985, PWM_LOWER, PWM_UPPER);

  if ((rc1 < MIDDLE + DEADZONE) && (rc1 > MIDDLE - DEADZONE))
    sL = 0;
  else
    sL = map(rc1, 2100, 900, -MAX_PWM, MAX_PWM);

  if ((rc2 < MIDDLE + DEADZONE) && (rc2 > MIDDLE - DEADZONE))
    sR = 0;
  else
    sR = map(rc2, 2100, 900, -MAX_PWM, MAX_PWM);
  
  MAX_INC = MAX_ACC*TCNT5;
  TCNT5 = 0;

  if (ABS(sL - oL) < MAX_INC)
    oL = sL;
  else 
    oL += (sL>oL ? MAX_INC : -MAX_INC);
    
  if (ABS(sR - oR) < MAX_INC)
    oR = sR;
  else 
    oR += (sR>oR ? MAX_INC : -MAX_INC);

  if (ABS(oL) > 1000)
    oL = oL>0?1000:-1000;
  if (ABS(oR) > 1000)
    oR = oR>0?1000:-1000;

  if (!ctrlConn) {
    oL = 0;
    oR = 0;
  }
    
  if (oL >= 0) {
    OCR4A = oL;
    PORTH &= ~(1<<PH5);
  }
  else {
    OCR4A = -oL;
    PORTH |= (1<<PH5);
  }
  if (oR >= 0) {
    OCR4B = oR;
    PORTH &= ~(1<<PH6);
  }
  else {
    OCR4B = -oR;
    PORTH |= (1<<PH6);
  } 

void initMotors() {
  DDRH |= (1<<PH3) | (1<<PH4) | (1<<PH5) | (1<<PH6);
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4A |= (1<<COM4A1) | (1<<COM4B1) | (1<<WGM41);
  TCCR4B |= (1<<WGM43) | (1<<WGM42) | (1<<CS40); 
  ICR4 = 1000;
  OCR4A = 0; 
  OCR4B = 0;
}

void initRC() {
  DDRD &= ~((1<<PD2) | (1<<PD3));
  DDRE &= ~((1<<PE4) | (1<<PE5));
  
  TCCR1A = 0;
  TCCR1B = (1<<CS11);
  
  EICRA = (1<<ISC20) | (1<<ISC31);
  EICRB = (1<<ISC41) | (1<<ISC51); 
  EIMSK = (1<<INT2) | (1<<INT3) | (1<<INT4) | (1<<INT5);
  ctrlConn = 0;
  sei();
}

ISR(INT2_vect) {
  if (PIND&(1<<PD2))
    TCNT1 = 0;
  else {
    rc1b[++rc1i<RCS?rc1i:(rc1i=0)] = TCNT1 >> 1;
    ctrlConn = 1;
  }
}

ISR(INT3_vect) {
  rc2b[++rc2i<RCS?rc2i:(rc2i=0)] = TCNT1 >> 1;
}

ISR(INT5_vect) {
  rc6 = TCNT1 >> 1;
}

