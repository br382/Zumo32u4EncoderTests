/*
  Zumo 32u4 (Pololu) encoder, angle, distance position recording tests.
  This is a re-implimentation of the existing Zumo libraries without
  referencing their source and using the Arduino IDE.
  To compile and burn to the Zumo 32u4, follow their instructions on connecting the Arduino IDE:
  Link: https://www.pololu.com/docs/0J63/5
  Reference for all-pins interrupt:
  http://www.geertlangereis.nl/Electronics/Pin_Change_Interrupts/PinChange_en.html
  http://thewanderingengineer.com/2014/08/11/arduino-pin-change-interrupts/
*/
#define WHEEL_DIAMETER 3.9 //outer-outer (cm)
#define COUNTS_PER_REVOLUTION (2.0*909.07) //fastest gear ratio
#define RAD2DEG (180.0/PI)
#define TRACK_SEP 8.5 // 8.5 center-center, 7.2 inner-inner, 10.0 outer-outer (cm)
#define XOR ^
#define FALSE 0
#define TRUE  1
#define HWB 31
double ENCODER_SCALE_FACTOR = PI * WHEEL_DIAMETER / COUNTS_PER_REVOLUTION; // cm per count
volatile double POS_X = 0.0, POS_Y = 0.0, HEADING = 0.0;
volatile bool r_now_a = FALSE;
volatile bool r_now_b = FALSE;
volatile bool r_last_a = FALSE;
volatile bool r_last_b = FALSE;
volatile bool l_now_a = FALSE;
volatile bool l_now_b = FALSE;
volatile bool l_last_a = FALSE;
volatile bool l_last_b = FALSE;
volatile unsigned int l_err = FALSE;
volatile unsigned int r_err = FALSE;
volatile long inc_R = 0;
volatile long inc_L = 0;


bool isError(bool A_last, bool A_now, bool B_last, bool B_now) { return ((A_last XOR A_now) && (B_last XOR B_now)); }


bool isForward(bool A_last, bool A_now, bool B_last, bool B_now)
{
  if ((A_last == TRUE)  && (A_now == TRUE)  && (B_last == FALSE) && (B_now == TRUE))  return(TRUE);
  if ((A_last == TRUE)  && (A_now == FALSE) && (B_last == TRUE)  && (B_now == TRUE))  return(TRUE);
  if ((A_last == FALSE) && (A_now == FALSE) && (B_last == TRUE)  && (B_now == FALSE)) return(TRUE);
  if ((A_last == FALSE) && (A_now == TRUE)  && (B_last == FALSE) && (B_now == FALSE)) return(TRUE);
  return FALSE;
}

bool isReverse(bool A_last, bool A_now, bool B_last, bool B_now)
{
  if ((A_last == FALSE) && (A_now == TRUE)  && (B_last == TRUE)  && (B_now == TRUE))  return(TRUE);
  if ((A_last == TRUE)  && (A_now == TRUE)  && (B_last == TRUE)  && (B_now == FALSE)) return(TRUE);
  if ((A_last == TRUE)  && (A_now == FALSE) && (B_last == FALSE) && (B_now == FALSE)) return(TRUE);
  if ((A_last == FALSE) && (A_now == FALSE) && (B_last == FALSE) && (B_now == TRUE))  return(TRUE);
  return FALSE;
}

void motorRightIsr()
{
  r_now_a = digitalRead(7);
  r_now_b = digitalRead(23);
  r_now_a = (r_now_a XOR r_now_b);
  r_err += isError(r_last_a, r_now_a, r_last_b, r_now_b);
  if(isForward(r_last_a, r_now_a, r_last_b, r_now_b)) inc_R -= 1;
  if(isReverse(r_last_a, r_now_a, r_last_b, r_now_b)) inc_R += 1;
  r_last_a = r_now_a;
  r_last_b = r_now_b;
}

void motorLeftIsr()
{
  l_now_a = digitalRead(8);
  l_now_b = digitalRead(HWB);
  l_now_a = (l_now_a XOR l_now_b);
  l_err += isError(l_last_a, l_now_a, l_last_b, l_now_b);
  if(isForward(l_last_a, l_now_a, l_last_b, l_now_b)) inc_L += 1;
  if(isReverse(l_last_a, l_now_a, l_last_b, l_now_b)) inc_L -= 1;
  l_last_a = l_now_a;
  l_last_b = l_now_b;
}

void setupPinChangeISR() {
    // configuring input pins wanted:
    pinMode(7, INPUT_PULLUP);
    pinMode(8, INPUT_PULLUP);
    pinMode(23, INPUT_PULLUP);
    pinMode(HWB, INPUT_PULLUP);
    
    cli(); //disable interrupts for configuring their settings
    //set ports that allow interrupting...
    //PCICR |= 0b00000001; //turn on PORTB
    //PCICR |= 0b00000010; //turn on PORTC
    //PCICR |= 0b00000100; //turn on PORTD
    PCICR |= 0b00000111; //turn on all PORT
    
    //set pins that allow interrupting...
    //PCMSK0 |= 0b00000001; // turn on pin PB0, which is on PCINT0, physical pin14
    //PCMSK1 |= 0b00010000; // turn on pin PC4, which is PCINT12, physical pin27
    //PCMSK2 |= 0b10000001; // turn on pins PD0 and PD7, PCINT16 and PCINT23
    PCMSK0 |= 0b11111111;
    PCMSK1 |= 0b11111111;
    PCMSK2 |= 0b11111111;
    sei(); //enable interrupts after config is done.
}

void pinChageISR() {//run on any pin changes
    motorRightIsr();
    motorLeftIsr();
}

// The Separate ISR bus callbacks defined:
ISR(PCINT0_vect) { //PORTB, PCINT0 - PCINT7
    pinChageISR();
}
ISR(PCINT1_vect) { //PORTC, PCINT8 - PCINT14
    pinChageISR();
}
ISR(PCINT2_vect) { //PORTD, PCINT16 - PCINT23
    pinChageISR();
}

double getDisplacement(){ return((inc_L + inc_R) * ENCODER_SCALE_FACTOR / 2.0); }

double getRotation(){ return ((inc_L - inc_R) * ENCODER_SCALE_FACTOR / TRACK_SEP); }

void setPos()
{
  long c_l = inc_L;
  long c_r = inc_R;
  double count_l = double(c_l);
  double count_r = double(c_r);
  //double count_l = count_r; verify distance while missing a left encoder
  double displacement  = ((count_l + count_r) * ENCODER_SCALE_FACTOR / 2.0);
  double rotation      = ((count_l - count_r) * ENCODER_SCALE_FACTOR / TRACK_SEP);
  HEADING = HEADING + rotation;
  if (HEADING >  2.0 * PI) HEADING = (HEADING - (2.0*PI));
  if (HEADING < 0) HEADING = (HEADING + (2.0*PI));
  POS_X = POS_X + displacement * cos(HEADING / 2.0);
  POS_Y = POS_Y + displacement * sin(HEADING / 2.0);
  inc_L -= c_l;
  inc_R -= c_r;
}


void setup(void) {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  //INPUT_PULLUP
  /*
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);
  pinMode(HWB, INPUT_PULLUP);
  //Unfortunately only the pin7 Hardware interrupt exists on leonardo (pin8 can't work this way)
  attachInterrupt(digitalPinToInterrupt(7), motorRightIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(8), motorLeftIsr, CHANGE);
  */

  //Motor Drive settings:
  // LEFT                 RIGHT
  pinMode(10, OUTPUT);      pinMode(9,  OUTPUT);
  pinMode(16, OUTPUT);      pinMode(15, OUTPUT);
  digitalWrite(10, FALSE); digitalWrite(9,  FALSE); //FALSE = 0%, TRUE = 100% power (PWM)
  digitalWrite(16, LOW); digitalWrite(15, LOW); //LOW -> FORWARD, HIGH -> REVERSE
  
  setupPinChangeISR(); //enable global ISR() bus handler(s)
}


void loop() {
  /*
  setPos();
  Serial.print("X,Y,Theta ");
  Serial.print(POS_X);   Serial.print(", ");
  Serial.print(POS_Y);   Serial.print(", ");
  Serial.print(HEADING * RAD2DEG); Serial.print(".     ");
  */
  Serial.print("Left Motor: ");
  Serial.print(inc_L); Serial.print("  Err: ");
  Serial.print(l_err);
  Serial.print("        Right Motor: ");
  Serial.print(inc_R); Serial.print("  Err: ");
  Serial.println(r_err);
  digitalWrite(13, !digitalRead(13));
  
}

