/* Authors:
            Tero Kultanen
            Jarkko Ruoho
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "TimerOne.h"

// Pin names
#define IC_CLR      2 // Input, Current limit active
#define RPM         3 // Input, Motor speed sensor (Hall A_IN signal)
#define ENABLE      4 // Output, Enable sensor pull-ups
#define DIRECTION   5 // Output, Motor rotating direction (actual direction depends on phase order)
#define PWM_STATOR  9 // Output, PWM, Stator voltage control PWM output
#define PWM_FIELD  10 // Output, PWM, Field voltage control PWM output
#define AI_AVG_I   A2 // Input, Analog, DC-link average current (Scale: delta_512=delta_90A, Offset: 512=0A)
#define AI_TEMP    A4 // Input, Analog, Thermistor
#define AI_VBATT   A5 // Input, Analog, Battery voltage (Scale: 1024 = 66V)
#define AI_TR2     A6 // Input, Analog, TR2 (Klippo: throttle/break)
#define AI_TR1     A7 // Input, Analog, TR1 (Klippo: actual trimmer, not used)
#define DEBUG_OUT   6 // Output, Debug output, used for loop time measurement signal
#define LED        13 // Output, Arduino Nano Led

// Parameters
#define THR_LOW_V       1.0 //Throttle input 0% level
#define THR_HIGH_V      4.0 //Throttle input 100% level
#define FIELD_MAX_V     4000 //Maximum Field voltage
#define FIELD_LOW_V     2000 //Maximum Field voltage
#define FIELD_HIGH_V    3000 //Maximum Field voltage
#define FIELD_HH_V      3500 //Maximum Field voltage
#define FIELD_DEFAULT_V 2500 //Maximum Field voltage
#define FIELD_MIN_V     2000 //Maximum Field voltage
#define I_MAX 40 //Software limited maximum phase current (with 0,8333mohm shunt)
#define OVLO_V 60 //Over voltage lockout threshold
#define UVLO_V 20 //Under voltage lockout threshold
#define TEMP_60C 576 //
#define TEMP_70C 593 //
#define OTLO TEMP_60C //Heatsink over temperature lockout
//60deg-C = 576, 70deg_C = 593
  // TODO: Read proper speed setting with serial println
// 2660 rpm == noin 88 speedCount
#define DEFAULT_SPEED 75

//Scaled parameters
int THR_LOW = (float)THR_LOW_V*1024/5; //Throttle input 0% level (Scale: 1024=5V)
int THR_HIGH = (float)THR_HIGH_V*1024/5; //Throttle input 100% level (Scale: 1024=5V)
int FIELD_DEFAULT = (float)FIELD_DEFAULT_V/66; //Maximum Field voltage
int FIELD_HIGH    = (float)FIELD_HIGH_V/66; //Maximum Field voltage
int FIELD_HH      = (float)FIELD_HH_V/66; //Maximum Field voltage
int FIELD_LOW     = (float)FIELD_LOW_V/66; //Maximum Field voltage
int OVLO = (float)OVLO_V/66*1024;
int UVLO = (float)UVLO_V/66*1024;
#define OVLO_mV 60000ull // UVLO in mV, 60 000 mV
#define UVLO_mV 20500ull // UVLO in mV, this should be accurate 20.0 V

#define MAX_THROTTLE 900

// Variables
int  Field_out; // Rotor PWM signal
char x = 0; // Loop time measurement toggle signal
int  Stator_out = 0; // Stator PWM signal
volatile unsigned long HW_OC = 0; //HW_OC Signal is set by interrupt if hardware has sensed an over-current event

int speedSet   = DEFAULT_SPEED;

void ISR_hall_triggered();
void ISR_over_current_sensed();

int getTR(int);
int getCurrent_A();
unsigned long long getVoltage_mV();
int getTemperature_C();

int throttleCtrl();
int getHallPeriod();

volatile unsigned char timerCnt   = 0;
volatile unsigned char preIncrCnt = 0;
volatile unsigned int  hallCnt    = 0;


#define RPMS 16
volatile unsigned char rpmPtr;
volatile int  rpmSensor[RPMS];

void setup() {
  // put your setup code here, to run once:
  pinMode(RPM, INPUT);
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
  pinMode(DEBUG_OUT, OUTPUT);
  digitalWrite(DEBUG_OUT, LOW);
  pinMode(DIRECTION, OUTPUT);
  //  digitalWrite(DIRECTION, HIGH);
  digitalWrite(DIRECTION, LOW);
  pinMode(PWM_STATOR, OUTPUT);
  pinMode(PWM_FIELD, OUTPUT);
  pinMode(AI_TR1, INPUT);
  pinMode(AI_TR2, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  attachInterrupt(digitalPinToInterrupt(RPM), ISR_hall_triggered, CHANGE); //Enables interrupt handling for speed measurement when Hall A is falling
  
  attachInterrupt(digitalPinToInterrupt(IC_CLR), ISR_over_current_sensed, FALLING); //Enables interrupt handling if overcurrent is sensed by hardware

  memset((void*)rpmSensor, 0, RPMS*sizeof(int));
  rpmPtr = 0;
  
  Timer1.initialize(55); // initialize timer1, and set a 55 u_second period

  //Initialize Timer2 for speed measurement

  // TCCR2A |= (1 << WGM20); // CTC Mode, we want to count upwards into infinity (with roll over 255 => 0)
  // normal mode (=0) should also work, but it was unstable
  TCCR2A |= (1 << WGM20) | (1 << WGM21); // Fast PWM Mode, we want to count only upwards into infinity (with roll over 255 => 0)
  TIMSK2 |= (1 << TOIE2); //Overflow interrupt
  OCR2A = 250; // Compare register value
  TIMSK2 |= (1 << OCIE2A); //Set interrupt on compare match
  TCCR2B = 7; // set prescaler to 1024 and starts PWM, 64 us per step
  
  //  Serial.begin(9600);
  Serial.begin(250000);
  Serial.println(TCCR2B, BIN);
  Serial.println(OCR2A, BIN);

  Serial.print("Battery ");
  Serial.print((unsigned int)getVoltage_mV(), DEC);
  Serial.println(" mV");

  Serial.print("Temperature ");
  Serial.print(getTemperature_C(), DEC);
  Serial.println(" ru (Random Unit)");

  Serial.print("AVG current ");
  Serial.print( getCurrent_A(), DEC);
  Serial.println(" A");

  /*  Serial.println("Write speed (3 digits):");
  int digit = 0;
  for (int i=0; i<3; i++) {
    int read = -1;
    while (read == -1)
      read = Serial.read();
    digit = digit*10 + read;
  }
  Serial.println("Got speed:");
  Serial.println(digit, DEC);

  if ((digit < 250) && (digit > 50))
    speedSet = digit;
  else
  */
  speedSet = DEFAULT_SPEED;

  interrupts(); // enable interrupts

}

int getTR(int i){
  int trimVoltage;
  if (i == 2) {
    trimVoltage = analogRead(AI_TR2);
  } else {
    trimVoltage = analogRead(AI_TR1);
  }
  return trimVoltage; // 0..1023
}

int getCurrent_A(){
  int avgCurrent = analogRead(AI_AVG_I)-512;
  avgCurrent = (avgCurrent * 90) / 512 ; // magic according to Tero's comment
  return avgCurrent;
}

unsigned long long getVoltage_mV(){
  unsigned long long volt;
  volt  = analogRead(AI_VBATT);
  volt *= 64;
  return volt;
}
int getTemperature_C(){
  int temp;
  temp = analogRead(AI_TEMP);
  // TEMP_60C == 576 == 60 deg C, 2 step per deg C
  temp -= TEMP_60C;
  temp /= 2;
  temp += 60;
  return temp;
}


int emergencyStop(int blinkCount) {
  int i;
  digitalWrite(ENABLE, LOW);
  Timer1.pwm(PWM_STATOR, 0);
  Timer1.pwm(PWM_FIELD, 0);
  // Blink loop forever!
  Serial.println("Emergency stop"); 
  Serial.println(blinkCount, DEC); 

  Serial.print("Batt ");
  Serial.print((unsigned int)getVoltage_mV(), DEC);
  Serial.println(" mV");
  Serial.print("I_avg ");
  Serial.print(getCurrent_A(), DEC);
  Serial.println(" mV");
  Serial.print("Temp ");
  Serial.print(getTemperature_C(), DEC);
  Serial.println(" ru (Random Unit)");
  Serial.print("OC Count ");
  Serial.print(HW_OC, DEC);
  Serial.println("");

  while(1) {
    for (i=0;i<blinkCount;i++) {
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
    }   
    delay(1000);
  } 
} 

void printStatus(int timerCnt, int speedCount, int currentThrottle) {

  switch (timerCnt%128) {
  case 0: 
    Serial.println("");
    break;
  case 7: 
    Serial.print("Batt ");
    Serial.print((unsigned int)getVoltage_mV(), DEC);
    Serial.println(" mV");
    break;
  case 9: 
    Serial.print("Current ");
    Serial.print(getCurrent_A(), DEC);
    Serial.println("");
    break;
    /*
    case 9: 
      Serial.print("TR1 ");
      Serial.print(getTR(1), HEX);
      Serial.println("");
      break;
    case 11: 
      Serial.print("TR2 ");
      Serial.print(getTR(2), HEX);
      Serial.println("");
      break;
    case 13:
      Serial.print("Temp ");
      Serial.print(getTemperature_C(), DEC);
      Serial.println(" ru (Random Unit)");
      break;
    case 15:
      Serial.print("Tmr Count ");
      Serial.print(timerCnt, DEC);
      Serial.println("");
      break;
    case 17:
      Serial.print("Hall ISR Count ");
      Serial.print(hallCnt, DEC);
      Serial.println("");
      break;
      */
  case 19:
    Serial.print("OC Count ");
    Serial.print(HW_OC, DEC);
    Serial.println("");
    break;
  case 21:
    Serial.print("Throttle ");
    Serial.print(currentThrottle, DEC);
    Serial.print(" at speed ");
    Serial.print(speedCount, DEC);
    Serial.println("");
    break;      
  case 23:
    Serial.print("Enable ");
    Serial.print(digitalRead(ENABLE), DEC);
    Serial.println("");
    break;      
  default:
    break;
  }
  
}


void loop() {
  static int Throttle = 0; // Stator PWM 
  int momentReq = 0; // Moment decrement or increment request
  int handleOn = 0;  // Throttle on or off (break)
  int speedCount;    // Motor rotating speed value
  
  static unsigned char lTimerCnt = 0;
  static unsigned char breakCnt  = 0;

  speedCount = getHallPeriod();  
  
  if (getTR(2) >= 0x3F0) {
    handleOn = 1;
    digitalWrite(ENABLE, HIGH);
  } else {
    handleOn = 0;
  }
  
  // Try to run throttleCtrl() once per Timer2 overflow. If we are slower, then we are.
  if (lTimerCnt != timerCnt) {
    lTimerCnt = timerCnt;    
    attachInterrupt(digitalPinToInterrupt(IC_CLR), ISR_over_current_sensed, FALLING); //Enables interrupt handling if overcurrent is sensed by hardware
    
    if (handleOn == 1) {
      Throttle  = throttleCtrl(speedSet, speedCount);
    } else { 
      Throttle = 0;
    }
    
    printStatus(timerCnt, speedCount, Throttle);
    
  } 
  
  Stator_out = Throttle;

  if (Stator_out > 950)
    Stator_out = 950;
    
  unsigned long long BatteryVoltage_mV = getVoltage_mV();

  if (BatteryVoltage_mV > OVLO_mV) {
    emergencyStop(3);
  }
  if (BatteryVoltage_mV < UVLO_mV) {
    emergencyStop(1);
  }

  int SinkTemperature = analogRead(AI_TEMP);

  if (SinkTemperature > OTLO) {
    emergencyStop(6);
  }

  if (handleOn == 0) {
    // Break harder if user has released the handle
    if (speedCount < 255) {
      momentReq = 1;
      breakCnt = timerCnt;
    } else if (((timerCnt+1)&0x7f) == (breakCnt&0x7f)) {
      // Just save some power
      momentReq = -1;
      digitalWrite(ENABLE, LOW);
    } else {
      momentReq = -1;
    }
  } else if (Throttle > 900) {
    momentReq = 1;
  } else if (Throttle > 945) {
    momentReq = 2;
  } else if (Throttle < 400) {
    momentReq = -1;
  }

  // TODO: Consider limiting Stator_out in case of high average current.
  // Maybe increment momentReq 
  if (getCurrent_A() > I_MAX) {
    momentReq++;
  }
  
  
  // Adjust Field voltage according to throttle level
  switch (momentReq) {
  case -1: // Idling => save power
    Field_out = (float)FIELD_LOW / BatteryVoltage_mV;
    break;
  case  1:
    Field_out = (float)FIELD_HIGH / BatteryVoltage_mV;
    break;
  case  2:
  case  3:
    Field_out = (float)FIELD_HH / BatteryVoltage_mV;
    break;
  default:
    Field_out = (float)FIELD_DEFAULT / BatteryVoltage_mV;
    break;
  }

  Timer1.pwm(PWM_STATOR, Stator_out);
  Timer1.pwm(PWM_FIELD, Field_out);
  //TODO: Use the following (it's faster)
  //Timer1.setPwmDuty(PWM_STATOR, Stator_out);
  //Timer1.setPwmDuty(PWM_FIELD, Field_out);

  //  delay(10);
}

void ISR_over_current_sensed() {
  HW_OC++;
  // Maximum count rate is 61 counts per second
  detachInterrupt(digitalPinToInterrupt(IC_CLR));
}


// Return throttle value in range 0..1023
int throttleCtrl( int targetSpeed, int speedCount ) {
  static int throttleI     = 0;
  int throttleP            = 0;
  int throttleD            = 0;
  int throttle             = 0;
  int SpeedError           = 0;
  static int SpeedErrorOld = 0;
   
  SpeedError = speedCount - targetSpeed;
  
  // TODO: Consider throttleI limiting during startup

  throttleI = throttleI + SpeedError;
  throttleI /=2;
  throttleP = SpeedError*64;
  throttleD = SpeedErrorOld - SpeedError;
  throttleD *= 32;
  
  SpeedErrorOld = SpeedError;

  if (throttleI > MAX_THROTTLE)
    throttleI = MAX_THROTTLE;
  if (throttleI < -MAX_THROTTLE)
    throttleI = MAX_THROTTLE;

  if (throttleP > MAX_THROTTLE)
    throttleP = MAX_THROTTLE;
  if (throttleP < -MAX_THROTTLE)
    throttleP = MAX_THROTTLE;

  if (throttleD > MAX_THROTTLE)
    throttleD = MAX_THROTTLE;
  if (throttleD < -MAX_THROTTLE)
    throttleD = MAX_THROTTLE;

  throttle = throttleI + throttleP + throttleD;
  if (throttle > MAX_THROTTLE)
    throttle = MAX_THROTTLE;
  if (throttle < 0)
    throttle = 0;

  if (targetSpeed == 0) {
    throttle  = 0;
    throttleI = 0;
    throttleP = 0;
    throttleD = 0;
  }   
  
  return throttle;
}

// Hall A signal interrupt handling for speed measurement
int getHallPeriod() {

  int periodArray[RPMS/2];
  static int speedPeriod = 0;
  static unsigned char localPtr;
  int i, j;

  if (localPtr == rpmPtr) {
    if (speedPeriod < 255) {
      speedPeriod++;
    }    
    return speedPeriod;
  }
  
  // take the current pointer - it could be updated during this function
  localPtr = rpmPtr;

  unsigned int newC = localPtr;
  unsigned int oldC = 0;

  // Calculate period based on rising and falling edge periods. Calculate
  // multiple periods in order to support filtering.
  for (i = 0; i<RPMS/2; i++) {
    newC = localPtr-i;
    newC %= RPMS;
    oldC = newC-2;
    oldC %= RPMS; 
    periodArray[i] = rpmSensor[newC] - rpmSensor[oldC];
    if ((periodArray[i] < 0) || (periodArray[i] > 1023)) {
      periodArray[i] = DEFAULT_SPEED;
    }
  }

  // Sort speedPeriods from lowest to highest
  for (j = 0; j < RPMS/2-1; j++) {
    for (i = 0; i < (RPMS/2-j)-1; i++) {
      // -10 causes values near zero to be interpetted as very large values (slow). 0 you can see in startup at least.
      if ((periodArray[i]-10) > (periodArray[i+1]-10)) {
	speedPeriod       = periodArray[i];
	periodArray[i]   = periodArray[i+1];
	periodArray[i+1] = speedPeriod;
      }
    }
  }

  // Take the average from center:
  // 1. dropping two slowest and two fastest samples
  // 2. Calculate the average
  speedPeriod = 0;
  for (i = 2; i < (RPMS/2-2); i++) {
    speedPeriod += periodArray[i];
  }
  speedPeriod /= ((RPMS/2) - 4);
  
  // Store the lastest value. If there are no valid input from hall, assume near stop condition
  if ((speedPeriod > 255) ||
      (speedPeriod < 10)) {
    speedPeriod = 255;
  }
  
  return speedPeriod;

}

// Hall A signal falling interrupt handler for speed measurement
void ISR_hall_triggered() {
  int tmp = (unsigned int)TCNT2; // Sample the counter value
  static int lTmp = 0;
  
  if (tmp < 5)  // With small TCNT2 values it is safer to use pre incremented counter value (othervise we'd have to be sure about interrupt order)
    tmp += ((int)preIncrCnt<<8);
  else
    tmp += ((int)timerCnt<<8);
  hallCnt++; // Count this interrupt for debuging
  if ((int)(tmp - lTmp) > 20) {
    rpmPtr++;
    rpmPtr %= RPMS;
    rpmSensor[rpmPtr] = tmp;
    //Generate measurement toggle signal
  }
  lTmp = tmp;
}

//Timer2 compare match interrupt handling
ISR (TIMER2_COMPA_vect)
{
  preIncrCnt++;
}

//Timer2 compare match interrupt handling
ISR (TIMER2_OVF_vect)
{
  timerCnt++;
}
