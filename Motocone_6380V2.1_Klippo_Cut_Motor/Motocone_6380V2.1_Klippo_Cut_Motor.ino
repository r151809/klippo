/* Authors:
            Tero Kultanen
            Jarkko Ruoho
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "TimerOne.h"

// Pin names
#define IC_CLR        2 // Input, Current limit active
#define RPM           3 // Input, Motor speed sensor (Hall A_IN signal)
#define ENABLE        4 // Output, Enable sensor pull-ups
#define DIRECTION     5 // Output, Motor rotating direction (actual direction depends on phase order)
#define PWM_STATOR    9 // Output, PWM, Stator voltage control PWM output
#define PWM_FIELD    10 // Output, PWM, Field voltage control PWM output
#define AI_AVG_I     A2 // Input, Analog, DC-link average current (Scale: delta_512=delta_90A, Offset: 512=0A)
#define AI_TEMP      A4 // Input, Analog, Thermistor
#define AI_VBATT     A5 // Input, Analog, Battery voltage (Scale: 1024 = 66V)
#define AI_TR2       A6 // Input, Analog, TR2 (Klippo: throttle/break)
#define AI_TR1       A7 // Input, Analog, TR1 (Klippo: actual trimmer, not used)
#define DEBUG_OUT     6 // Output, Debug output, used for loop time measurement signal
#define LED          13 // Output, Arduino Nano Led, External LED,        Gray   cable
#define EXT_LED_EN   12 // Output, Other side for external LED in switch, Violet cable
#define EXT_LED_EN_N 11 // Blue
#define DEBUG2        8 // Green
#define DEBUG3        7 // Yellow
#define DEBUG4        6 // Orange                 



// Parameters
#define THR_LOW_V       1.0 //Throttle input 0% level
#define THR_HIGH_V      4.0 //Throttle input 100% level
#define FIELD_DEFAULT_V 7100 //Maximum Field voltage
#define FIELD_BREAK_V   2000 //Maximum Field voltage
#define FIELD_PWM_CORRECTION 1080 // Multiplication value to ouput Volts
#define I_MAX 30 //Software limited maximum phase current (with 0,8333mohm shunt)
#define TEMP_60C 576 //
#define TEMP_70C 593 //
#define OTLO TEMP_60C //Heatsink over temperature lockout
//60deg-C = 576, 70deg_C = 593
  // TODO: Read proper speed setting with serial println
// 2660 rpm == noin 88 speedCount
#define DEFAULT_SPEED 75 // 2017.05.27 75 sounds ok

//Scaled parameters
int THR_LOW = (float)THR_LOW_V*1024/5; //Throttle input 0% level (Scale: 1024=5V)
int THR_HIGH = (float)THR_HIGH_V*1024/5; //Throttle input 100% level (Scale: 1024=5V)
#define OVLO_mV 60000ull // OVLO in mV, 60 000 mV
//#define UVLO_mV 44500ull // UVLO in mV

#define MAX_THROTTLE   900
#define MAX_THROTTLE_P 400
#define MAX_THROTTLE_I 500
#define MAX_THROTTLE_D 400

// Variables
int  Field_out; // Rotor PWM signal
char x = 0; // Loop time measurement toggle signal
int  Stator_out = 0; // Stator PWM signal
volatile unsigned long HW_OC = 0; //HW_OC Signal is set by interrupt if hardware has sensed an over-current event

int speedSet   = 255;

void ISR_hall_triggered();
void ISR_over_current_sensed();

int getTR(int);
int getCurrent_A();
unsigned long long getVoltage_mV();
long long getCurrent_I();
int getTemperature_C();
unsigned long long getCompensatedUVLO_mV(long long currentA);
  
int throttleCtrl();
int getHallPeriod();

volatile unsigned char timerCnt   = 0;
volatile unsigned char preIncrCnt = 0;
volatile unsigned int  hallCnt    = 0;

int PIDClip  = 0;
int PIDClipP = 0;
int PIDClipI = 0;
int PIDClipD = 0;


#define RPMS 16
volatile unsigned char rpmPtr;
volatile int  rpmSensor[RPMS];

/************************************************************
 * What would be scientific way of defining UVLO limits for *
 * any current?                                             *
 * Current is probably lot slower to change, but that's     *
 * should be a minor issue (we could drop the filtering in  *
 * SW.                                                      *
 ************************************************************/
unsigned long long getCompensatedUVLO_mV(long long currentA) {
  if (currentA < 2) {
    return 46000ull; // Idle voltage must be higher
  } else if (currentA < 5) {
    return 45000ull; // 0,2..1C == Idling
  } else if (currentA < 9) {
    return 44500ull; // 1..2 C == Light cutting
  } else if (currentA < 21) {
    return 43500ull; // 2..5 C == Heavy cutting
  } else {
    return 43200ull; // 5.. C  == Impossibly high current
  }
}

unsigned int VBattArray[16];
int IBattArray[16];

void updateVBatt() {
  static int ptr = 0;
  VBattArray[ptr] = analogRead(AI_VBATT);
  IBattArray[ptr++] = getCurrent_A();
  ptr %= 16;
}

int fieldPwm[8];

void setup() {
  int i;
  // put your setup code here, to run once:
  pinMode(RPM, INPUT);
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, LOW);
  pinMode(DEBUG_OUT, OUTPUT);
  digitalWrite(DEBUG_OUT, LOW);
  pinMode(DIRECTION, OUTPUT);
  digitalWrite(DIRECTION, HIGH);
  pinMode(PWM_STATOR, OUTPUT);
  pinMode(PWM_FIELD, OUTPUT);
  pinMode(AI_TR1, INPUT);
  pinMode(AI_TR2, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // Provide active low and active high for LED to select from (by connecting the other end of LED differently)
  pinMode(EXT_LED_EN, OUTPUT);
  digitalWrite(EXT_LED_EN, HIGH);
  pinMode(EXT_LED_EN_N, OUTPUT);
  digitalWrite(EXT_LED_EN_N, HIGH);

  delay(1);
    
  attachInterrupt(digitalPinToInterrupt(RPM), ISR_hall_triggered, CHANGE); //Enables interrupt handling for speed measurement when Hall A is falling
  
  attachInterrupt(digitalPinToInterrupt(IC_CLR), ISR_over_current_sensed, FALLING); //Enables interrupt handling if overcurrent is sensed by hardware

  memset((void*)rpmSensor, 0, RPMS*sizeof(int));
  rpmPtr = 0;

  // Quick average
  for (i=0; i<16; i++) {
    updateVBatt();  
  }
  
  Timer1.initialize(55); // initialize timer1, and set a 55 u_second period
  //  Timer1.attachInterrupt(updateVBatt);
  
  //Initialize Timer2 for speed measurement

  // TCCR2A |= (1 << WGM20); // CTC Mode, we want to count upwards into infinity (with roll over 255 => 0)
  // normal mode (=0) should also work, but it was unstable
  TCCR2A |= (1 << WGM20) | (1 << WGM21); // Fast PWM Mode, we want to count only upwards into infinity (with roll over 255 => 0)
  TIMSK2 |= (1 << TOIE2); //Overflow interrupt
  OCR2A   = 250; // Compare register value
  TIMSK2 |= (1 << OCIE2A); //Set interrupt on compare match
  TCCR2B  = 7; // set prescaler to 1024 and starts PWM, 64 us per step
  
  Serial.begin(38400);
  //Serial.begin(250000);
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
  speedSet = 255;

  // Quick average update
  for (i=0; i<16; i++) {
    updateVBatt();  
  }
  
  unsigned long long BatteryVoltage_mV = getVoltage_mV();

  Serial.print("Battery avg ");
  Serial.print((unsigned int)BatteryVoltage_mV, DEC);
  Serial.println(" mV");

  fieldPwm[0] = ((long long)(FIELD_DEFAULT_V)*FIELD_PWM_CORRECTION) / BatteryVoltage_mV;
  fieldPwm[1] = ((long long)(FIELD_DEFAULT_V)*FIELD_PWM_CORRECTION) / BatteryVoltage_mV;
  fieldPwm[2] = ((long long)(FIELD_DEFAULT_V)*FIELD_PWM_CORRECTION) / BatteryVoltage_mV;
  fieldPwm[3] = ((long long)(FIELD_DEFAULT_V)*FIELD_PWM_CORRECTION) / BatteryVoltage_mV;
  fieldPwm[4] = ((long long)(FIELD_DEFAULT_V)*FIELD_PWM_CORRECTION) / BatteryVoltage_mV;
  fieldPwm[5] = ((long long)(FIELD_DEFAULT_V)*FIELD_PWM_CORRECTION) / BatteryVoltage_mV;
  fieldPwm[6] = ((long long)(FIELD_DEFAULT_V)*FIELD_PWM_CORRECTION) / BatteryVoltage_mV;
  fieldPwm[7] = ((long long)(FIELD_DEFAULT_V)*FIELD_PWM_CORRECTION) / BatteryVoltage_mV;

  for (i=0; i<1; i++) {
    Serial.print("fieldPwm[");
    Serial.print(i,DEC);
    Serial.print("] = ");
    Serial.println(fieldPwm[i],DEC);

  }

  Field_out = momentToField(0);
  Timer1.pwm(PWM_FIELD, Field_out);

  interrupts(); // enable interrupts

  // Small delay after enabling the field, before starting up should allow some
  // field to grow up. This should help in keeping the startup phase currents
  // in more reasonable level (less HW_OC events).
  delay(100);    

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
  long long avgCurrent = (long long)analogRead(AI_AVG_I)-512;
  avgCurrent = (avgCurrent * (long long)90) / 512 ; // magic according to Tero's comment
  return (int)avgCurrent;
}

unsigned long long getCurrentVoltage_mV(){
  unsigned long long volt;
  volt  = analogRead(AI_VBATT);
  volt *= 64;  // AD reading *66 is correct mv setting. We approximate that with *64.
  return volt;
}

unsigned long long getVoltage_mV(){
  unsigned long long volt = 0;
  int i;
  for (i=0; i<15; i++) {
    volt += VBattArray[i];
  }
  // This is calibrated at 2017.07.28
  volt -= VBattArray[i]/2;
  volt += VBattArray[i]/4;
  //volt -= VBattArray[i]/16;
  // 52,8 V at multimeter => produces reading 52600 mV

  volt *= 4; // AD reading *66 is correct mv setting. We approximate that with *16*4.
  // TODO debug Remove
  // volt = 25000;
  
  return volt;
}

// Similar function for getting Current
long long getCurrent_I(){
  long long amp = 0;
  int i;
  for (i=0; i<16; i++) {
    amp  += IBattArray[i];
  }
  amp /= 16;
  return amp;
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

int momentToField(int momentReq) {

  if(momentReq < -2) {
    emergencyStop(8);
  }
  if(momentReq > 5) {
    emergencyStop(10);
  }
  return fieldPwm[momentReq+2];

}

int emergencyStop(int blinkCount) {
  int i = 0;
  unsigned int enteringVoltage = getVoltage_mV();
  unsigned int enteringCurrent = getCurrent_I();
  digitalWrite(ENABLE, LOW);
  Timer1.pwm(PWM_STATOR, 0);
  Timer1.pwm(PWM_FIELD, 0);
  // Blink loop forever!
  Serial.println("Emergency stop"); 
  Serial.print("Battery ");
  Serial.print(enteringVoltage, DEC);
  Serial.print(" mV Current ");
  Serial.print(enteringCurrent, DEC);
  Serial.print(" mV unfilterd ");
  Serial.print((unsigned int)getCurrentVoltage_mV(), DEC);
  Serial.println(" mV ");
  Serial.print("AVG current ");
  Serial.print( getCurrent_A(), DEC);
  Serial.println(" A");
  Serial.print("Temperature ");
  Serial.print(getTemperature_C(), DEC);
  Serial.println(" ru (Random Unit)");
  
  Serial.print("Emergency stop "); 
  Serial.print(blinkCount, DEC); 
  switch(blinkCount) {
  case 2:
    Serial.print(" UVLO limit "); 
    Serial.print((unsigned int)getCompensatedUVLO_mV(enteringCurrent), DEC);
    Serial.println(" mv"); 
    break;
  case 4:
    Serial.println(" OVLO."); 
    break;
  case 6:
    Serial.println(" OTLO."); 
    break;
  case 8:
    Serial.println(" too low momentReq"); 
    break;
  case 10:
    Serial.println(" too high momentReq"); 
    break;
  default:
    Serial.println(" unknown."); 
    break;
  }

  while(1) {
    // Quick average
    for (i=0; i<16; i++) {
      updateVBatt();  
    }
    Serial.println("Emergency stop"); 
    Serial.print("Battery ");
    Serial.print(enteringVoltage, DEC);
    Serial.print(" mV Current ");
    Serial.print(enteringCurrent, DEC);
    Serial.print(" mV unfilterd ");
    Serial.print((unsigned int)getCurrentVoltage_mV(), DEC);
    Serial.println(" mV ");
    Serial.print("AVG current ");
    Serial.print( getCurrent_A(), DEC);
    Serial.println(" A");
    Serial.print("Temperature ");
    Serial.print(getTemperature_C(), DEC);
    Serial.println(" ru (Random Unit)");
    printStatus(7, 0, 0, 0, 0, (unsigned int)getVoltage_mV());
    for (i=0;i<blinkCount;i++) {
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
    }   
    for (i=0; i<20; i++) {
      delay(50);
    }
  } 
} 

void printStatus(int timerCnt, int speedCount, int currentThrottle, int stator, int field, unsigned int voltage) {

  switch (timerCnt%256) {
  case 0: 
    //    Serial.println("");
    break;
  case 7: 
  case 7+128: 
    Serial.print("Batt ");
    Serial.print(voltage, DEC);
    Serial.print(" mV");
    Serial.print(" Enable ");
    Serial.print(digitalRead(ENABLE), DEC);
    Serial.print(" Current ");
    Serial.print(getCurrent_A(), DEC);
    Serial.print(" A ");
    Serial.print("OC Count ");
    Serial.print(HW_OC, DEC);
    Serial.print(" Temp ");
    Serial.print(getTemperature_C(), DEC);
    Serial.print(" ru (Random Unit)");
    Serial.println("");
    break;
    /*
    case 11: 
      Serial.print("TR2 ");
      Serial.print(getTR(2), HEX);
      Serial.println("");
      break;
    case 13+128:
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
  case 21:
  case 149:
    //case 21+64:
    //case 149+64:
    Serial.print("Thrtl ");
    Serial.print(currentThrottle, DEC);
    Serial.print(" spd ");
    Serial.print(speedCount, DEC);
    Serial.print(" stat ");
    Serial.print(stator, DEC);
    Serial.print(" fld ");
    Serial.print(field, DEC);
    Serial.print(" Sum ");
    Serial.print(PIDClip, DEC);
    Serial.print(" P ");
    Serial.print(PIDClipP, DEC);
    Serial.print(" I ");
    Serial.print(PIDClipI, DEC);
    Serial.print(" D ");
    Serial.print(PIDClipD, DEC);
    Serial.println("");
    break;      
  case 23:
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
  // TODO debug remove, when we have throttle control in use
  handleOn = 1;
  digitalWrite(ENABLE, HIGH);
    
  // Try to run throttleCtrl() once per Timer2 overflow. If we are slower, then we are.
  if (lTimerCnt != timerCnt) {
    attachInterrupt(digitalPinToInterrupt(IC_CLR), ISR_over_current_sensed, FALLING); //Enables interrupt handling if overcurrent is sensed by hardware
    
    if (handleOn == 1) {
      // Do we really need this slowliness?
      if (speedSet > DEFAULT_SPEED)
	speedSet -= 8;
      else
	speedSet = DEFAULT_SPEED;
      // IIR for throttle change - this makes things smoother
      Throttle -= Throttle/16;
      Throttle += throttleCtrl(speedSet, speedCount) / 16;
    } else {
      // This was: Throttle = 0, but that might increase the breaking current too fast. So we slow down slowly, upto 100 timer ticks
      Throttle -= Throttle/8;
      Throttle -= 1;
      if (Throttle < 0)
	Throttle = 0;
      // Clear also the speed PID
      speedSet = 255;
      throttleCtrl(255, speedCount);
    }    
  } 
  
  Stator_out = Throttle;

  if (Stator_out > 950)
    Stator_out = 950;
    
  unsigned long long BatteryVoltage_mV = getVoltage_mV();
  unsigned long long UVLO_mV = getCompensatedUVLO_mV(getCurrent_A());

  if (BatteryVoltage_mV > OVLO_mV) {
    emergencyStop(4);
  }
  if (BatteryVoltage_mV < UVLO_mV) {
    emergencyStop(2);
  }

  int SinkTemperature = analogRead(AI_TEMP);

  if (SinkTemperature > OTLO) {
    emergencyStop(6);
  }

  if (handleOn == 0) {
    // Break harder if user has released the handle
    if (speedCount < 255) {
      breakCnt = timerCnt;
    } else if (((timerCnt+1)&0x7f) == (breakCnt&0x7f)) {
      // Just save some power
      if (lTimerCnt != timerCnt) {
	Serial.println("Stop breaking and disable all");
      }
      digitalWrite(ENABLE, LOW);
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
    // In case of breaking the increasing field increases current, so reverse for breaking
    momentReq++;
  }

  // Breaking must be done with small field
  if (handleOn == 0) {
      momentReq = -2;
  }
  
  Field_out = momentToField(momentReq);
  
  if (lTimerCnt != timerCnt) {
    lTimerCnt = timerCnt;    
    printStatus(timerCnt, speedCount, Throttle,
		Stator_out, Field_out, BatteryVoltage_mV);//Field_out);
  }
    
  Timer1.pwm(PWM_STATOR, Stator_out);
  Timer1.pwm(PWM_FIELD, Field_out);

  updateVBatt();  

  //TODO: Use the following (it's faster according to this https://playground.arduino.cc/Code/Timer1)
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
  static int throttleI        = 0;
  int throttleP               = 0;
  int throttleD               = 0;
  int throttle                = 0;
  int SpeedError              = 0;
  static int SpeedErrorOld    = 0;
  static unsigned long OC_cnt = 0;
   
  SpeedError = speedCount - targetSpeed;
  
  // TODO: Consider throttleI limiting during startup

  if ((SpeedError/8 == 0) && (SpeedError < 0)) {
    throttleI  = throttleI - 1;      
  } else if ((SpeedError/8 == 0) && (SpeedError > 0)) {
    throttleI  = throttleI + 1;      
  } else {
    throttleI  = throttleI + SpeedError/8;
  }
  throttleP  = SpeedError*32;
  throttleD  = SpeedErrorOld - SpeedError;
  throttleD *= 16;
  
  SpeedErrorOld = SpeedError;

  if (throttleI > MAX_THROTTLE_I) {
    throttleI = MAX_THROTTLE_I;
    PIDClipI++;
  }
  if (throttleI < 0) {
    throttleI = 0;
    PIDClipI++;
  }
  if (speedCount > 250) {
    // Do not integrate while we are stationary
    throttleI = 0;
  }
  
  if (throttleP > MAX_THROTTLE_P) {
    throttleP = MAX_THROTTLE_P;
    PIDClipP++;
  }
  if (throttleP < 0) {
    throttleP = 0;
  }

  if (throttleD > MAX_THROTTLE_D) {
    throttleD = MAX_THROTTLE_D;
    PIDClipD++;
  }
  if (throttleD < -MAX_THROTTLE_D) {
    throttleD = -MAX_THROTTLE_D;
    PIDClipD++;
  }

  // Feed-forward component
  if ((targetSpeed == 0) || (targetSpeed == 255)) {
    throttle = 0;
  } else {
    throttle = 550 - speedCount*2; // 400 at target speed
  }
  // Feed-back component
  throttle += throttleP;
  throttle += throttleI;
  throttle += throttleD;
  
  if (throttle > MAX_THROTTLE) {
    throttle = MAX_THROTTLE;
    PIDClip++;
  }

  // In case of HW detected over current we make dramatic adjustment
  if (OC_cnt != HW_OC) {
    throttle  -= throttle  / 8;
    throttleI -= throttleI / 8;
    OC_cnt = HW_OC;
  }
  
  if (throttle < 0) {
    throttle = 0;
    PIDClip++;
  }

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
