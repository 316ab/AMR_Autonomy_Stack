// ============================================================
//  AMR ROBOT — Full Hardware Code v33.0  (MD10C actuator)
//
//  CHANGE vs v31/v32 (ACTUATOR ONLY - nothing else touched):
//    * The ULTRASONICS NO LONGER DRIVE THE ACTUATOR AT ALL.
//      The actuator moves ONLY on explicit serial commands:
//        e = EXTEND fully and HOLD  (ignores ultrasonics)
//        u = RETRACT fully and HOLD (ignores ultrasonics)
//        s = STOP
//      There is no auto extend/retract from table detection anymore.
//    * Ultrasonics are still read + published over serial exactly as
//      before (safety_monitor / IMU / battery output UNCHANGED).
//
//  Everything else (sonar reading, serial format + rate limiter, IMU,
//  LEDs, battery, LCD) is identical to the working v31/v32.
//
// -- PIN MAP --------------------------------------------------
//  HC-SR04 Front Trig=32 Echo=33 | Back Trig=14 Echo=15
//  Left Trig=18 Echo=19 | Right Trig=28 Echo=29
//  Up-Front Trig=35 Echo=36 | Up-Back Trig=16 Echo=17
//  LED FL=22 BL=23 FR=31 BR=30
//  Voltage A1  Current ACS712 30A A0
//  Actuator MD10C PWM=26 DIR=27
//  GY-87 + LCD SDA=20 SCL=21 LCD=0x27
// ============================================================
#include <FastLED.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU6050_light.h>
#include <HMC5883L.h>
#include <Adafruit_BMP085.h>
// -- OBJECTS --------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 20, 4);
MPU6050           mpu(Wire);
HMC5883L          compass;
Adafruit_BMP085   bmp;
bool imuOK=false, compassOK=false, bmpOK=false;
float imu_roll=0, imu_pitch=0;
float ax=0, ay=0, az=0;
float gx=0, gy=0, gz=0;
// -- LED ------------------------------------------------------
#define NUM_LEDS   8
#define BRIGHTNESS 80
#define PIN_LED_FL 22
#define PIN_LED_BL 23
#define PIN_LED_FR 31
#define PIN_LED_BR 30
CRGB leds_FL[NUM_LEDS], leds_BL[NUM_LEDS];
CRGB leds_FR[NUM_LEDS], leds_BR[NUM_LEDS];
// -- SONAR ----------------------------------------------------
#define MAX_DIST_CM  300
#define MAX_ECHO_US  17500
// >>> GLITCH REJECTION: ignore any raw reading above this many cm.
#define REJECT_ABOVE 65
// (Table hysteresis constants kept for reference but NO LONGER drive
//  the actuator - see tickActState. tableDetected is still computed so
//  the LCD can show TBL/--- but it does not move the actuator.)
#define TABLE_ENTER  20
#define TABLE_EXIT   30
#define TABLE_DEBOUNCE 4
struct Sonar {
  uint8_t trig; uint8_t echo;
  int dist;          // last FILTERED (good) distance
  int m0,m1,m2;      // median window of last 3 accepted raw reads
  uint8_t mi;        // median window index
};
Sonar sonars[6] = {
  {32,33,MAX_DIST_CM,MAX_DIST_CM,MAX_DIST_CM,MAX_DIST_CM,0},
  {14,15,MAX_DIST_CM,MAX_DIST_CM,MAX_DIST_CM,MAX_DIST_CM,0},
  {18,19,MAX_DIST_CM,MAX_DIST_CM,MAX_DIST_CM,MAX_DIST_CM,0},
  {28,29,MAX_DIST_CM,MAX_DIST_CM,MAX_DIST_CM,MAX_DIST_CM,0},
  {35,36,MAX_DIST_CM,MAX_DIST_CM,MAX_DIST_CM,MAX_DIST_CM,0},
  {16,17,MAX_DIST_CM,MAX_DIST_CM,MAX_DIST_CM,MAX_DIST_CM,0},
};
uint8_t       sonarIdx=0, sonarState=0;
unsigned long sonarStateTs=0, lastSonarFire=0;
#define SONAR_INTERVAL 35
bool tableDetected=false;
uint8_t tablePresentCount=0, tableGoneCount=0;
// -- ANALOG ---------------------------------------------------
#define PIN_VOLTAGE      A1
#define PIN_CURRENT      A0
#define VOLTAGE_RATIO    5.0
#define VOLTAGE_CAL      0.992
#define ACS712_SENS      0.066
#define CURRENT_DEADBAND 1.0
int   acs712_zero=512;
float filtVoltage=0.0, filtCurrent=0.0;
#define ADC_SAMPLES 16
int     voltBuf[ADC_SAMPLES], currBuf[ADC_SAMPLES];
uint8_t adcIdx=0;
unsigned long lastADC=0;
// -- ACTUATOR  Cytron MD10C -----------------------------------
#define ACT_PWM   26
#define ACT_DIR   27
#define DIR_EXTEND  LOW
#define DIR_RETRACT HIGH
#define ACT_SPEED     255
#define ACT_STROKE_MS 10000
#define ACT_RAMP_MS   10
#define ACT_MIN_MOVE_MS 600     // commit to a direction this long before reversing
bool          actMoving=false, actMovingDir=true;
bool          fullyExtended=false, fullyRetracted=true;
// COMMAND-ONLY actuator control. These two flags are the ONLY things that
// decide direction now. Ultrasonics do NOT affect the actuator.
bool          cmdExtendHold  = false;   // 'e' -> extend fully and hold
bool          cmdRetractHold = true;    // 'u' -> retract fully and hold (start down)
int           actPWM=0;
unsigned long actMoveStart=0, lastRamp=0;
// -- TIMING ---------------------------------------------------
unsigned long lastIMU=0,lastSerial=0,lastLED=0,lastLCD=0,lastFlash=0;
bool flashState=false;
#define IMU_INTV    50
#define SERIAL_INTV 100
#define LED_INTV     60
#define LCD_INTV    500
#define FLASH_INTV  200
// ============================================================
//  ACTUATOR HELPERS
// ============================================================
void startMove(bool dirUp, unsigned long now){
  actMoving=true; actMovingDir=dirUp; actMoveStart=now; actPWM=0; lastRamp=now;
  digitalWrite(ACT_DIR, dirUp?DIR_EXTEND:DIR_RETRACT);   // direction set ONCE
  Serial.println(dirUp?F(">> EXTENDING"):F(">> RETRACTING"));
}
void driveStop(){ analogWrite(ACT_PWM,0); }
// -- small helpers -------------------------------------------
int median3(int a,int b,int c){
  if(a>b){int t=a;a=b;b=t;}
  if(b>c){int t=b;b=c;c=t;}
  if(a>b){int t=a;a=b;b=t;}
  return b;
}
void printDist(int d){ if(d<10)Serial.print(F("  ")); else if(d<100)Serial.print(F(" ")); Serial.print(d); }
CRGB colorForDist(int d,bool fl){ if(d<=20)return fl?CRGB::Red:CRGB::Black; if(d<=60)return CRGB::Yellow; return CRGB::Green; }
void setStrip(CRGB*a,CRGB*b,int d,bool fl){ CRGB c=colorForDist(d,fl); for(int i=NUM_LEDS/2;i<NUM_LEDS;i++){a[i]=c;b[i]=c;} }
void setSide (CRGB*a,CRGB*b,int d,bool fl){ CRGB c=colorForDist(d,fl); for(int i=0;i<NUM_LEDS/2;i++){a[i]=c;b[i]=c;} }
void startupAnim(){
  CRGB cols[]={CRGB::Blue,CRGB::Green,CRGB::Yellow,CRGB::Red,CRGB::Black};
  for(int c=0;c<5;c++){ fill_solid(leds_FL,NUM_LEDS,cols[c]); fill_solid(leds_BL,NUM_LEDS,cols[c]);
    fill_solid(leds_FR,NUM_LEDS,cols[c]); fill_solid(leds_BR,NUM_LEDS,cols[c]); FastLED.show(); delay(300);}
}
void lcdRecover(){ Wire.end(); delay(10); Wire.begin(); Wire.setClock(100000); Wire.setWireTimeout(3000,true); lcd.init(); lcd.backlight(); }
void updateLCD(){
  if(Wire.getWireTimeoutFlag()){ Wire.clearWireTimeoutFlag(); lcdRecover(); return; }
  lcd.setCursor(0,0);
  if      (actMoving && actMovingDir)  lcd.print(F("Act: EXTENDING  >>> "));
  else if (actMoving && !actMovingDir) lcd.print(F("Act: RETRACTING <<< "));
  else if (fullyExtended)              lcd.print(F("Act: HOLDING (UP)   "));
  else                                 lcd.print(F("Act: IDLE (DOWN)    "));
  lcd.setCursor(0,1);
  lcd.print(F("UF:"));
  if(sonars[4].dist<100)lcd.print(' '); if(sonars[4].dist<10)lcd.print(' ');
  lcd.print(sonars[4].dist); lcd.print(F("cm UB:"));
  if(sonars[5].dist<100)lcd.print(' '); if(sonars[5].dist<10)lcd.print(' ');
  lcd.print(sonars[5].dist);
  lcd.print(tableDetected?F("cm TBL"):F("cm ---"));
  lcd.setCursor(0,2);
  lcd.print(F("V:")); lcd.print(filtVoltage,2);
  lcd.print(F("V  I:")); lcd.print(fabs(filtCurrent),2); lcd.print(F("A       "));
  lcd.setCursor(0,3);
  if      (filtVoltage>5.0&&filtVoltage<20.0) lcd.print(F("!! BATTERY CRITICAL!"));
  else if (filtVoltage>5.0&&filtVoltage<21.5) lcd.print(F("!  Battery Low      "));
  else                                        lcd.print(F("   Battery OK       "));
}
// ============================================================
//  SETUP
// ============================================================
void setup(){
  pinMode(ACT_PWM,OUTPUT); pinMode(ACT_DIR,OUTPUT);
  analogWrite(ACT_PWM,0); digitalWrite(ACT_DIR,DIR_RETRACT);
  for(int i=0;i<6;i++){ pinMode(sonars[i].trig,OUTPUT); pinMode(sonars[i].echo,INPUT); digitalWrite(sonars[i].trig,LOW); }
  Serial.begin(115200);
  delay(500);
  long s=0; for(int i=0;i<200;i++){ s+=analogRead(PIN_CURRENT); delayMicroseconds(400);} acs712_zero=s/200;
  for(int i=0;i<ADC_SAMPLES;i++){ voltBuf[i]=analogRead(PIN_VOLTAGE); currBuf[i]=acs712_zero; }
  Serial.print(F("ACS712 zero=")); Serial.println(acs712_zero);
  Wire.begin(); Wire.setClock(100000); Wire.setWireTimeout(3000,true);
  lcd.init(); lcd.backlight(); lcd.clear();
  lcd.setCursor(0,0); lcd.print(F("   AMR Robot v33    "));
  lcd.setCursor(0,1); lcd.print(F("  Initializing...   "));
  Serial.println(F("LCD OK"));
  lcd.setCursor(0,2); lcd.print(F("MPU-6050...         "));
  Serial.print(F("MPU-6050... "));
  if(mpu.begin()==0){
    Wire.beginTransmission(0x68); Wire.write(0x37); Wire.write(0x02); Wire.endTransmission();
    lcd.setCursor(0,3); lcd.print(F("Keep still 3s...    "));
    mpu.calcOffsets(true,true); imuOK=true;
    lcd.setCursor(0,3); lcd.print(F("MPU-6050 OK         ")); Serial.println(F("OK"));
  } else { lcd.setCursor(0,3); lcd.print(F("MPU-6050 FAIL       ")); Serial.println(F("FAIL")); }
  Wire.beginTransmission(0x1E);
  if(Wire.endTransmission()==0){
    compass.setRange(HMC5883L_RANGE_1_3GA);
    compass.setMeasurementMode(HMC5883L_CONTINOUS);
    compassOK=true; Serial.println(F("HMC5883L OK"));
  } else Serial.println(F("HMC5883L FAIL"));
  bmpOK=bmp.begin(); Serial.println(bmpOK?F("BMP180 OK"):F("BMP180 FAIL"));
  FastLED.addLeds<WS2812B,PIN_LED_FL,GRB>(leds_FL,NUM_LEDS);
  FastLED.addLeds<WS2812B,PIN_LED_BL,GRB>(leds_BL,NUM_LEDS);
  FastLED.addLeds<WS2812B,PIN_LED_FR,GRB>(leds_FR,NUM_LEDS);
  FastLED.addLeds<WS2812B,PIN_LED_BR,GRB>(leds_BR,NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS); FastLED.setMaxPowerInVoltsAndMilliamps(5,1600);
  startupAnim();
  driveStop();
  lcd.clear();
  Serial.println(F(""));
  Serial.println(F("=============================================="));
  Serial.println(F("       AMR ROBOT v33 - MD10C Actuator"));
  Serial.println(F("  Actuator = COMMAND ONLY (US does NOT move it)"));
  Serial.println(F("  Commands: e=Extend&Hold  u=Retract&Hold  s=Stop"));
  Serial.println(F("=============================================="));
  Serial.flush();
}
// ============================================================
//  LOOP
// ============================================================
static bool loopStarted=false;
void loop(){
  unsigned long now=millis();
  if(!loopStarted){ loopStarted=true; Serial.println(F("Loop started")); }
  tickIMU(now); tickSonar(now); tickADC(now); tickFlash(now);
  tickLED(now); tickActRamp(now); tickActState(now);
  tickLCD(now); tickSerial(now); tickCmd();
}
// ============================================================
//  TICK FUNCTIONS
// ============================================================
void tickIMU(unsigned long now){
  if(!imuOK) return;
  if(now-lastIMU<IMU_INTV) return;
  lastIMU=now;
  mpu.update();
  imu_roll  = mpu.getAngleX();
  imu_pitch = mpu.getAngleY();
  ax = mpu.getAccX();
  ay = mpu.getAccY();
  az = mpu.getAccZ();
  gx = mpu.getGyroX();
  gy = mpu.getGyroY();
  gz = mpu.getGyroZ();
}
// Accept a raw reading: feed it through a median-of-3 filter.
void acceptReading(Sonar &s, int rawCm){
  int v;
  if (rawCm <= 0 || rawCm > MAX_DIST_CM) v = REJECT_ABOVE;   // bad/echoless -> FAR
  else if (rawCm > REJECT_ABOVE)         v = REJECT_ABOVE;   // cap far readings
  else                                   v = rawCm;          // valid close reading
  s.m0=s.m1; s.m1=s.m2; s.m2=v;            // shift median window
  s.dist = median3(s.m0,s.m1,s.m2);        // median filters out single spikes
}
// Still compute tableDetected for the LCD display ONLY. It no longer
// drives the actuator (see tickActState).
void updateTableState(){
  bool looksPresent = (sonars[4].dist <= TABLE_ENTER) &&
                      (sonars[5].dist <= TABLE_ENTER);
  bool looksGone     = (sonars[4].dist >  TABLE_EXIT) ||
                      (sonars[5].dist >  TABLE_EXIT);
  if(looksPresent){ tablePresentCount++; tableGoneCount=0; }
  else if(looksGone){ tableGoneCount++; tablePresentCount=0; }
  if(tablePresentCount>=TABLE_DEBOUNCE) tableDetected=true;
  if(tableGoneCount   >=TABLE_DEBOUNCE) tableDetected=false;
}
void tickSonar(unsigned long now){
  Sonar &s=sonars[sonarIdx];
  switch(sonarState){
    case 0:
      if(now-lastSonarFire<SONAR_INTERVAL) return;
      lastSonarFire=now;
      digitalWrite(s.trig,LOW); delayMicroseconds(2);
      digitalWrite(s.trig,HIGH); delayMicroseconds(10);
      digitalWrite(s.trig,LOW);
      sonarState=1; sonarStateTs=micros();
      break;
    case 1:
      if(digitalRead(s.echo)==HIGH){ sonarState=2; sonarStateTs=micros(); }
      else if(micros()-sonarStateTs>5000){
        acceptReading(s, MAX_DIST_CM);
        sonarIdx=(sonarIdx+1)%6; sonarState=0;
        updateTableState();
      }
      break;
    case 2:
      if(digitalRead(s.echo)==LOW){
        int cm=(micros()-sonarStateTs)/58;
        acceptReading(s, cm);
        sonarIdx=(sonarIdx+1)%6; sonarState=0;
        updateTableState();
      } else if(micros()-sonarStateTs>MAX_ECHO_US){
        acceptReading(s, MAX_DIST_CM);
        sonarIdx=(sonarIdx+1)%6; sonarState=0;
        updateTableState();
      }
      break;
  }
}
void tickADC(unsigned long now){
  if(now-lastADC<8) return; lastADC=now;
  voltBuf[adcIdx]=analogRead(PIN_VOLTAGE); currBuf[adcIdx]=analogRead(PIN_CURRENT);
  adcIdx=(adcIdx+1)%ADC_SAMPLES;
  long vS=0,cS=0; for(int i=0;i<ADC_SAMPLES;i++){vS+=voltBuf[i];cS+=currBuf[i];}
  float rawV=(vS/(float)ADC_SAMPLES)*(5.0/1023.0)*VOLTAGE_RATIO*VOLTAGE_CAL;
  float rawC=((cS/(float)ADC_SAMPLES-acs712_zero)*(5.0/1023.0)/ACS712_SENS);
  if(fabs(rawC)<CURRENT_DEADBAND) rawC=0.0;
  filtVoltage=0.9*filtVoltage+0.1*rawV;
  filtCurrent=0.6*filtCurrent+0.4*rawC;
}
void tickFlash(unsigned long now){ if(now-lastFlash<FLASH_INTV) return; lastFlash=now; flashState=!flashState; }
void tickLED(unsigned long now){
  if(now-lastLED<LED_INTV) return; lastLED=now;
  setStrip(leds_FL,leds_FR,sonars[0].dist,flashState);
  setStrip(leds_BL,leds_BR,sonars[1].dist,flashState);
  setSide (leds_FL,leds_BL,sonars[2].dist,flashState);
  setSide (leds_FR,leds_BR,sonars[3].dist,flashState);
  FastLED.show();
}
// -- ACTUATOR CONTROL  (COMMAND ONLY - ultrasonics do NOT move it) -----
//  Direction is decided SOLELY by the last command:
//    cmdExtendHold  (set by 'e') -> drive up,   then hold extended
//    cmdRetractHold (set by 'u') -> drive down,  then hold retracted
//  No table detection, no hysteresis, no auto reversal.
void tickActState(unsigned long now){
  bool targetUp = cmdExtendHold;   // true = extend, false = retract

  if (targetUp && !fullyExtended) {
    if (!actMoving || !actMovingDir) {
      if (actMoving && !actMovingDir && (now-actMoveStart < ACT_MIN_MOVE_MS)) {
        // finishing minimum retract commit before reversing
      } else { fullyRetracted=false; startMove(true, now); }
    }
    if (actMoving && actMovingDir && (now-actMoveStart >= ACT_STROKE_MS)) {
      actMoving=false; fullyExtended=true; driveStop();
      Serial.println(F(">> FULLY EXTENDED - HOLDING"));
    }
  }
  else if (!targetUp && !fullyRetracted) {
    if (!actMoving || actMovingDir) {
      if (actMoving && actMovingDir && (now-actMoveStart < ACT_MIN_MOVE_MS)) {
        // finishing minimum extend commit before reversing
      } else { fullyExtended=false; startMove(false, now); }
    }
    if (actMoving && !actMovingDir && (now-actMoveStart >= ACT_STROKE_MS)) {
      actMoving=false;
      fullyRetracted=true;
      driveStop();
      Serial.println(F(">> FULLY RETRACTED - HOLDING"));
    }
  }
  else {
    if(actMoving){
      // keep moving until stroke time finishes
    }
  }
}
// Smooth ramp - only PWM; direction was set once in startMove().
void tickActRamp(unsigned long now){
  if(!actMoving) return;
  if(now-lastRamp<ACT_RAMP_MS) return;
  lastRamp=now;
  if(actPWM<ACT_SPEED){ actPWM+=4; if(actPWM>ACT_SPEED)actPWM=ACT_SPEED; }
  analogWrite(ACT_PWM, actPWM);
}
void tickLCD(unsigned long now){ if(now-lastLCD<LCD_INTV) return; lastLCD=now; updateLCD(); }
void tickSerial(unsigned long now)
{
  if(now - lastSerial < SERIAL_INTV) return;   // rate limiter (KEEP - protects sonar timing)
  lastSerial = now;
  // Ultrasonic
  Serial.print("U,");
  Serial.print(sonars[0].dist);   // Front
  Serial.print(",");
  Serial.print(sonars[1].dist);   // Back
  Serial.print(",");
  Serial.print(sonars[2].dist);   // Left
  Serial.print(",");
  Serial.print(sonars[3].dist);   // Right
  Serial.print(",");
  Serial.print(sonars[4].dist);   // Upper Front
  Serial.print(",");
  Serial.println(sonars[5].dist); // Upper Back
  // Battery
  Serial.print("S,");
  Serial.print(filtVoltage, 2);
  Serial.print(",");
  Serial.println(fabs(filtCurrent), 2);
  // IMU
  Serial.print("R,");
  Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);
  Serial.print(",");
  Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.print(gz);
  Serial.print(",");
  Serial.print(imu_roll);
  Serial.print(",");
  Serial.println(imu_pitch);
}
void tickCmd() {
  if(!Serial.available()) return;
  char c=Serial.read(); unsigned long now=millis();
  // e = EXTEND fully and HOLD (ignores ultrasonics completely)
  if(c=='e'){
    cmdExtendHold  = true;
    cmdRetractHold = false;
    fullyExtended  = false;
    fullyRetracted = false;
    startMove(true,now);
    Serial.println(F(">> EXTEND & HOLD (US ignored)"));
  }
  // u = RETRACT fully and HOLD (ignores ultrasonics completely)
  if(c=='u'){
    cmdExtendHold  = false;
    cmdRetractHold = true;
    fullyExtended  = false;
    fullyRetracted = false;
    startMove(false,now);
    Serial.println(F(">> RETRACT & HOLD (US ignored)"));
  }
  // s = STOP (halts the actuator wherever it is)
  if(c=='s'){
    actMoving=false; driveStop();
    Serial.println(F(">> STOP"));
  }
  // r and a kept as aliases so old senders don't break:
  //   r behaves like u (retract & hold), a does nothing harmful (no auto now)
  if(c=='r'){
    cmdExtendHold  = false;
    cmdRetractHold = true;
    fullyExtended  = false;
    fullyRetracted = false;
    startMove(false,now);
    Serial.println(F(">> RETRACT & HOLD (alias r)"));
  }
  if(c=='a'){
    // No auto mode anymore - actuator is command-only. Acknowledge only.
    Serial.println(F(">> (auto disabled - command-only)"));
  }
}
