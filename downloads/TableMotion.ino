/* Table-motion feedback prototype: Arduino UNO R3 + Amperka AMP-B041 / LIS331DLH.
 * Wiring: AMP-B041: V=5V, G=GND, D(SDA)=A4, C(SCL)=A5.
 * LED: D6 ->330ohm->LED anode; cathode=GND.
 * PS1240 passive piezo: D9 ->1kohm->node; node->piezo(+),
 * node->1.5kohm->GND; piezo(-)->GND. The divider gives ~3V pulses.
 * Never connect 9V to the 5V pin. Battery+ via switch -> VIN, battery- ->GND.
 * Serial115200: r=recalibrate, +=higher threshold, -=lower threshold,
 * b=toggle sound, t=output test, p=status. Settings are not persistent.
 * This detects acceleration/vibration, not touch or displacement in mm.
 */
#include <Wire.h>
#include <math.h>

const uint8_t LED_PIN=6, PIEZO_PIN=9, READY_PIN=LED_BUILTIN;
const uint16_t RATE_HZ=100, SAMPLE_MS=1000/RATE_HZ, CAL_SAMPLES=300;
const float RAW_TO_G=0.001f/16.0f; // LIS331DLH +/-2g: 1mg per 12-bit digit, left aligned
const float MIN_THRESHOLD_G=0.025f; // starting value: tune on the actual table
const float MAX_CAL_NOISE_G=0.025f;
const uint32_t LED_HOLD_MS=1000, SOUND_MS=120, IGNORE_MS=1250;
const uint8_t REQUIRED_HITS=2;
enum Mode : uint8_t { CALIBRATING, ARMED, FAULT };
Mode mode=FAULT;
uint8_t address=0, hitCount=0;
bool soundEnabled=true, settling=false, alarmActive=false;
uint32_t lastSample=0, calStarted=0, alarmStarted=0, lastData=0, lastLog=0;
uint16_t calCount=0;
float meanG[3]={0,0,0}, m2[3]={0,0,0}, baseline[3]={0,0,0};
float thresholdG=MIN_THRESHOLD_G, noiseG=0, latestDynamicG=0;

bool writeReg(uint8_t reg,uint8_t value) {
  Wire.beginTransmission(address);Wire.write(reg);Wire.write(value);
  return Wire.endTransmission()==0;
}
bool readRegs(uint8_t reg,uint8_t *buf,uint8_t count) {
  Wire.beginTransmission(address);Wire.write(reg);
  if(Wire.endTransmission(false)!=0) return false;
  uint8_t got=Wire.requestFrom(address,count,(uint8_t)true);
  if(got!=count) {while(Wire.available()) Wire.read();return false;}
  for(uint8_t i=0;i<count;i++) buf[i]=Wire.read();
  return true;
}
void stopOutputs() {
  noTone(PIEZO_PIN);digitalWrite(PIEZO_PIN,LOW);digitalWrite(LED_PIN,LOW);
  digitalWrite(READY_PIN,LOW);alarmActive=false;
}
void fail(const __FlashStringHelper *message) {
  stopOutputs();mode=FAULT;hitCount=0;
  Serial.print(F("ERROR: "));Serial.println(message);
  Serial.println(F("Check wiring / keep table still; send r to retry."));
}
bool initSensor() {
  const uint8_t candidates[]={0x18,0x19};bool found=false;
  for(uint8_t i=0;i<2;i++) {
    address=candidates[i];uint8_t id=0;
    if(readRegs(0x0F,&id,1) && id==0x32) {found=true;break;}
  }
  if(!found) return false;
  // LIS331DLH: power down, bypass high-pass filter, no interrupts,
  // BDU enabled, little endian, +/-2g; normal mode at 100Hz, XYZ on.
  if(!writeReg(0x20,0x07)||!writeReg(0x21,0x00)||!writeReg(0x22,0x00)||
     !writeReg(0x23,0x80)||!writeReg(0x24,0x00)||!writeReg(0x30,0x00)||
     !writeReg(0x34,0x00)||!writeReg(0x20,0x2F)) return false;
  uint8_t control1=0,control4=0;
  return readRegs(0x20,&control1,1)&&readRegs(0x23,&control4,1)&&
         control1==0x2F && control4==0x80;
}
// 1=fresh XYZ, 0=not ready, -1=I2C error.
int8_t sampleSensor(float a[3]) {
  uint8_t status=0,b[6];
  if(!readRegs(0x27,&status,1)) return -1;
  if(!(status&0x08)) return 0;
  // Sub-address bit7 enables register auto-increment for all six bytes.
  if(!readRegs(0x28|0x80,b,6)) return -1;
  for(uint8_t i=0;i<3;i++) {
    int16_t raw=(int16_t)((uint16_t)b[2*i]|((uint16_t)b[2*i+1]<<8));
    a[i]=(float)raw*RAW_TO_G;
  }
  return 1;
}
void startCalibration() {
  stopOutputs();
  if(!initSensor()) {fail(F("LIS331DLH not found/configuration failed"));return;}
  calCount=0;hitCount=0;settling=false;noiseG=0;
  for(uint8_t i=0;i<3;i++) {meanG[i]=0;m2[i]=0;baseline[i]=0;}
  mode=CALIBRATING;calStarted=millis();lastData=calStarted;lastSample=calStarted;
  Serial.println(F("CALIBRATING: leave the table untouched for about 5 seconds."));
}
void status() {
  Serial.print(F("mode="));Serial.print((int)mode);
  Serial.print(F(" threshold_g="));Serial.print(thresholdG,4);
  Serial.print(F(" noise_g="));Serial.print(noiseG,4);
  Serial.print(F(" sound="));Serial.println(soundEnabled?F("on"):F("off"));
}
void signalEvent(bool test=false) {
  alarmStarted=millis();alarmActive=true;settling=true;hitCount=0;
  digitalWrite(LED_PIN,HIGH);
  if(soundEnabled) tone(PIEZO_PIN,4000,SOUND_MS);
  Serial.print(test?F("TEST"):F("MOVEMENT"));
  Serial.print(F(" dynamic_g="));Serial.println(latestDynamicG,4);
}
void serialCommands() {
  while(Serial.available()) {
    char c=Serial.read();
    if(c=='r'||c=='R') startCalibration();
    else if(c=='b'||c=='B') {soundEnabled=!soundEnabled;if(!soundEnabled) noTone(PIEZO_PIN);status();}
    else if(c=='p'||c=='P') status();
    else if(c=='t'||c=='T') {if(mode==ARMED) signalEvent(true);}
    else if(c=='+') {thresholdG=fminf(0.5f,thresholdG+0.005f);status();}
    else if(c=='-') {thresholdG=fmaxf(0.008f,thresholdG-0.005f);status();}
  }
}
void setup() {
  pinMode(LED_PIN,OUTPUT);pinMode(PIEZO_PIN,OUTPUT);pinMode(READY_PIN,OUTPUT);
  stopOutputs();Serial.begin(115200);Wire.begin();Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000,true);
#endif
  Serial.println(F("TableMotion UNO + LIS331DLH; prototype, thresholds require tuning."));
  startCalibration();
}
void loop() {
  serialCommands();uint32_t now=millis();
  if(mode==FAULT) {digitalWrite(LED_PIN,((now/125)%2)?HIGH:LOW);return;}
  if(mode==CALIBRATING) digitalWrite(LED_PIN,((now/350)%2)?HIGH:LOW);
  if(mode==ARMED) {
    digitalWrite(READY_PIN,(now%2000)<50?HIGH:LOW);
    if(alarmActive && (uint32_t)(now-alarmStarted)>=LED_HOLD_MS) {digitalWrite(LED_PIN,LOW);alarmActive=false;}
  }
  if((uint32_t)(now-lastSample)<SAMPLE_MS) return;
  lastSample=now;float a[3];int8_t result=sampleSensor(a);
  if(result<0) {fail(F("I2C read failed"));return;}
  if(result==0) {
    if((uint32_t)(now-lastData)>500) fail(F("No fresh accelerometer data"));
    return;
  }
  lastData=now;
  if(mode==CALIBRATING) {
    if((uint32_t)(now-calStarted)<1000) return;
    if((uint32_t)(now-calStarted)>8000) {fail(F("Calibration timed out"));return;}
    calCount++;
    for(uint8_t i=0;i<3;i++) {float delta=a[i]-meanG[i];meanG[i]+=delta/calCount;m2[i]+=delta*(a[i]-meanG[i]);}
    if(calCount<CAL_SAMPLES) return;
    noiseG=sqrtf((m2[0]+m2[1]+m2[2])/(CAL_SAMPLES-1));
    float gravity=sqrtf(meanG[0]*meanG[0]+meanG[1]*meanG[1]+meanG[2]*meanG[2]);
    if(noiseG>MAX_CAL_NOISE_G || gravity<0.75f || gravity>1.25f) {fail(F("Table moved during calibration / invalid gravity"));return;}
    thresholdG=fmaxf(MIN_THRESHOLD_G,6.0f*noiseG);
    for(uint8_t i=0;i<3;i++) baseline[i]=meanG[i];
    mode=ARMED;digitalWrite(LED_PIN,LOW);Serial.println(F("READY"));status();return;
  }
  if(settling) {
    if((uint32_t)(now-alarmStarted)<IGNORE_MS) return;
    // Re-arm from the settled pose; avoid treating the buzzer vibration as a new event.
    for(uint8_t i=0;i<3;i++) baseline[i]=a[i];
    settling=false;hitCount=0;return;
  }
  float sum=0;
  for(uint8_t i=0;i<3;i++) {float d=a[i]-baseline[i];sum+=d*d;}
  latestDynamicG=sqrtf(sum);
  if(latestDynamicG>thresholdG) {if(hitCount<255) hitCount++;} else hitCount=0;
  if(hitCount>=REQUIRED_HITS || latestDynamicG>2.0f*thresholdG) {signalEvent();return;}
  // Slow baseline tracking only when quiet; this intentionally ignores very slow drift.
  if(latestDynamicG<0.5f*thresholdG) for(uint8_t i=0;i<3;i++) baseline[i]+=0.005f*(a[i]-baseline[i]);
  if((uint32_t)(now-lastLog)>=250) {
    lastLog=now;Serial.print(F("g="));Serial.print(latestDynamicG,4);
    Serial.print(F(" threshold="));Serial.println(thresholdG,4);
  }
}
