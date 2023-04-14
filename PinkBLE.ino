#include <ArduinoBLE.h>
//#include <pico/multicore.h>
//#include <Arduino_LSM6DSOX.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
bool debugPrint = 1;

//LittleFS stuff
#define LFS_MBED_RP2040_VERSION_MIN_TARGET      "LittleFS_Mbed_RP2040 v1.1.0"
#define LFS_MBED_RP2040_VERSION_MIN             1001000
#define _LFS_LOGLEVEL_          1
#define RP2040_FS_SIZE_KB       64
#define FORCE_REFORMAT          false
#include <LittleFS_Mbed_RP2040.h>
LittleFS_MBED *myFS;
char ServoFile[] = MBED_LITTLEFS_FILE_PREFIX "/servoData.txt";//Servo data file.
char ServoChars[216];//Char array holding servo data for r/w.
int ServoInts[72];//Int array holding min,max,centers of servos. HipMin, HipCenter, HipMax, KneeMin, KneeCenter, KneeMax, AnkleMin, AnkleCenter, AnkleMax

//BluetoothSetup
BLEService RCService("27810000-4786-42e7-bd27-45257b2e988f"); // BLE rc Service
BLEDescriptor RCLabelDescriptor("27810000-4786-42e7-bd27-45257b2e988f", "BLE_RC");
// BLE AppInventorRC Characteristic - custom 128-bit UUID, read and writable by central
BLEIntCharacteristic RC_AngleCharacteristic("27810001-4786-42e7-bd27-45257b2e988f", BLERead | BLEWrite | BLEWriteWithoutResponse);
BLEIntCharacteristic RC_SpeedCharacteristic("27810002-4786-42e7-bd27-45257b2e988f", BLERead | BLEWrite | BLEWriteWithoutResponse);
BLEIntCharacteristic RC_HeightCharacteristic("27810003-4786-42e7-bd27-45257b2e988f", BLERead | BLEWrite | BLEWriteWithoutResponse);
//Characteristics for Setup and Calibration
BLEIntCharacteristic RC_RotationCharacteristic("27810004-4786-42e7-bd27-45257b2e988f", BLERead | BLEWrite | BLEWriteWithoutResponse);
BLEStringCharacteristic RC_LegCharacteristic("27810005-4786-42e7-bd27-45257b2e988f", BLERead | BLEWrite | BLEWriteWithoutResponse, 20);//Takes a String of values seperated by a comma(,).
BLEIntCharacteristic RC_LegDataTestCharacteristic("27810006-4786-42e7-bd27-45257b2e988f", BLERead | BLEWrite | BLEWriteWithoutResponse);
//Global Variables for data recived over bluetooth
int LastSpeed = 0;
int LastAngle = 360;
int LastHeight = 101;
int LastRot = 0;
String LastLeg = "leg";
//int LegInts[] = {0,0,0,0,0};//Value1 = Method( 1=pwm, 0=microseconds ), Value2 = Leg#, Hip, Knee, Ankle. Does this need to global?
int LastLegInts[] = {0,0,0,0,0};//Lastleg movements, used for debouncing. Could be used to slow down movement, but probably not necessary for calibrations.
int tick = 0;
bool FirstConnection = 1;

// Timer, plus a timer
#include <arduino-timer.h>
int phaseCount = 0;
int phaseSpeed = 5000;//Base Speed
int cycleSpeed = 0;
auto timer = timer_create_default(); // create a timer with default settings
//Timer<16, millis, void *> timer;
size_t repeat_count = 1;
bool phaseController(void *opaque) {
  size_t limit = (size_t)opaque;
  Serial.print("PhaseController: ");
  Serial.print(repeat_count);
  Serial.print("/");
  Serial.println(limit);
  timeCycle(phaseCount, cycleSpeed);
  phaseCount++;
  if(phaseCount == 8){
    phaseCount = 0;
  }
  return ++repeat_count <= limit; // remove this task after limit reached
}

///Phaser.ino Globals
int moveOrder[8];//Sequence of legs moves for gait
int phaseList[8];//current phase for leg

//TRIG globals that needed to be migrated to avoid "not declared in this scope" errors.
int FR_Angles[5];//minXkAngle, minXaAngle, maxXkAngle, maxXaAngle, distance in mm
int atk90[3];//knee, ankle that = 90 and have the height requirement. Third value is the x, distance away from the hipjoint.
int legAssignment[8];//0 = front leg(s), 1= left leg(s), 2 = rear leg(s), 3 = right leg(s)
float hip0Angle[8];// The direction each hip is facing when the servo is set to 0 degrees.
int hipRanges[8];
//8/30/22 New
int hipDist2Angle[8];
int numOf_leftLegs;
int numOf_rightLegs;
int leftOrder[3];//How many to access^^
int rightOrder[3];


//SERVOSTUFF
Adafruit_PWMServoDriver pwmR = Adafruit_PWMServoDriver();//Right PCA9685
Adafruit_PWMServoDriver pwmL = Adafruit_PWMServoDriver(0x43);//Left PCA9685
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates
// our servo # counter
uint8_t servonum = 0;
int lastAngles[24] = {0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0};//This will hold the last angles from a movement [leg*3+joint]

// Serial.print variable TYPE. Unecessary later.
// void types(String a) { Serial.println("it's a String"); }
// void types(int a) { Serial.println("it's an int"); }
// void types(char *a) { Serial.println("it's a char*"); }
// void types(float a) { Serial.println("it's a float"); }
// void types(bool a) { Serial.println("it's a bool"); }

void createRandomArray(char * servoFile){//Populates a file with an array of random chars for LittleFS to store.
  int values[72];
  char output[216];
  for(int i = 0; i < (3*3*8); i++){//Generate 72 random numbers in place of low, mid, and high for 24 servos.
    values[i] = random(50,700);
  }
  for(int i = 0; i < (3*3*8); i++){// Breaks the random numbers into an array of chars.
    Serial.print(i);
    Serial.print(": ");
    Serial.println(values[i]);
    //Convert array of ints into array of chars
    output[i*3] = ((values[i]/100) % 10) + '0';
    output[i*3+1] = ((values[i]/10) % 10) + '0';
    output[i*3+2] = (values[i] % 10) + '0';
  }  
  writeFile(servoFile, output, sizeof(output));
}
void saveServosArray(){
  for(int i = 0; i < (3*3*8); i++){// Breaks the random numbers into an array of chars.
    //Serial.print(i);
    //Serial.print(": ");
    //Serial.println(ServoInts[i]);
    //Convert array of ints into array of chars
    ServoChars[i*3] = ((ServoInts[i]/100) % 10) + '0';
    ServoChars[i*3+1] = ((ServoInts[i]/10) % 10) + '0';
    ServoChars[i*3+2] = (ServoInts[i] % 10) + '0';
  }  
  writeFile(ServoFile, ServoChars, sizeof(ServoChars));
  //readCharsFromFile(ServoFile);
}
void writeFile(const char * path, const char * message, size_t messageSize){ //LittleFS writer.
  myFS = new LittleFS_MBED();
  if (!myFS->init()) 
  {
    Serial.println("LITTLEFS Mount Failed");
    return;
  }
  Serial.print("Writing file: "); Serial.print(path);

  FILE *file = fopen(path, "w");
  if (file){
    Serial.println(" => Open OK");
  }
  else{
    Serial.println(" => Open Failed");
    return;
  } 
  if (fwrite((uint8_t *) message, 1, messageSize, file)){
    Serial.println("* Writing OK");
  } 
  else{
    Serial.println("* Writing failed");
  }  
  fclose(file);
  if(myFS->unmount()){
    Serial.println( "WriteComplete" );
  }
}
void readCharsFromFile(const char * path){ //LittleFS reader. Modify to send to global array.
  myFS = new LittleFS_MBED();
  if (!myFS->init()) 
  {
    Serial.println("LITTLEFS Mount Failed");
    return;
  }
  Serial.print("readCharsFromFile: "); Serial.print(path);

  FILE *file = fopen(path, "r");
  
  if (file) 
  {
    Serial.println(" => Open OK");
  }
  else
  {
    Serial.println(" => Open Failed");
    return;
  }

  char c;
  int i = 0;
  while (true) 
  {
    c = fgetc(file);
    
    if ( feof(file) ) 
    { 
      break;
    }
    else{   
      Serial.print(c);
      //write to global ServoChar array.
      ServoChars[i++] = c;
    }
  }
  Serial.println(" ");

  fclose(file);
  if(myFS->unmount()){
    Serial.println( "\nReadComplete" );
  }
}
void pwmScan(){//Gives a list of all available I2C devices
  byte error, address;
  int nDevices;
    
  Serial.println(F("Scanning..."));
    
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
         
       
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
    
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println(F("No I2C devices found\n"));
  else
    Serial.println(F("done\n"));
}
void pwmCalm(){
  //Serial.println("Calm Down!");
  for(int i = 0; i < 8; i++){
    for(int j = 0; j < 3; j++){
      moveLeg(1, i, j, 4096);
      //delay(50);
    }
  }
  
}
void pwmReset(){
  Serial.println("pwmReset");
  pwmR.reset();
  pwmL.reset();
  pwmR.begin();
  pwmR.setOscillatorFrequency(27000000);
  pwmR.setPWMFreq(SERVO_FREQ);  
  pwmL.begin();
  pwmL.setOscillatorFrequency(27000000);
  pwmL.setPWMFreq(SERVO_FREQ);
}
void pwmCheck(){//Testing to see what this function does. later add Leg and Joint ID <<----
  Serial.println("PwmCheck-R: ");
  for(uint8_t i = 0; i < 16; i++){
    Serial.print(i);
    Serial.print(":");
    Serial.print(pwmR.getPWM(i));
    Serial.print(" ");
  }
  Serial.println(" ");
  Serial.println(" PwmCheck-L: ");
  for(uint8_t i = 0; i < 16; i++){
    Serial.print(i);
    Serial.print(":");
    Serial.print(pwmL.getPWM(i));
    Serial.print(" ");
  }
  Serial.println(" ");
  pwmCalm();
}
void moveLeg(bool methodPwm, int leg, int joint, int val){//Move Servos using setPWM() Used for testing...
  // Serial.print("Leg:");
  // Serial.print(leg);
  // Serial.print(" Method:");
  // Serial.print(methodPwm);
  if(leg < 4){//RightBoard
    servonum = leg * 4 + joint;
    // Serial.print(" Moving pwmR, channel:");
    // Serial.print(servonum);
    // Serial.print(" Value:");
    // Serial.println(val);
    if(methodPwm){
      pwmR.setPWM(servonum, 0, val);
    }
    else
      pwmR.writeMicroseconds(servonum, val);
    // if (joint == 0 || 2){//Release?
    //   //delay(50);
    //   pwmR.setPWM(servonum, 0, 4096);
    // }
  }
  else{//LeftBoard
    servonum = (leg - 4) * 4 + joint;
    // Serial.print(" Moving pwmL, channel:");
    // Serial.print(servonum);
    // Serial.print(" Value:");
    // Serial.println(val);
    if(methodPwm){
      pwmL.setPWM(servonum, 0, val);
    }
    else
      pwmL.writeMicroseconds(servonum, val);
    // if (joint == 0 || 2){//Release
    //   //delay(50);
    //   pwmR.setPWM(servonum, 0, 4096);
    // }
  }
}
void moveSave(int leg, int joint, float angle){//Same as above, but takes a floating angle.Move Servos using setPWM(), save last position as an angle. Takes an angle to reduce the number of maps called.
  int val = findJointAngle(leg, joint, angle);
  // Serial.print("Leg:");
  // Serial.print(leg);
  // Serial.print(" Joint:");
  // Serial.print(joint);
  // Serial.print(" Val:");
  // Serial.println(val);

  //Serial.println("got This Far0");
  
  lastAngles[leg*3+joint] = angle;
  if(leg < 4){//RightBoard
    pwmR.setPWM(leg * 4 + joint, 0, val);
  }
  else{//LeftBoard
    pwmL.setPWM((leg - 4) * 4 + joint, 0, val);
  }
  if(joint ==2 && (angle == 90 || angle ==0)){
    moveLeg(1, leg, joint, 4096);
    Serial.println("Endstop: 0/90");
  }
}
int medianCenter(int leg, int joint){// Returns midpoint between set min and max. joint 0, 1, or 2...
  //HipMin 0, HipCenter 1, HipMax 2, KneeMin 3, KneeCenter 4, KneeMax 5, AnkleMin 6, AnkleCenter 7, AnkleMax 8 
  // Serial.print(" ServoMax:");
  // Serial.print(ServoInts[leg*9+(joint*3)+2]);
  // Serial.print(" | ServoMin:");
  // Serial.print(ServoInts[leg*9+(joint*3)]);
  return ((ServoInts[leg*9+(joint*3)+2] + ServoInts[leg*9+(joint*3)])/2);
}
void moveKnees(bool sequential, int position){ // Moves all knees. Sequential or simultaneously, down(0), up(1), center(2).
  if(sequential){//SEQUENTIAL
    for(int i = 0; i< 8; i++){ // HipMin, HipCenter, HipMax, KneeMin, KneeCenter, KneeMax, AnkleMin, AnkleCenter, AnkleMax
      if(position == 0){//All Knees from min to max(Down)
        for(int kPos = ServoInts[i*9+5]; kPos >= ServoInts[i*9+3]; kPos-- ){ 
          moveLeg(1, i, 1, kPos);
        }
      }
      else if(position ==1){//All Knees from Max -> Min(UP)
        for(int kPos = ServoInts[i*9+3]; kPos <= ServoInts[i*9+5]; kPos++ ){ 
          moveLeg(1, i, 1, kPos);
        }
      }
    }
  }
  else {//Simultaneously
    int steps = 250;
    if(position == 0){ //Down
      for(int s = 1; s <= steps; s++){
        for(int i = 0; i < 8; i++){//#of legs
          int kPos = ServoInts[i*9+5]-((((ServoInts[i*9+5]-ServoInts[i*9+3]) *10) /steps)*s/10);//Max - range/steps *step.Extra Math to avoid floats.
          if(kPos >= ServoInts[i*9+3]){
            moveLeg(1, i, 1, kPos);
          }
          else{
            Serial.print("Capped at Min:");
            Serial.println(ServoInts[i*9+3]);
            moveLeg(1, i, 1, ServoInts[i*9+3]);
          }
        }
      }
    }
    else if(position ==1){//UP
      for(int s = 1; s <= steps; s++){
        for(int i = 0; i < 8; i++){//#of legs
          int kPos = ServoInts[i*9+3]+((((ServoInts[i*9+5]-ServoInts[i*9+3]) *10) /steps)*s/10);//Max - range/steps *step.Extra Math to avoid floats.
          if(kPos <= ServoInts[i*9+5]){
            moveLeg(1, i, 1, kPos);
          }
          else{
            Serial.print("Capped at Max:");
            Serial.println(ServoInts[i*9+5]);
            moveLeg(1, i, 1, ServoInts[i*9+5]);
          }
        }
      }
    }
  }
}
void moveAnkles(bool sequential, int pos){ // Moves all knees. Sequential or simultaneously, up(1), down(0), or center(3).
  if(sequential){//SEQUENTIAL
    for(int i = 0; i< 8; i++){ // HipMin, HipCenter, HipMax, KneeMin, KneeCenter, KneeMax, AnkleMin, AnkleCenter, AnkleMax
      if(pos == 0){  //All ANKLES from MAX to Min(Down)
        for(int aPos = ServoInts[i*9+8]; aPos >= ServoInts[i*9+6]; aPos-- ){ 
          moveLeg(1, i, 2, aPos);
        }
      }
      else if(pos == 1) { //All Ankles from Min to MAX(UP)
        for(int aPos = ServoInts[i*9+6]; aPos <= ServoInts[i*9+8]; aPos++ ){
          moveLeg(1, i, 2, aPos);
        }
      }
    }
  }
  else {//Simultaneously
    int steps = 250;
    if(pos == 0){ //Down
      for(int s = 1; s <= steps; s++){
        for(int i = 0; i < 8; i++){//#of legs
          int aPos = ServoInts[i*9+8]-((((ServoInts[i*9+8]-ServoInts[i*9+6]) *10) /steps)*s/10);//Max - range/steps *step.Extra Math to avoid floats.
          if(aPos >= ServoInts[i*9+6]){
            moveLeg(1, i, 2, aPos);
          }
          else{
            Serial.print("Capped at Min:");
            Serial.println(ServoInts[i*9+6]);
            moveLeg(1, i, 2, ServoInts[i*9+6]);
          }
        }
      }
    }
    else if(pos == 1){//UP
      for(int s = 1; s <= steps; s++){
        for(int i = 0; i < 8; i++){//#of legs
          int aPos = ServoInts[i*9+6]+((((ServoInts[i*9+8]-ServoInts[i*9+6]) *10) /steps)*s/10);//Max - range/steps *step.Extra Math to avoid floats.
          if(aPos <= ServoInts[i*9+8]){
            moveLeg(1, i, 2, aPos);
          }
          else{
            Serial.print("Capped at Max:");
            Serial.println(ServoInts[i*9+8]);
            moveLeg(1, i, 2, ServoInts[i*9+8]);
          }
        }
      }
    }
  }
}
void moveHips(bool sequential, int position){// position 0 = minimum LEGS 3 and 4 are reversed...
  if(sequential){
    for(int i = 0; i< 8; i++){ // HipMin, HipCenter, HipMax, KneeMin, KneeCenter, KneeMax, AnkleMin, AnkleCenter, AnkleMax
      Serial.print("Leg ");
      Serial.print(i);
      if(position == 0){  //All HIPS to minimum value, Counter Clockwise
        Serial.print(" Min:");
        Serial.println(ServoInts[i*9+0]);
        for(int hPos = ServoInts[i*9+2]; hPos >= ServoInts[i*9+0]; hPos-- ){ 
          moveLeg(1, i, 0, hPos);
        }
      }
      else if(position == 1) { //All HIPS from MIN to MAX, Clockwise
        Serial.print(" Max:");
        Serial.println(ServoInts[i*9+2]);
        for(int hPos = ServoInts[i*9+0]; hPos <= ServoInts[i*9+2]; hPos++ ){
          moveLeg(1, i, 0, hPos);
        }
      }
    }
  }
  else{//Simultaneously
  for(int m = 0; m < 8; m++){
    Serial.print("Leg:");
    Serial.print(m);
    Serial.print(" Min:");
    Serial.print(ServoInts[m*9+0]);
    Serial.print(" Max:");
    Serial.println(ServoInts[m*9+2]);
  }
    int steps = 250;
    //if(position == 0){//Move to mins
    for(int s = 1; s <= steps; s++){
      for(int i = 0; i < 8; i++){//#of legs
        
        if(position == 0 && i != 3 && i != 4){//Move to mins
          int hPos = ServoInts[i*9+2]-((((ServoInts[i*9+2]-ServoInts[i*9+0]) *10) /steps)*s/10);//Max - range/steps *step.Extra Math to avoid floats.
          if(hPos >= ServoInts[i*9+0]){
            moveLeg(1, i, 0, hPos);
          }
        }
        else if(position == 0){//Move 3&4 opposite to max
          int hPos = ServoInts[i*9+0]+((((ServoInts[i*9+2]-ServoInts[i*9+0]) *10) /steps)*s/10);//Max - range/steps *step.Extra Math to avoid floats.
          if(hPos <= ServoInts[i*9+2]){
            moveLeg(1, i, 0, hPos);
          }
        }
        else if(position == 1 && i != 3 && i != 4){//Move to maxs
          int hPos = ServoInts[i*9+0]+((((ServoInts[i*9+2]-ServoInts[i*9+0]) *10) /steps)*s/10);//Max - range/steps *step.Extra Math to avoid floats.
          if(hPos <= ServoInts[i*9+2]){
            moveLeg(1, i, 0, hPos);
          }
        }
        else if(position == 1){//If 3 or 4, move to mins
          int hPos = ServoInts[i*9+2]-((((ServoInts[i*9+2]-ServoInts[i*9+0]) *10) /steps)*s/10);//Max - range/steps *step.Extra Math to avoid floats.
          if(hPos >= ServoInts[i*9+0]){
            moveLeg(1, i, 0, hPos);
          }
        }
      }
    }
  }
}
int findJointAngle(int leg, int joint, float angle){//input leg, joint, desired angle. output servo PWM signal for required angle.
  //HipMin 0, HipCenter 1, HipMax 2, KneeMin 3, KneeCenter 4, KneeMax 5, AnkleMin 6, AnkleCenter 7, AnkleMax 8  
  int mappedJoint = 999; 
  if(joint == 0 && (angle >= -45 && angle <= 45)){//Positive Values move clockwise
    if(leg != 3 && leg !=4){ //Input ranges from +45 to -45
      mappedJoint = map(angle, -45, 45, ServoInts[leg*9+0], ServoInts[leg*9+2]);
    }
    else if(angle >= -35 && angle <= 35){ // Input ranges from +35 to -35. These Servos(3&4) also run backward....
      mappedJoint = map(angle, -35, 35, ServoInts[leg*9+2], ServoInts[leg*9+0]);
    }
  }
  else if( joint == 1 && (angle >= -10 && angle <= 100)){//KneeAngles 
    mappedJoint = map(angle, 0, 90, ServoInts[leg*9+5], ServoInts[leg*9+3]);
  }
  else if( joint == 2 && (angle >= 0 && angle <=90)){//AnkleAngles All ANKLES from MAX to Min(Down)
    mappedJoint = map(angle, 0,90,ServoInts[leg*9+6],ServoInts[leg*9+8]);
  }
  if(mappedJoint != 999){
    // Serial.print(" Mapped:");
    // Serial.println(mappedJoint);
    return mappedJoint;
  }
}
void setup() {
  Serial.begin(9600);
  //while (!Serial)//This prevents headlessmode(not connected to a serial monitor/pc)
    delay(1000);

  // Begin BLE initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");
  }
  // set advertised local name and service UUID:
  BLE.setLocalName("Pinky_RX");
  BLE.setAdvertisedService(RCService);
  // add the characteristics to the service
  RCService.addCharacteristic(RC_AngleCharacteristic);
  RCService.addCharacteristic(RC_SpeedCharacteristic);
  RCService.addCharacteristic(RC_HeightCharacteristic);
  RCService.addCharacteristic(RC_RotationCharacteristic);
  RCService.addCharacteristic(RC_LegCharacteristic);
  RCService.addCharacteristic(RC_LegDataTestCharacteristic);  
  // add service
  BLE.addService(RCService);
  // start advertising
  BLE.advertise();
  Serial.println("Pinky Peripheral, waiting for connections....");

  //Initialize pca9685 boards
  pwmR.begin();
  pwmR.setOscillatorFrequency(27000000);
  pwmR.setPWMFreq(SERVO_FREQ);  
  pwmL.begin();
  pwmL.setOscillatorFrequency(27000000);
  pwmL.setPWMFreq(SERVO_FREQ);

  //Pathing from Trig.ino
  setHipRanges();
  numOf_leftLegs = 0;
  numOf_rightLegs = 0;
  generate_HeightTable();//This does not need to be run every time. Calculates height at combined angles. Trig.ino


  //FileSystem checks OS...
  //   #if defined(LFS_MBED_RP2040_VERSION_MIN)
  // if (LFS_MBED_RP2040_VERSION_INT < LFS_MBED_RP2040_VERSION_MIN)
  // {
  //   Serial.print("Warning. Must use this on Version equal or later than : ");
  //   Serial.println(LFS_MBED_RP2040_VERSION_MIN_TARGET);
  // }
  // #endif
  //createRandomArray(ServoFile);//Run once for setup!
  readCharsFromFile(ServoFile);
  for (int i = 0; i < 72; i++){
    ServoInts[i] = ((ServoChars[i*3] - '0') * 100) + ((ServoChars[i*3+1] - '0') * 10) + (ServoChars[i * 3 + 2] - '0');
    // Serial.print(ServoInts[i]);
    // Serial.print(", ");
  }
  //Stand guard pose on timer
  //timer.every(50, standGuardSteps, (void *)90);
 
  
  
}
void loop() {
  //timer.tick();//So the timer can run while there is no bluetooth connected
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();
  // if a central is connected to peripheral:
  if (central) {
    if (FirstConnection){//Only runs when a connection is first established.
      FirstConnection = 0;
      Serial.print("Connected to central: ");
      Serial.println(central.address());// print the central's MAC address:
      Serial.print("Active Timers:");
      Serial.println(timer.size());
      if(timer.size() == 0){
        pwmCalm();
        pwmReset();//disconnecting and reconnecting will reset the boards if timers arent running
        //pwmScan();//Scan for pwm addresses
      }
      
    }
    while (central.connected()) {// while the central is still connected to peripheral:
      timer.tick(); // tick the timer here because this loop runs only while BLE is connected
      //Serial.print("Waiting for the write thing: ");
      //delay(100);//This causes hangs / crashes
      if (RC_AngleCharacteristic.written() || RC_SpeedCharacteristic.written() || RC_HeightCharacteristic.written() || RC_RotationCharacteristic.written()) {
        //Debounce the BLE
        if (LastAngle != RC_AngleCharacteristic.value() || LastSpeed != RC_SpeedCharacteristic.value() || LastHeight != RC_HeightCharacteristic.value() || LastRot != RC_RotationCharacteristic.value()){//Work this Later
          // Serial.print("Angle: ");
          // Serial.print(RC_AngleCharacteristic.value());
          // Serial.print(" Speed: ");
          // Serial.print(RC_SpeedCharacteristic.value());
          // Serial.print(" Height: ");
          // Serial.print(RC_HeightCharacteristic.value());
          // Serial.print(" Rotation: ");
          // Serial.println(RC_RotationCharacteristic.value());
          // if (LastAngle != RC_AngleCharacteristic.value() && timer.size() == 0 && RC_SpeedCharacteristic.value() != 0){
          //   // numOf_leftLegs = 0;//Reset the side count
          //   // numOf_rightLegs = 0;
          //   // for(int i=0; i<8; i++){
          //   //   hipTrajectory(i, RC_AngleCharacteristic.value());//Figure out leg orientation for the angle of attack
          //   // }
          //   // sideOrder();//In Trig.ino. Finds the order of side legs from front to back.
          //   // phaseCycle();//In Phaser.ino. Outputs the phase order for legs based upon the first front leg.
          //   // orientLegs(RC_AngleCharacteristic.value());
          //   // LastAngle = RC_AngleCharacteristic.value();
          //   // Serial.print("Speed setting while angle calculations:");
          //   // Serial.println(RC_SpeedCharacteristic.value());
          // }
          if(LastHeight != RC_HeightCharacteristic.value() && timer.size() == 0){//Call stuff from Trig.ino
            Serial.print("Calculating stuff for BLE height:");
            Serial.println(RC_HeightCharacteristic.value());
            find_frontPath(heightTranslator(RC_HeightCharacteristic.value()));//Updates global variable FRangles on this page.
            find_90attack(heightTranslator(RC_HeightCharacteristic.value()));
            LastHeight = RC_HeightCharacteristic.value();
          } 
          //LastAngle = RC_AngleCharacteristic.value();//This was moved to the conditional above
          LastSpeed = RC_SpeedCharacteristic.value();
          // LastHeight = RC_HeightCharacteristic.value();
          LastRot = RC_RotationCharacteristic.value();
        }
      }
      if (RC_LegCharacteristic.written()){  //LegCharacteristic is a String. Value1 = Method( 1=pwm, 0=microseconds ), Value2 = Leg#, 3=Hip, 4=Knee, 5=Ankle, 6=push to ServoInts[72]
        if(LastLeg != RC_LegCharacteristic.value()){//Debounce this BLE write
          LastLeg = RC_LegCharacteristic.value();
          char c[RC_LegCharacteristic.valueLength()];
          LastLeg.toCharArray(c, sizeof(c));
          //Serial.println(c);
          String pch;
          pch = strtok(c,",");
          int i = 0;
          int LegInts[] = {0,0,0,0,0};//Value1 = Method( 1=pwm, 0=microseconds ), Value2 = Leg#, Hip, Knee, Ankle
          while (pch != NULL && i < 6){//Transfter from BLE String to local array
            //types(pch);
            LegInts[i++] = pch.toInt();
            pch = strtok (NULL, ",");
          }
          // for(int i = 0; i < 5; ++i){
          //   Serial.println(LegInts[i]);
          // }
          if(LegInts[0] > 1){// Set value for min/max/center. 2 = HipMin, 3 = HipCenter, 4 = HipMax, 5 = KneeMin, 6KneeCenter, 7KneeMax, 8AnkleMin, 9AnkleCenter, 10AnkleMax. Use Switch...?
            Serial.print("Old Int Element: ");
            Serial.print(LastLegInts[1]*9+(LegInts[0]-2));
            Serial.print("  OldValue: ");
            Serial.println(ServoInts[LastLegInts[1]*9+(LegInts[0]-2)]);
            if(LegInts[0] < 5 && LastLegInts[2] != 0){//Hips
              ServoInts[LastLegInts[1]*9+(LegInts[0]-2)] = LastLegInts[2];
              Serial.print("NewHip:");
              Serial.println(LastLegInts[2]);
            }
            else if(LegInts[0] > 4 && LegInts[0] < 8 && LastLegInts[3] != 0){//Knees
              ServoInts[LastLegInts[1]*9+(LegInts[0]-2)] = LastLegInts[3];
              Serial.print("NewKnee:");
              Serial.println(LastLegInts[3]);
            }
            else if(LegInts[0] > 7 && LegInts[0] < 11 && LastLegInts[4] != 0){//Ankles
              ServoInts[LastLegInts[1]*9+(LegInts[0]-2)] = LastLegInts[4];
              Serial.print("NewAnkle:");
              Serial.println(LastLegInts[4]);
            }
            // Serial.print("NewServoInts:");
            // for(int i = 0; i < 72; i++){
            //   Serial.print(i);
            //   Serial.print(":");
            //   Serial.print(ServoInts[i]);
            //   Serial.print(" ");
            // } 
            Serial.println(" ");
            if(LegInts[0] >= 2 && LegInts[0] <= 10 && LastLegInts[0] != 0)  {
              saveServosArray();
            }         
          }
          else if(LegInts != LastLegInts){//New Leg Data! Move servos accordingly.
            Serial.print("Leg:");
            Serial.print(LegInts[1]);
            if(LegInts[2] != LastLegInts[2]){
              Serial.print(" Hip=");
              Serial.println(LegInts[2]);
              moveLeg(LegInts[0], LegInts[1], 0, LegInts[2]);
            }
              if(LegInts[3] != LastLegInts[3]){
              Serial.print(" Knee=");
              Serial.println(LegInts[3]);
              moveLeg(LegInts[0], LegInts[1], 1, LegInts[3]);
            }
              if(LegInts[4] != LastLegInts[4]){
              Serial.print(" Ankle=");
              Serial.println(LegInts[4]);
              moveLeg(LegInts[0], LegInts[1], 2, LegInts[4]);
            }
            
            for(int i = 0; i < 5; ++i){//Update the LastLegInts
              LastLegInts[i] = LegInts[i];  
              //Serial.println(LastLegInts[i]);
            }
          }

        }
      }
      if (RC_LegDataTestCharacteristic.written()){//Move joints to test out persistant data storage, Hips will be seperate from Ankles and Knees
        // Serial.print("Test Button: ");
        // Serial.println(RC_LegDataTestCharacteristic.value());
        if(RC_LegDataTestCharacteristic.value() == 0){//Simultaneously Contract all joints
          moveKnees(0, 0);
          moveAnkles(0, 0);
          //Move Hips Min. Forward or backward? Neither, LEFT/CCW!
          //moveHips(0,0);
          pwmCalm();
        }
        else if(RC_LegDataTestCharacteristic.value() == 1){//Contract all joints sequentially
          moveKnees(1, 0);
          moveAnkles(1, 0);
          //moveHips(1,0);//CCW
          pwmCalm();
        }
        else if(RC_LegDataTestCharacteristic.value() == 2){//Center by set Values. Measured angles?....
          Serial.println("SetCenterValues: ServoInts[leg*9+((joint*3)+1]");
        }
        else if(RC_LegDataTestCharacteristic.value() == 3){//Center my mid between min and max.
          for(int j = 0; j < 3; j++){
            for(int l = 0; l < 8; l++){
              // Serial.print("Leg:");
              // Serial.print(l);
              // Serial.print(" Joint:");
              // Serial.print(j);
              int center = medianCenter(l, j);

              moveLeg(1, l, j, center);
              delay(20);
              // Serial.print(" MidPoint:");
              // Serial.println(center);
            }
          }
          pwmCalm();
        }
        else if(RC_LegDataTestCharacteristic.value() == 4){//Expand all joints
          moveKnees(1, 1);//Method 1 = sequence, up
          moveAnkles(1, 1);
          //moveHips(1,1);
          pwmCalm();
        }
        else if(RC_LegDataTestCharacteristic.value() == 5){//Expand all joints
          moveKnees(0, 1);//Method 1 = Simultaneously, up
          moveAnkles(0, 1);
          //moveHips(0,1);
          pwmCalm();
        }
        else if(RC_LegDataTestCharacteristic.value() == 6){
          pwmCheck();
        }
        else if(RC_LegDataTestCharacteristic.value() == 7){
          moveHips(0,0);//CCW
        }
        else if(RC_LegDataTestCharacteristic.value() == 8){
          moveHips(0,1);//CCW
        }
      }
      if(LastSpeed > 0 && timer.size() == 0){//MOVE
        if (LastAngle != RC_AngleCharacteristic.value()){// if this takes too long in the cycle, only run it if the deviance is 15-45 degrees off of the last
          orientLegs(RC_AngleCharacteristic.value());
          LastAngle = RC_AngleCharacteristic.value();
        }
        cycleSpeed = LastSpeed;
        repeat_count = 1;
        // call the repeat_x_times(MOVE) function every 1000 millis (1 second)
        //timer.every(phaseSpeed/(LastSpeed/2), phaseController, (void *)8);
        timer.every(phaseSpeed/2, phaseController, (void *)8);
      }      
    }
    if(!FirstConnection){
      // when the central disconnects, print it out:
      Serial.print(F("Disconnected from central: "));
      Serial.println(central.address());
      FirstConnection = 1;
    }
  }
}