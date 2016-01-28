//Quadricopter Test Program

#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define PIDSTEP 25
#define LED_VOLTAGE 100
#define MINTHROTTLE 1000
#define MAXTHROTTLE 2000
#define SAFELIMIT_LOW 1450
#define SAFELIMIT_HIGH 1850
#define CAMERA_1_MID 98
#define CAMERA_1_RIGHT 168
#define CAMERA_1_LEFT 28
#define CAMERA_2_FRONT 140
#define CAMERA_2_BELOW 60
#define MIN_DISTANCE_SIDE 220
#define MIN_DISTANCE_TOP 300
#define MPU_STABLE_TIME 20000

MPU6050 mpu(0x68);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw, pitch, roll;
double yaw_u, pitch_u, roll_u;
float yaw_setpoint, pitch_setpoint, roll_setpoint;
float yaw_lastErr = 0.0, pitch_lastErr = 0.0, roll_lastErr = 0.0;
boolean iniState = true;

//PID parameters choice
float yaw_kp = 0.0, yaw_kd = 0.0;
float pitch_kp = 0.005, pitch_kd = 200.0;
//float pitch_kp = 0.0, pitch_kd = 0.0;
float roll_kp = 0.5, roll_kd = 1000.0;
//float roll_kp = 0.0, roll_kd = 0.0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady(){
  mpuInterrupt = true;
}

Servo motor_1;
Servo motor_2;
Servo motor_3;
Servo motor_4;
Servo camera_1;
Servo camera_2;

int infraredpin1 = A1;
int infraredpin2 = A2;
int infraredpin3 = A3;
int infraredpin4 = A4;
int infraredpin5 = A5;

int ledP = 42;
int ledN = 43;
int ledVoltage = 12;

int motor_power = 4 * MINTHROTTLE;
int motor_1_cst = MINTHROTTLE;
int motor_2_cst = MINTHROTTLE;
int motor_3_cst = MINTHROTTLE;
int motor_4_cst = MINTHROTTLE;
int camera_1_cst = CAMERA_1_MID;
int camera_2_cst = CAMERA_2_BELOW;

int dis_left = 0;
int dis_forward = 0;
int dis_right = 0;
int dis_back = 0;
int dis_top = 0;

//int GPS_BaudRate = ;
int APC_BaudRate = 9600;
int ledpin = 13;
int flightTimer = 0;
int controlTimer = 0;
int gpsTimer = 0;

String GPS_Signal;
int GPS_counter = 0;
boolean GPS_indoor = true;

char input;

void quadInitialize(void);
void radioControl(char c);
void flightPatrol(void);
void compute(void);
void gpsDataProcess(void);
void readline(void);

void keyResponseW(void);
void keyResponseS(void);
void keyResponseA(void);
void keyResponseD(void);
void keyResponseQ(void);
void keyResponseE(void);
void keyResponseR(void);
void keyResponseF(void);
void keyResponseI(void);
void keyResponseK(void);
void keyResponseJ(void);
void keyResponseL(void);
void keyResponseU(void);
void keyResponseO(void);
void keyResponseY(void);
void keyResponseH(void);
void keyResponseG(void);
void keyResponseT(void);
void keyResponseX(void);

void setup(){
  Serial.begin(9600);
  Serial1.begin(APC_BaudRate);
//  Serial2.begin(GPS_BaudRate);

  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
//  mpu.setFIFOEnabled(false);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }

  pinMode(ledpin, OUTPUT);
  digitalWrite(ledpin, LOW);

  pinMode(ledP, OUTPUT);
  pinMode(ledN, OUTPUT);
  pinMode(ledVoltage, OUTPUT);

  motor_1.attach(3);
  motor_2.attach(4);
  motor_3.attach(5);
  motor_4.attach(6);
  camera_1.attach(10);
  camera_2.attach(11);

  quadInitialize();

  delay(20000);
  digitalWrite(ledpin, HIGH);
}

void loop(){
  if(!dmpReady) return;
  if(!mpuInterrupt && fifoCount < packetSize) return;

  //50Hz
  if(millis() - flightTimer > PIDSTEP - 1){
    flightTimer = millis();
    flightPatrol();
  }
  //20Hz
  if(millis() - controlTimer > 49){
    controlTimer = millis();
    if(Serial1.available()){
      input = Serial1.read();
      radioControl(input);
    }
  }
  //2Hz
  if(millis() - gpsTimer > 499){
    gpsTimer = millis();
    gpsDataProcess();
  }
}

void quadInitialize(void){

  digitalWrite(ledP, LOW);
  digitalWrite(ledN, LOW);
  analogWrite(ledVoltage, LED_VOLTAGE);

  camera_1.write(CAMERA_1_MID);
  camera_2.write(CAMERA_2_BELOW);
  motor_1.write(MAXTHROTTLE);
  motor_2.write(MAXTHROTTLE);
  motor_3.write(MAXTHROTTLE);
  motor_4.write(MAXTHROTTLE);
  delay(1000);

  motor_1.write(MINTHROTTLE);
  motor_2.write(MINTHROTTLE);
  motor_3.write(MINTHROTTLE);
  motor_4.write(MINTHROTTLE);
  delay(2000);
}

void flightPatrol(void){
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();
  if((mpuIntStatus & 0x10) || fifoCount == 1024){
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer); //Take Quaternion from DMP
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);  //Take Yaw, Pitch, Roll from DMP, in radius

    //yaw, pitch, roll in degrees
    yaw = ypr[0] * 180 / M_PI;
    pitch = ypr[1] * 180 / M_PI;
    roll = ypr[2] * 180 / M_PI;

    if(iniState == true){
      yaw_setpoint = yaw;
      pitch_setpoint = 0.0;
      roll_setpoint = 45.0;
      iniState = false;
    }
    compute();
  }

  dis_left = analogRead(infraredpin1);
  dis_forward = analogRead(infraredpin2);
  dis_right = analogRead(infraredpin3);
  dis_back = analogRead(infraredpin4);
  dis_top = analogRead(infraredpin5);

  //
/*
  if(motor_1_cst < SAFELIMIT_LOW) motor_1_cst = SAFELIMIT_LOW;
  else if(motor_1_cst > SAFELIMIT_HIGH) motor_1_cst = SAFELIMIT_HIGH;
  if(motor_2_cst < SAFELIMIT_LOW) motor_2_cst = SAFELIMIT_LOW;
  else if(motor_2_cst > SAFELIMIT_HIGH) motor_2_cst = SAFELIMIT_HIGH;
  if(motor_3_cst < SAFELIMIT_LOW) motor_3_cst = SAFELIMIT_LOW;
  else if(motor_3_cst > SAFELIMIT_HIGH) motor_3_cst = SAFELIMIT_HIGH;
  if(motor_4_cst < SAFELIMIT_LOW) motor_4_cst = SAFELIMIT_LOW;
  else if(motor_4_cst > SAFELIMIT_HIGH) motor_4_cst = SAFELIMIT_HIGH;
*/

  if(motor_1_cst > SAFELIMIT_HIGH) motor_1_cst = SAFELIMIT_HIGH;
  if(motor_2_cst > SAFELIMIT_HIGH) motor_2_cst = SAFELIMIT_HIGH;
  if(motor_3_cst > SAFELIMIT_HIGH) motor_3_cst = SAFELIMIT_HIGH;
  if(motor_4_cst > SAFELIMIT_HIGH) motor_4_cst = SAFELIMIT_HIGH;

/*
  Serial.print(millis());Serial.print("\t");
  Serial.print(motor_1_cst);Serial.print("\t");
  Serial.print(motor_2_cst);Serial.print("\t");
  Serial.print(motor_3_cst);Serial.print("\t");
  Serial.print(motor_4_cst);Serial.print("\t");
  Serial.print(yaw);Serial.print("\t");
//  Serial.print(yaw_u);Serial.print("\t");
  Serial.print(pitch);Serial.print("\t");
//  Serial.print(pitch_u);Serial.print("\t");
  Serial.println(roll); //Serial.print("\t");
//  Serial.println(roll_u);
*/
  motor_1.write(motor_1_cst);
  motor_2.write(motor_2_cst);
  motor_3.write(motor_3_cst);
  motor_4.write(motor_4_cst);
  camera_1.write(camera_1_cst);
  camera_2.write(camera_2_cst);
  
}

void compute(void){
  float yaw_error = yaw_setpoint - yaw;
  float pitch_error = pitch_setpoint - pitch;
  float roll_error = roll_setpoint - roll;
/*
  Serial.print(yaw_error); Serial.print("\t");
  Serial.print(pitch_error); Serial.print("\t");
  Serial.println(roll_error);
*/
  float yaw_dErr = (yaw_error - yaw_lastErr) / PIDSTEP;
  float pitch_dErr = (pitch_error - pitch_lastErr) / PIDSTEP;
  float roll_dErr = (roll_error - roll_lastErr) / PIDSTEP;

  yaw_u = yaw_kp * yaw_error + yaw_kd * yaw_dErr;
  pitch_u = pitch_kp * pitch_error + pitch_kd * pitch_dErr;
  roll_u = roll_kp * roll_error + roll_kd * roll_dErr;

  yaw_lastErr = yaw_error;
  pitch_lastErr = pitch_error;
  roll_lastErr = roll_error;

  motor_1_cst = 0.25 * (motor_power + pitch_u + roll_u + yaw_u);
  motor_2_cst = 0.25 * (motor_power - pitch_u + roll_u - yaw_u);
  motor_3_cst = 0.25 * (motor_power - pitch_u - roll_u + yaw_u);
  motor_4_cst = 0.25 * (motor_power + pitch_u - roll_u - yaw_u);
}

//$GPRMC Data Process
//tech progress check - manipulated signals
void gpsDataProcess(void){
  if(GPS_indoor == true){
    GPS_Signal = "GPS Signal Undeteced";
  }
  else{
    if(GPS_counter == 0) GPS_Signal = "3101.58791,N,12126.17560,E";
    else if(GPS_counter == 1) GPS_Signal = "3101.38802,N,12126.28234,E";
    else if(GPS_counter == 2) GPS_Signal = "3101.39183,N,12126.29530,E";
    else if(GPS_counter == 3) GPS_Signal = "3101.44447,N,12126.17344,E";
    else if(GPS_counter == 4) GPS_Signal = "3101.55936,N,12126.19265,E";
    else if(GPS_counter == 5) GPS_Signal = "3101.48266,N,12126.23777,E";
    else if(GPS_counter == 6) GPS_Signal = "3101.57746,N,12126.26330,E";
    else if(GPS_counter == 7) GPS_Signal = "3101.39533,N,12126.21460,E";
    else if(GPS_counter == 8) GPS_Signal = "3101.47941,N,12126.25080,E";
    else if(GPS_counter == 9) GPS_Signal = "3101.45064,N,12126.18674,E";
  }
}

//Embeded in gpsDataProcess
void readline(void){
  //
}

void radioControl(char c){
  if(c == 'w') keyResponseW(); //flight control part
  else if(c == 's') keyResponseS();
  else if(c == 'a') keyResponseA();
  else if(c == 'd') keyResponseD();
  else if(c == 'q') keyResponseQ();
  else if(c == 'e') keyResponseE();

  else if(c == 'r') keyResponseR(); //take off and land
  else if(c == 'f') keyResponseF();

  else if(c == 'i') keyResponseI(); //camera motion control
  else if(c == 'k') keyResponseK();
  else if(c == 'j') keyResponseJ();
  else if(c == 'l') keyResponseL();
  else if(c == 'u') keyResponseU();
  else if(c == 'o') keyResponseO();

  else if(c == 'y') keyResponseY(); //LED control
  else if(c == 'h') keyResponseH();

  else if(c == 'g') keyResponseG(); //GPS data read
  else if(c == 't') keyResponseT();

  else if(c == 'x') keyResponseX(); //End program
}

void keyResponseW(void){
  //testing code
  roll_setpoint = roll_setpoint + 0.5;
}

void keyResponseS(void){
  //testing code
  roll_setpoint = roll_setpoint - 0.5;
}

void keyResponseA(void){
  //testing code
  pitch_setpoint = pitch_setpoint + 0.5;
}

void keyResponseD(void){
  //testing code
  pitch_setpoint = pitch_setpoint - 0.5;
}

void keyResponseQ(void){
  //testing code
  motor_power = 4 * MINTHROTTLE;
}

void keyResponseE(void){
  //testing code
  motor_power = 1525 * 4;
}

void keyResponseR(void){
  //testing code
  motor_power = motor_power + 10;
  if(motor_power < 4 * MINTHROTTLE) motor_power = 4 * MINTHROTTLE;
  else if(motor_power > 4 * MAXTHROTTLE) motor_power = 4 * MAXTHROTTLE;
}

void keyResponseF(void){
  //testing code
  motor_power = motor_power - 10;
  if(motor_power < 4 * MINTHROTTLE) motor_power = 4 * MINTHROTTLE;
  else if(motor_power > 4 * MAXTHROTTLE) motor_power = 4 * MAXTHROTTLE;
}

//Camera Up
void keyResponseI(void){
  camera_2_cst = camera_2_cst + 4;
  if(camera_2_cst > CAMERA_2_FRONT){
    camera_2_cst = CAMERA_2_FRONT;
  }
  else if(camera_2_cst < CAMERA_2_BELOW){
    camera_2_cst = CAMERA_2_BELOW;
  }
}

//Camera Down
void keyResponseK(void){
  camera_2_cst = camera_2_cst - 4;
  if(camera_2_cst > CAMERA_2_FRONT){
    camera_2_cst = CAMERA_2_FRONT;
  }
  else if(camera_2_cst < CAMERA_2_BELOW){
    camera_2_cst = CAMERA_2_BELOW;
  }
}

//Camera Left
void keyResponseJ(void){
  camera_1_cst = camera_1_cst - 5;
  if(camera_1_cst > CAMERA_1_RIGHT){
    camera_1_cst = CAMERA_1_RIGHT;
  }
  else if(camera_1_cst < CAMERA_1_LEFT){
    camera_1_cst = CAMERA_1_LEFT;
  }
}

//Camera Right
void keyResponseL(void){
  camera_1_cst = camera_1_cst + 5;
  if(camera_1_cst > CAMERA_1_RIGHT){
    camera_1_cst = CAMERA_1_RIGHT;
  }
  else if(camera_1_cst < CAMERA_1_LEFT){
    camera_1_cst = CAMERA_1_LEFT;
  }
}

//Camera Mid + Front
void keyResponseU(void){
  camera_1_cst = CAMERA_1_MID;
  camera_2_cst = CAMERA_2_FRONT;
}

//Camera Bird's Eye
void keyResponseO(void){
  camera_1_cst = CAMERA_1_MID;
  camera_2_cst = CAMERA_2_BELOW;
}

//Lights On
void keyResponseY(void){
  digitalWrite(ledP, HIGH);
  digitalWrite(ledN, LOW);
  analogWrite(ledVoltage, LED_VOLTAGE);
}

//Lights Off
void keyResponseH(void){
  digitalWrite(ledP, LOW);
  digitalWrite(ledN, LOW);
  analogWrite(ledVoltage, LED_VOLTAGE);
}

//Send GPS Signal
void keyResponseG(void){
  Serial1.println(GPS_Signal);
  GPS_counter++;
  if(GPS_counter == 10) GPS_counter = 0;
}

void keyResponseT(void){
  if(GPS_indoor == true) GPS_indoor = false;
  else GPS_indoor = true;
}

//Terminatinating program
void keyResponseX(void){
  //
}
