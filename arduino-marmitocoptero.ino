#include <PID_v1.h>
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#include <Wire.h>
#include "rc_rx.h"

#define DEBUG
//#define DEBUG_RC //Print the RC information
//#define DEBUG_GYRO //Print GYRO and ACEL information
#define DEBUG_PID //Print PID constants information
#define DEBUG_EQUILIBRIO //Print aceleration, roll inclination and offset
#define AJUST_PID //Enable PID ajust by serial
//#define DEBUG_TIME

#define dt 20 //Tempo de variação do Loop 20ms

//Pinos dos motores
#define MOTOR_FD 0
#define MOTOR_FD_PIN 8  //Motor Frontal Direita
#define MOTOR_FE 1
#define MOTOR_FE_PIN 11 //Motor Frontal Esquerda
#define MOTOR_TD 2
#define MOTOR_TD_PIN 9 //Motor Traseiro Direita
#define MOTOR_TE 3
#define MOTOR_TE_PIN 12 //Motor Traseiro Esquerdo

#define LIMITE_PID 250 //Limite de aceleração para correção dos motores
#define LIMITE_Ki 50

//RC limits
#define TROTTLE_MIN 1000 //Leitura mínima
#define TROTTLE_INI 1100 //Início aceleração
#define TROTTLE_MAX 2000 //Leitura máxima


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

unsigned long t=0; //Time Variables

//pidRoll Roll
double SetpointRoll = 0, InputRoll = 0, OutputRoll = 0;
//pidRoll Pitch
double SetpointPitch = 0, InputPitch = 0, OutputPitch = 0;

double  P = 2,
I = 5,
D = 1.21;

PID pidRoll(&InputRoll, &OutputRoll, &SetpointRoll,P,I,D, DIRECT);
PID pidPitch(&InputPitch, &OutputPitch, &SetpointPitch,P,I,D, DIRECT);

//double P = 3.3,  
  //     I = 1.9,
    //   D = 1.5;
Servo motor[4];

int  offset = 0;
int aceleracao = TROTTLE_MIN;
int acelDireita = TROTTLE_MIN;
int acelEsquerda = TROTTLE_MIN;

void setup(){
  #ifdef DEBUG
    Serial.begin(115200);
  #endif
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  Serial.println("Inicializou");
  delay(100);

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(985);
  mpu.setYAccelOffset(-472);
  mpu.setZAccelOffset(1363);
  mpu.setXGyroOffset(24);
  mpu.setYGyroOffset(11);
  mpu.setZGyroOffset(41);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  init_rc_rx();

  motor[MOTOR_FD].attach(MOTOR_FD_PIN);
  motor[MOTOR_FE].attach(MOTOR_FE_PIN);

  SetpointRoll = 0;
  SetpointPitch = 0;

  pidRoll.SetMode(AUTOMATIC);
  pidRoll.SetOutputLimits(-LIMITE_PID,LIMITE_PID);
  pidRoll.setMaxOutputI(LIMITE_Ki);

  pidPitch.SetMode(AUTOMATIC);
  pidPitch.SetOutputLimits(-LIMITE_PID,LIMITE_PID);
  pidRoll.setMaxOutputI(LIMITE_Ki);

}
void loop(){

  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    roll = ypr[2] * 180/M_PI;
    pitch = ypr[1] * 180/M_PI;
    yaw = ypr[0] * 180/M_PI;
    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }

  //Ajuste PID
  #ifdef AJUST_PID
    ajustePID();
  #endif

  read_rc_rx();  
  
  aceleracao = CH3;
  
  if( aceleracao > TROTTLE_INI){
    InputRoll = roll;
    pidRoll.Compute();
  }
  offset = OutputRoll;

  if( aceleracao - offset < TROTTLE_MIN ){
    offset = aceleracao - TROTTLE_MIN;
  }

  if( aceleracao + offset > TROTTLE_MAX ){
    offset = TROTTLE_MAX - aceleracao;
  }

  if( aceleracao < TROTTLE_INI){
    aceleracao = TROTTLE_MIN;
    offset = 0;
  }
  
  acelDireita = aceleracao - offset;
  acelEsquerda = aceleracao + offset;

  motor[MOTOR_FD].writeMicroseconds(acelDireita);
  motor[MOTOR_FE].writeMicroseconds(acelEsquerda);

  #ifdef DEBUG_GYRO
    // Output raw
    Serial.print("\tP=");
    Serial.print(pitch);
    Serial.print("\tR=");
    Serial.print(roll);  
    Serial.print("\tY=");
    Serial.print(yaw);
  #endif

  #ifdef DEBUG_RC
    Serial.print("\tCH1=");
    Serial.print(CH1);
    Serial.print("\tCH2=");
    Serial.print(CH2);
    Serial.print("\tCH3=");
    Serial.print(CH3);
    Serial.print("\tCH4=");
    Serial.print(CH4);
  #endif
  #ifdef DEBUG_EQUILIBRIO
    Serial.print("\tAcel=");
    Serial.print(aceleracao);
    Serial.print("\tRoll=");
    Serial.print(roll);
    Serial.print("\tOff=");
    Serial.print(offset);
  #endif
  #ifdef DEBUG_PID
    Serial.print("\tP=");
    Serial.print(P);
    Serial.print("\tI=");
    Serial.print(I);
    Serial.print("\tD=");
    Serial.print(D);
  #endif
  #ifdef DEBUG_TIME
    Serial.print("\tT=");
    Serial.print((micros()/1000)- t);
  #endif
  #ifdef DEBUG
    Serial.println("\t");
  #endif
}
void ajustePID(){
  char caractere;  
  if(Serial.available() > 0) {
    // Lê byte da serial
    caractere = Serial.read();
    // Ignora caractere de quebra de linha
    switch(caractere){
      case 'p':
      P = P*0.9;
      break;
      case 'P':
      P = P*1.1;
      break;
      case 'i':
      I = I*0.9;
      break;
      case 'I':
      I = I*1.1;
      break;
      case 'd':
      D = D*0.9;
      break;
      case 'D':
      D = D*1.1;
      break;
    }
    pidRoll.SetTunings(P, I, D);
  }  
}
