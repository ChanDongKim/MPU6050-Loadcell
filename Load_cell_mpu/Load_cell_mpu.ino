#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "HX711.h" //HX711로드셀 엠프 관련함수 호출
#include <SoftwareSerial.h>


SoftwareSerial bluetooth(7,8);

MPU6050 mpu;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
//int SensorPin = A0; //analog pin 0
const int MPU_addr=0x68;

#define calibration_factor -7050.0 // 로드셀 스케일 값 선언
#define DOUT  3 //엠프 데이터 아웃 핀 넘버 선언
#define CLK  4  //엠프 클락 핀 넘버 

#define BLUE 12 // RGB
#define GREEN 11 // RGB
#define RED 10 //RGB
int Color[7][3]= {
    {255,0,0}, //빨
    {255,50,0}, //주
    {255,255,0}, //노
    {0,255,0}, //초
    {0,150,255}, //파
    {0,0,255}, //남
    {100,0,255} //보
  };

HX711 scale(DOUT, CLK); //엠프 핀 선언 
float road = 0; // 로드셀 값 변수

#define DEBUG
#ifdef DEBUG
//#define DPRINT(args...)  Serial.print(args)             //OR use the following syntax:
#define DPRINTSTIMER(t)    for (static unsigned long SpamTimer; (unsigned long)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define  DPRINTSFN(StrSize,Name,...) {char S[StrSize];Serial.print("\t");Serial.print(Name);Serial.print(" "); Serial.print(dtostrf((float)__VA_ARGS__ ,S));}//StringSize,Name,Variable,Spaces,Percision
#define DPRINTLN(...)      Serial.println(__VA_ARGS__)
#else
#define DPRINTSTIMER(t)    if(false)
#define DPRINTSFN(...)     //blank line
#define DPRINTLN(...)      //blank line
#endif

//#define LED_PIN 13

// ================================================================
// ===                         Kalman Fliter                          ===
// ================================================================
double deg, dgy_y ;
double dt;
double val;
uint32_t pasttime ;

class kalman {
  public :
    double getkalman(double acc, double gyro, double dt) {
      //project the state ahead
      angle += dt * (gyro - bias) ;

      //Project the error covariance ahead
      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle) ;
      P[0][1] -= dt * P[1][1] ;
      P[1][0] -= dt * P[1][1] ;
      P[1][1] += Q_gyro * dt ;

      //Compute the Kalman gain
      double S = P[0][0] + R_measure ;
      K[0] = P[0][0] / S ;
      K[1] = P[1][0] / S ;

      //Update estimate with measurement z
      double y = acc - angle ;
      angle += K[0] * y ;
      bias += K[1] * y ;

      //Update the error covariance
      double P_temp[2] = {P[0][0], P[0][1]} ;
      P[0][0] -= K[0] * P_temp[0] ;
      P[0][1] -= K[0] * P_temp[1] ;
      P[1][0] -= K[1] * P_temp[0] ;
      P[1][1] -= K[1] * P_temp[1] ;

      return angle ;
    } ;
    
    void init(double angle, double gyro, double measure) {
      Q_angle = angle ;
      Q_gyro = gyro ;
      R_measure = measure ;

      angle = 0 ;
      bias = 0 ;

      P[0][0] = 0 ;
      P[0][1] = 0 ;
      P[1][0] = 0 ;
      P[1][1] = 0 ;
    } ;

    double getvar(int num) {
      switch (num) {
        case 0 :
          return Q_angle ;
          break ;
        case 1 :
          return Q_gyro ;
          break ;
        case 2 :
          return R_measure ;
          break ;
      }
    } ;

  private :
    double Q_angle, Q_gyro, R_measure ;
    double angle, bias ;
    double P[2][2], K[2] ;
} ;

kalman kal ;

// supply your own gyro offsets here, scaled for min sensitivity use MPU6050_calibration.ino
// -4232  -706  1729  173 -94 37
//                       XA   YA    ZA    XG  YG   ZG
int MPUOffsets[6] = {  -3115, 400, 1033,  119, 5, -23};


// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
int FifoAlive = 0; // tests if the interrupt is triggering
int IsAlive = -20;     // counts interrupt start at -20 to get 20+ good values before assuming connected
// MPU control/status vars
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
byte StartUP = 100; // lets get 100 readings from the MPU before we start trusting them (Bot is not trying to balance at this point it is just starting up.)

void MPU6050Connect() {
  static int MPUInitCntr = 0;
  // initialize device
  mpu.initialize(); // same
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();// same

  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));

    if (MPUInitCntr >= 10) return; //only try 10 times
    delay(10);
    MPU6050Connect(); // Lets try again
    return;
  }
  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);

  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  // enable Arduino interrupt detection
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  Serial.print("mpu.getInterruptDrive=  "); Serial.println(mpu.getInterruptDrive());
  attachInterrupt(0, dmpDataReady, RISING); //pin 2 on the Uno
  mpuIntStatus = mpu.getIntStatus(); // Same
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  delay(1000); // Let it Stabalize
  mpu.resetFIFO(); // Clear fifo buffer
  mpu.getIntStatus();
  mpuInterrupt = false; // wait for next interrupt
}

// ================================================================
// ===                    MPU DMP Get Data                      ===
// ================================================================
void GetDMP() { // Best version I have made so far
  // Serial.println(F("FIFO interrupt at:"));
  // Serial.println(micros());
  static unsigned long LastGoodPacketTime;
  mpuInterrupt = false;
  FifoAlive = 1;
  fifoCount = mpu.getFIFOCount();
  if ((!fifoCount) || (fifoCount % packetSize)) { // we have failed Reset and wait till next time!
//    digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
    mpu.resetFIFO();// clear the buffer and start over
  } else {
    while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
      mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
      fifoCount -= packetSize;
    }
    LastGoodPacketTime = millis();
    MPUMath(); // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
//    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
  }
}

// ================================================================
// ===                        LED COLOR                         ===
// ================================================================
void ColorChoice(int a) {
  analogWrite(RED, Color[a][0]);
  analogWrite(GREEN, Color[a][1]);
  analogWrite(BLUE, Color[a][2]);
}


// ================================================================
// ===                        MPU Math                          ===
// ================================================================
float Yaw, Pitch, Roll;
void MPUMath() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  Yaw = (ypr[0] * 180.0 / M_PI) + 180; //x deg
  Pitch = (ypr[1] *  180.0 / M_PI) + 180; //y deg
  Roll = (ypr[2] *  180.0 / M_PI) + 180; //z deg
  DPRINTSTIMER(10) {
  DPRINTSFN(15, "\tX:", Yaw, 6, 1); // Print
  //DPRINTSFN(15, "\tY:", Pitch, 6, 1); // how to use DPRINTSFN() 15: USING 15 CHARACTERS MAX for the number to string conversion, "\tPitch": displayed name, Pitch: Floating point value to display, 6: using 6 characters in the conversion, 1: trim the float to 1 decimal place
  //DPRINTSFN(15, "\tRoll:", Roll, 6, 1); //DPRINTSFN() Will display any floating point number with simple and space everything nicely
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  deg = atan2(AcX, AcZ) * 180 / PI ;  //acc data to degree data
  dgy_y = GyY / 131. ;  //gyro output to
  
  dt = (double)(micros() - pasttime) / 1000000;
  pasttime = micros();  //convert output to understandable data
  
  val = kal.getkalman(deg, dgy_y, dt) ;  //get kalman data

  road = scale.get_units();
  Serial.print("\troad: "); Serial.print(road);//무게 시리얼 모니터 출력 
  Serial.print("\tkal: "); Serial.print(val);// 칼만필터값 시리얼 모니터 출력
  if(road < 5){
    Serial.print("0");
    ColorChoice(0); // 빨간색 출력
  } else {
    int ColorMap = map(road, 5, 25, 1,6); //로드셀의 값에 따른 led 색과 매핑
    ColorChoice(ColorMap); //주황색~보라색 출력
  }
    DPRINTLN(); // Simple New Line
  }
}

// ================================================================
// ===                         Setup                            ===
// ================================================================
void setup() {
  Serial.begin(115200);
  bluetooth.begin(115200);
  while (!Serial);
  Serial.println("i2cSetup");
  i2cSetup();
  Serial.println("MPU6050Connect");
  MPU6050Connect();
  Serial.println("Setup complete");
//  pinMode(LED_PIN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  kal.init(0.001, 0.003, 0.03) ;  //init kalman filter
  
  scale.set_scale(calibration_factor);  //스케일 지정 
  scale.tare();  //스케일 설정
}
// ================================================================
// ===                          Loop                            ===
// ================================================================
void loop() {
 
  String strpmp="";
  int val1 = Yaw;
  int val2 = val;
  int val3 = road;
  
  if (mpuInterrupt ) { // wait for MPU interrupt or extra packet(s) available
    GetDMP(); 

    strpmp += val1;
    strpmp += ",";
    strpmp += val2;
    strpmp += ",";
    strpmp += val3;

    bluetooth.print(strpmp);
   
  }
}
