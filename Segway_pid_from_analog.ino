#include <Wire.h>
// mpu address
#define MPU6050 0x68
// power register
#define MPU6050_POWER 0x6B
// accelerometr register
#define MPU6050_ACCEL_XOUT_H 0x3B
int p=0, h=0, s=0;;
void setRegister(int address, byte reg, byte data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
  delay(5);
}

byte getRegisters(int address, byte reg, byte * buffer, byte length) {
  byte count = 0;
  
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.beginTransmission(address);
  Wire.requestFrom(address, (int)length);
  while(Wire.available()) {
    buffer[count] = Wire.read();  
    count++;
  }
  Wire.endTransmission();
  
  return count;
}

// mpu6050 data
byte buffer [14];
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;
float angle_x = 0, angle_y = 0; //angle_z = 0;

// time
int dt = 10;

// engines
#define E1_DIR 8
#define E1_PWM 10
float E1_POWER = 0;

#define E2_DIR 9
#define E2_PWM 11
float E2_POWER = 0;

//ANALOG JOYSTICK
const int SW_pin = 2; // digital pin connected to switch output
const int X_pin = 0; // analog pin connected to X output
const int Y_pin = 1; // analog pin connected to Y output

void setup() {
  //ANALOG JOYSTICK
  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);
  
  Wire.begin();
  Serial.begin(9600);

  // mpu6050 init and calibrate
  Serial.println("* calibrate mpu6050");
  setRegister(MPU6050, MPU6050_POWER, 0); delay(50); // wake up mpu6050
 //calibrateMpu6050(); delay(50);

  Serial.println("* motor settings");
  // set engines pins
  pinMode(E1_DIR, OUTPUT);
  pinMode(E1_PWM, OUTPUT);
  pinMode(E2_DIR, OUTPUT);
  pinMode(E2_PWM, OUTPUT);

  setEngine(1, 0); // id {1, 2}, pwm <-255, 255>
  setEngine(2, 0); // id {1, 2}, pwm <-255, 255>
}

void setEngine(int id, int pwm) {
  // "left" engine
  bool dir = 1;
  if(pwm < 0) {
    dir = 0;
    pwm *= -1;
  }
  
  if(id == 1) {
    digitalWrite(E1_DIR, dir);
    analogWrite(E1_PWM, pwm);
  }
  else if (id == 2) {
    digitalWrite(E2_DIR, dir);
    analogWrite(E2_PWM, pwm);
  }
}

void calibrateMpu6050() {
    int samples = 100;
    // gyro
    float tmpx = 0, tmpy = 0, tmpz = 0;
    for(int i=0; i<samples; ) {
      byte readed = getRegisters(MPU6050, MPU6050_ACCEL_XOUT_H, buffer, 14);
      if(readed == 14) {
          ++i;
          int gyro_x = word(buffer[8], buffer[9]);
          int gyro_y = word(buffer[10], buffer[11]);
          int gyro_z = word(buffer[12], buffer[13]);
          tmpx += gyro_x;
          tmpy += gyro_y;
          tmpz += gyro_z;
      }
      else {
          Serial.println("error");
      }
      delay(10);
    }
    gyro_x_offset = tmpx / samples;
    gyro_y_offset = tmpy / samples;
    gyro_z_offset = tmpz / samples;
    // accelerometer
    float accel_x = (int)word(buffer[0], buffer[1]);
    float accel_y = (int)word(buffer[2], buffer[3]);
    float accel_z = (int)word(buffer[4], buffer[5]);
    accel_x = mapf(accel_x, -32768, 32767, -2, 2);
    accel_y = mapf(accel_y, -32768, 32767, -2, 2);
    accel_z = mapf(accel_z, -32768, 32767, -2, 2);
    // set starting angles
    angle_x = atan2(accel_y, accel_z) * 180 / PI;
    angle_y = atan2(accel_x, accel_z) * 180 / PI;
}

void getAngles() {
  byte readed = getRegisters(MPU6050, MPU6050_ACCEL_XOUT_H, buffer, 14);

  if(readed == 14)
  {
    // gyro
    float gyro_x = (int)word(buffer[8], buffer[9]);
    float gyro_y = (int)word(buffer[10], buffer[11]);
    //Serial.print(mapf(gyro_x, -32768, 32767, -250, 250)); Serial.print("\t");
    //Serial.print(mapf(gyro_y, -32768, 32767, -250, 250)); Serial.println();
    gyro_x = mapf(gyro_x - gyro_x_offset, -32768, 32767, -250, 250);
    gyro_y = mapf(gyro_y - gyro_y_offset, -32768, 32767, -250, 250);
    
    // accelerometer
    float accel_x = (int)word(buffer[0], buffer[1]);
    float accel_y = (int)word(buffer[2], buffer[3]);
    float accel_z = (int)word(buffer[4], buffer[5]);
    accel_x = mapf(accel_x, -32768, 32767, -2, 2);
    accel_y = mapf(accel_y, -32768, 32767, -2, 2);
    //Serial.print(accel_y); Serial.println();
    accel_z = mapf(accel_z, -32768, 32767, -2, 2);
    float accel_x_angle = atan2(accel_y, accel_z) * 180 / PI;
    float accel_y_angle = atan2(accel_x, accel_z) * 180 / PI;
    // complementary filter
    angle_x = 0.98 * (angle_x + gyro_x * dt / 1000.0) + 0.02 * accel_x_angle;
    angle_y = 0.98 * (angle_y - gyro_y * dt / 1000.0) + 0.02 * accel_y_angle;
    //Serial.print(angle_y); Serial.println();
  }
}

// PID
float goalAngle = 0;
float prev_diff = 0;
float Kp = 60.0, Ki = 0.5, Kd = 0; // pocz. Kp=10, Ki=0.05, Kd=0 // next 14, 0.14
float I = 0;

float calculatePid(float goalAngleAnalog) {
  float diff = goalAngleAnalog - angle_y;
  float P = diff;
  I += diff;
  float D = diff - prev_diff;
  prev_diff = diff;

  float pid = P * Kp + I * Ki + D * Kd;
  return pid;
}

//teraz Kp i Ki jest pobierane z dzojstika
float calculatePidJoystick(float goalAngleAnalog,float _Kp, float _Ki) {
  //Serial.print(angle_y); Serial.println();
  float diff = goalAngleAnalog - angle_y;
  float P = diff;
  I += diff;
  float D = diff - prev_diff;
  prev_diff = diff;

  float pid = P * _Kp + I * _Ki + D * Kd;
  return pid;
}
//JOYSTICK ANALOG
int xRead;
int yRead;
float x=0.00001;
float y=0;

void loop() {
//  JOYSTICK ANALOG

  xRead=analogRead(X_pin);
  yRead=analogRead(Y_pin);
  //x=map(xRead,0,1023,-5,5);
  y=map(yRead,1023,0,5,-5)-1;
  Serial.print("x: "); Serial.print(xRead); Serial.print("\t");
  Serial.print("y: "); Serial.print(y); Serial.println();
  float xTemp=0.9*x + map(xRead,0,1023,5,-5)*0.1;
  x=xTemp;
  
  //Serial.println(x); 
 

  getAngles();

  //Serial.println(angle_y);
  if(angle_y > 35 || angle_y < -35)
  {
    setEngine(1, 0); // id {1, 2}, pwm <-255, 255>
    setEngine(2, 0); // id {1, 2}, pwm <-255, 255>
    return;
  }

  // pid
  float pid = calculatePid(x);
  pid *= -1;
 
  // engines
  int maxPower = 255;
  E1_POWER = pid;
  if(E1_POWER > maxPower) E1_POWER = maxPower;
  else if (E1_POWER < -maxPower) E1_POWER = -maxPower;
  
 
  if(y<0){      // LEFT
    E2_POWER = pid * 1.4;
    E1_POWER = pid * 0.6;
  }    
  else if (y>0){ // RIGHT
    E2_POWER = pid * 0.6;
    E1_POWER = pid * 1.4;
  }
  else
    E2_POWER = E1_POWER;  // the same power for both engines


  setEngine(1, E1_POWER); // id {1, 2}, pwm <-255, 255>
  setEngine(2, E2_POWER); // id {1, 2}, pwm <-255, 255>
  

  delay(dt);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
