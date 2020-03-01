#include <MPU6050.h>
#include <HMC5883L.h>
#include <BMP085NB.h>
#include <SPI.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <Servo.h>
#include <MovingAverage.h>
#include <RXInterrupt.h>

MPU6050 mpu;
HMC5883L compass;
BMP085NB baro;

//---------------------------------------------------------------------------------------------------
// Definitions
MovingAverage average1(0.1); //range 0-1 ,semakin kecil alpa, noise semakin kecil
MovingAverage average2(0.1);
MovingAverage average3(0.1);

#define sampleFreq 5000.0f    // sample frequency in Hz
#define PI 3.14159265
#define serial_print Serial1
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

//---------------------------------------------------------------------------------------------------
// Variable definitions
bool flag_pid,flag_RC,flag_compensation,flag_inc_pwm ;

volatile float beta = 10;                // 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame
int16_t magnet[3], accel[3], gyro[3];
int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
float yaw, pitch, roll, alti, temp,exp_yaw,exp_pitch,exp_roll;
float yaw_val,yaw_y;
unsigned long t;
unsigned long last_delay = 0;
int temperature;
long pressure;
float calibrated_values[3],uncalibrated_values[3];
uint32_t lastTime = 0 , T = 20, timer_baro;
float heading;

float values_from_magnetometer[3];
double Input, Output, Setpoint, dt, timer_f;
float magneto;

float time;
int roll1,pitch1,yaw1;
int pwm_roll=0;

double u = 0 ;
float kp_roll = 0;
float kd_roll = 0;
float kp_pitch = 0;
float kd_pitch = 0;
float kp_yaw = 0;
float kd_yaw = 0;
int esc1 = 1000;
int esc2 = 1000;
int esc3 = 1000;
int esc4 = 1000;
Servo L_F_prop;
Servo L_B_prop;
Servo R_F_prop;
Servo R_B_prop;
float  pwm_L_F, pwm_L_B,pwm_R_B,pwm_R_F;
unsigned long time1;
int ch1,ch2,ch3,ch4,ch5,ch6;
float Derivative_yaw = 0 ; 
float Derivative_pitch = 0 ;  
float Derivative_roll = 0;
//---------------------------------------------------------------------------------------------------

 short values[5];
 int INPUT_YAW,INPUT_THROTTLE,INPUT_PITCH,INPUT_ROLL;
 bool SWITCH_ESC;
 float roll_rc = 0;
 float pitch_rc = 0;
 float yaw_rc = 0;
 float sp_roll,sp_pitch,sp_yaw;
void setup() 
{
  
  Serial.begin(115200);
  serial_print.begin(57600);
  Wire.begin();
  
  //Set pin for RC
  pinMode(20, INPUT); // Set our input pins as such
  pinMode(21, INPUT);
  pinMode(22, INPUT);
  pinMode(23, INPUT);
  pinMode(4, INPUT);
  pinMode(3, INPUT);
  
  //Set pin for Motor for roll
  L_F_prop.attach(10);
  L_B_prop.attach(9);  
  R_F_prop.attach(5);
  R_B_prop.attach(4);  
//  

  
  time = millis(); //Start counting time in milliseconds
  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs won't start up or enter in the configure mode.
   * The min value is 1000us and max is 2000us, REMEMBER!*/
  R_F_prop.writeMicroseconds(esc1); 
  R_B_prop.writeMicroseconds(esc2);
  L_F_prop.writeMicroseconds(esc3); 
  L_B_prop.writeMicroseconds(esc4);
  
  int pins[5] = {20, 21, 22, 23, 3};
  initChannels(pins, 5);
  
  mpu_init();
  compass_init();
  baro.initialize();
  baro.setSeaLevelPressure(101000);
  flag_pid = false;
  flag_RC = false;
  flag_compensation = false;
  flag_inc_pwm = false;
  average1.reset(Derivative_yaw); 
  average2.reset(Derivative_pitch) ;  
  average3.reset(Derivative_roll);
  //delay(7000); //delay motor untuk strat up 
  delay(1500);
}

void loop() 
{
  time1=millis();

  getdata();
  MadgwickAHRSupdateIMU(gx, gy, gz, ay, ax, az);
  getypr();
////////////////////////////////////////////////  
  if(flag_compensation == true){
  compensation();
  }
  else if(flag_pid == false){
  u=0;
  }
/////////////////////////////////////////////////  
  if(flag_pid == true){
  moving_avg_filter();
  PD_control();

  }
  
//////////////////////////////////////////////////
//uncomment apabila tidak menggunakan RC
//  if(flag_RC == true){
//     RC_mode();
//     
//  }
//  if((flag_RC == false)||(values[4]<1100)){
//  R_F_prop.writeMicroseconds(esc1); 
//  R_B_prop.writeMicroseconds(esc2);
//  L_F_prop.writeMicroseconds(esc3); 
//  L_B_prop.writeMicroseconds(esc4);  
//  }
/////////////////////////////////////////////////    
     
//
//   if(roll1 > 90)
//      {
//      flag_pid = false;
//      flag_RC = false;
//       }
//      if(pitch1 >90)
//      {
//      flag_RC = false  ;
//      flag_pid = false;
//      }  
      
  R_F_prop.writeMicroseconds(pwm_R_F); 
  R_B_prop.writeMicroseconds(pwm_R_B);
  L_F_prop.writeMicroseconds(pwm_L_F); 
  L_B_prop.writeMicroseconds(pwm_L_B);
  
 // Serial_EventRF();
 // Serial_EventArd();
   
  delay_print(50);
}
