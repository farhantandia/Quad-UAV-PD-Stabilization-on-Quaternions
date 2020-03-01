float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

// IMU algorithm update
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope-- quaternion derivative measured by gyro
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz); 
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

//    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void getypr()
{
  //ahrs to euler
 roll = -57.2957795131*atan2(2*(q0*q1 + q2*q3) , 1 - 2*(q1*q1 + q2*q2));
 pitch = -57.2957795131*asin(2*(q0*q2 - q3*q1));
 roll1 = abs(roll);
 pitch1 = abs(pitch);
 //yaw = -57.2957795131*atan2(2*(q0*q3 + q1*q2) , 1 - 2*(q2*q2 + q3*q3));
 
}

void mpu_init()
{
  mpu.initialize();
  mpu.setDLPFMode(MPU6050_DLPF_BW_188);
  mpu.setI2CBypassEnabled(01);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
}

void compass_init()
{
  compass.initialize();
  compass.setDataRate(0x06);
  compass.setMode(0x00);
  compass.getReadyStatus();
  compass.getLockStatus();
}

void getdata()
{
  mpu.getMotion6(&accel[0],&accel[1],&accel[2],&gyro[0],&gyro[1],&gyro[2]);

  gyro[0]+= 10 ;//GYRO_AVERAGE_OFFSET_X;
  gyro[1]+= 10;//GYRO_AVERAGE_OFFSET_Y;
  gyro[2]+= 2;//GYRO_AVERAGE_OFFSET_Z;

  gx=gyro[0]/65.536;
  gy=gyro[1]/65.536;
  gz=gyro[2]/65.536;
  
  ax=accel[0];//4096.0;
  ay=accel[1];//4096.0;
  az=accel[2];//4096.0;
  
  compass.getHeading(&magnet[0],&magnet[1],&magnet[2]);
  mx = magnet[0];
  my = magnet[1];
  mz = magnet[2];


//    heading pada sumbu y
//    yaw = atan2(mx, my);
//    float declinationAngle = (0.0 + (58.0 / 60.0)) / (180 / M_PI);
//    yaw += declinationAngle;
//    yaw_y = (-yaw) * 180/M_PI;
//    yaw_val=map(yaw_y,360,0,-180,180);
//    
    //heading pada sumbu x
    heading = atan2(my,mx);
    if(heading < 0)
    {
      heading += (2 * M_PI);
    }
   yaw = (heading*180/M_PI);
   
    yaw_val=map(yaw,360,0,-180,180);
    
  timer_baro = millis();
  baro.pollData(&temperature, &pressure, &alti);
  temp = (temperature/10);
}



