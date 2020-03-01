void RC_mode(){
 
updateChannels(values, 5);  

INPUT_YAW = values[0];
INPUT_THROTTLE = map(values[1],1000,2000,0,500);
INPUT_PITCH = values[2];
INPUT_ROLL = values[3];

roll_rc = map(INPUT_ROLL,1000,2000,-10,10);
pitch_rc = map(INPUT_PITCH,1000,2000,-10,10);
yaw_rc = map(INPUT_YAW,1000,2000,-10,10);
if (values[4]>1900){
      flag_pid = 1;  
      pwm_awal = 0.5687857;
      kp_roll = 2.9;
      kd_roll = 1.7;
      kp_pitch = 0;
      kd_pitch = 0;
      kp_yaw = 0;
      kd_yaw = 0;
}

//if ((values[0]>1900) && (values[2]<1100) && (values[3]<1100)){
//      
//  
//      delay(1000);
//}
}

