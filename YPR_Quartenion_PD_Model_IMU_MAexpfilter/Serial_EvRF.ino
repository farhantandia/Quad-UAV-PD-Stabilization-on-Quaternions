void print_Serial()
{
//   Serial.flush();
//   Serial.print("IMU");
//   Serial.print(" ");
////Buat Kalibrasi IMU  
//   Serial.print(yaw);
//   Serial.print(" ");
//   Serial.print(pitch);
//   Serial.print(" ");
//   Serial.print(roll);
//   Serial.print("   ");
//   Serial.print(gx);
//   Serial.print(" ");
//   Serial.print(gy);
//   Serial.print(" ");
//   Serial.print(gz);
//   Serial.print(" ");
//   Serial.print(sampleFreq);
//   Serial.print("   ");
//   Serial.print(gyro[0]);
//   Serial.print(" ");
//   Serial.print(gyro[1]);
//   Serial.print(" ");
//   Serial.print(gyro[2]);
//   Serial.println("   ");  

//  Serial.flush();
// // Serial.print("IMU");
// // Serial.print("  ");
  Serial.print("yaw : ");
  Serial.print(int(yaw_val));
  
  Serial.print("  pitch : ");
  Serial.print(pitch1);
  
  Serial.print("  roll : ");
  Serial.print(roll1);
  Serial.println(" ");
//  Serial.print(",  ");
//   Serial.print(gyro[0]);
//   Serial.print(" ");
//   Serial.print(gyro[1]);
//   Serial.print(" ");
//   Serial.print(gyro[2]);
//   Serial.print(gy);
//   Serial.print(",");
//   Serial.print(gy);
//   Serial.print(",");
//   Serial.print(gz);
//   Serial.print(",");  
//  
//  Serial.print("        ");
//  Serial.print(tau_x);
//  Serial.print("  ");
//  Serial.print(tau_y);
//  Serial.print("  ");
//  Serial.print(tau_z);
// Serial.print("         ");
//  Serial.print(T1_N);
//  Serial.print("  ");
//  Serial.print(T2_N);
//  Serial.print("  ");
//  Serial.print(T3_N);
//  Serial.print("  ");
//  Serial.print(T4_N);
//  
//  Serial.print("         ");
//  Serial.print(T1_g);
//  Serial.print("  ");
//  Serial.print(T2_g);
//  Serial.print("  ");
//  Serial.print(T3_g);
//  Serial.print("  ");
//  Serial.print(T4_g);
  
//  Serial.print("         ");
//  Serial.print(x1_1);
//  Serial.print("  ");
//  Serial.print(x1_2);
//  Serial.print("  ");
//  Serial.print(x1_3);
//  Serial.print("  ");
//  Serial.print(x1_4);
//   
//   Serial.print("         ");
//  Serial.print(x2_1);
//  Serial.print("  ");
//  Serial.print(x2_2);
//  Serial.print("  ");
//  Serial.print(x2_3);
//  Serial.print("  ");
//  Serial.print(x2_4);
  
// 
//  Serial.print(",");
//  Serial.print(pwm_R_F);
//  Serial.print(",");
//  Serial.print(pwm_L_F );
//  Serial.print(",");
//  Serial.print(pwm_L_B);
//  Serial.print(",");
//  Serial.print(pwm_R_B);
//  Serial.print(" ");
//  
//  
//  Serial.print(INPUT_YAW); //yaw 1000-(1500)-2000
//  Serial.print(" ");
//  Serial.print(INPUT_THROTTLE);//throttle (1000)-1500-2000
//  Serial.print(" ");
//  Serial.print(INPUT_PITCH);//pitch 1000-(1500)-2000
//  Serial.print(" ");
//  Serial.print(INPUT_ROLL); //roll 1000-(1500)-2000
//  Serial.print(" ");
//  Serial.print(SWITCH_ESC); //switch SWB 1000 = 1, 2000 = 2
//  

//  Serial.print(alti);
//  Serial.print(" ");
//  Serial.print(temp);
//  Serial.print(" ");
//  Serial.print(pressure);
 
  serial_print.print("yaw : ");
  serial_print.print(yaw_val);
  
  serial_print.print("  pitch : ");
  serial_print.print(pitch);
  
  serial_print.print("  roll : ");
  serial_print.print(roll);
//   Serial.flush();
//  serial_print.print(time1);
//  serial_print.print(",");
//  serial_print.print(yaw_val);
//  serial_print.print(",");
//  serial_print.print(pitch);
//  serial_print.print(",");
//  serial_print.print(roll);
//  serial_print.print(",");
//   serial_print.print(gy);
//   serial_print.print(",");
//   serial_print.print(gx);
//   serial_print.print(",");
//   serial_print.print(gz);
//   serial_print.print(" ");  
//  serial_print.print(pwm_R_F);
//  serial_print.print(" ");
//  serial_print.print(pwm_L_F );
//  serial_print.print(" ");
//  serial_print.print(pwm_R_B);
//  serial_print.print(" ");
//  serial_print.print(pwm_L_B);
 serial_print.println(" ");

}

void Serial_EventRF(){

  if (serial_print.available() > 0)
  {
    char w = (char)serial_print.read();
  if(w =='0'){
      pwm_L_F = 1000;
     pwm_L_B = 1000;
     pwm_R_B = 1000;
     pwm_R_F = 1000;
      Serial.println("Motor OFF");
  }
    if(w =='1'){
      pwm_L_F = 1070;
     pwm_L_B = 1070;
     pwm_R_B = 1070;
     pwm_R_F = 1070;
      Serial.println("Motor ON");
    }
    if(w =='2'){
      pwm_L_F = 1200;
     pwm_L_B = 1200;
     pwm_R_B = 1200;
     pwm_R_F = 1200;
      serial_print.println("Motor ON");
    }
    
    if(w =='3'){
      kp_yaw = 0.5;
      kd_yaw = 0.39;
      //serial_print.println("KP 2.9 KD 1.6");
    }
    if(w =='d'){
      flag_compensation = 1;
      serial_print.println("Compensation ON");
    }
    if(w =='a'){
      flag_compensation = 0;
      serial_print.println("Compensation OFF");
     }
    if(w =='x'){
      flag_pid = 1;
      serial_print.println("PD Control ON");
      pwm_awal = 0.5687857;
      kp_yaw = 0.5;
      kd_yaw = 0.39;
     
      Serial.println("PD MOTOR ON");
    }
    if(w =='z'){
       flag_pid = 0;
       pwm_L_F = 1000;
       pwm_L_B = 1000;
       pwm_R_B = 1000;
       pwm_R_F = 1000;
      Serial.println("Motor OFF");
      Serial.println("PD Control OFF");
     }
     
    if(w =='v'){
      flag_RC = 1;
      serial_print.println("RC ON");
      
    }
    if(w =='c'){
      flag_RC = 0;
      serial_print.println("RC OFF");
     }
     
      if(w =='p'){
      serial_print.println(" ");
      serial_print.print("Proportional Constant+ ");
      kp_yaw += 0.01; //didapat untuk yaw kp 0.088 dan kd 0.044 dan tidak berlaku pada pitch dan roll
      serial_print.print(kp_yaw);
      serial_print.println(" ");
    }
    if(w =='o'){
      serial_print.println(" ");
      serial_print.print("Proportional Constant- ");
      kp_yaw -= 0.01;
      serial_print.print(kp_yaw);
      serial_print.println(" ");
    }
     if(w =='l'){
      serial_print.println(" ");
      serial_print.print("Derivative Constant+ ");
      kd_yaw += 0.01;
      serial_print.print(kd_yaw);
      serial_print.println(" ");
    }
    if(w =='k'){
      serial_print.println(" ");
      serial_print.print("Derivative Constant- ");
      kd_yaw -= 0.01;
      serial_print.print(kd_yaw);
      serial_print.println(" ");
      
    }
      if(w =='b'){
      yaw_rc=0;
    }
     if(w =='j'){
      yaw_rc=30;
   }
    if(w =='h'){
    yaw_rc=60;
//  kp_yaw = 0.3;
//  kd_yaw = 0.15;
//    }
    if(w =='m'){
      yaw_rc=15;
    }
    if(w =='n'){
     yaw_rc=-60;
    }
    if(w =='r'){
      kp_roll = 0;
      kd_roll = 0;
      kp_pitch = 0;
      kd_pitch = 0;
      kp_yaw = 0;
      kd_yaw = 0;
      serial_print.println("KP & KD = 0");
     }
         
  }
  }
}


void delay_print(int delaynya)
{
  if((millis()-last_delay) >= delaynya)
  {
    print_Serial();
    last_delay = millis();
  }
}
