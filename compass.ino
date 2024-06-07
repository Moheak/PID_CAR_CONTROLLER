
/*
  Axis definition (differs from definition printed on the board!):
    X axis pointing forward (towards the short edge with the connector holes)
    Y axis pointing to the right I guess Left not right
    and Z axis pointing down.
    
  Positive yaw   : clockwise
  Positive roll  : right wing down
  Positive pitch : nose up
  
  Transformation order: first yaw then pitch then roll if needed for ealur angles.
*/





void gyro_setup(void) {
    Wire.beginTransmission(gyro_address);                        //Start communication with the GY-80.
    Wire.write(0x20);                                            //We want to write to register 20.
    Wire.write(0x0F);                                            //Set the register bits as 00001111 (Turn on the gyro and enable all axis).
    Wire.endTransmission();                                      //End the transmission with the gyro.
  
    Wire.beginTransmission(gyro_address);                        //Start communication with the GY-80.
    Wire.write(0x23);                                            //We want to write to register 23.
    Wire.write(0x10);                                            //500 dps Set the register bits as 00010000 (Block Data Update active).
    Wire.endTransmission();                                      //End the transmission with the gyro.
    delay(250);                                                  //Give the gyro time to start
  
  Wire.beginTransmission(ADXL345_address);                       // Start communicating with the device
  Wire.write(0x2D);                                              // Access/ talk to POWER_CTL Register - 0x2D
  // Enable measurement
  Wire.write(8);                                                 // Bit D3 High for measuring enable (8dec -> 0000 1000 binary)
  Wire.endTransmission(); 
  delay(10);

  Wire.beginTransmission(ADXL345_address);                       // Start communicating with the device
  Wire.write(0x31); //
  // Enable measurement
  Wire.write(0b00100010);                                        // setting sinsing range of accelerometer equals to: +- 8g  note that:  1 g = 9.8 m/s^2 (down gravity)
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(ADXL345_address);
  Wire.write(0x1E);
  Wire.write(-1);
  Wire.endTransmission();
  delay(10);
  //Y-axis
  Wire.beginTransmission(ADXL345_address);
  Wire.write(0x1F);
  Wire.write(1);
  Wire.endTransmission();
  delay(10);
  //Z-axis
  Wire.beginTransmission(ADXL345_address);
  Wire.write(0x20);
  Wire.write(3);
  Wire.endTransmission();
  delay(10);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This subroutine handles the calibration of the gyro. It stores the avarage gyro offset of 2000 readings.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_gyro(void) {

  cal_int = 0;                                                                        //Set the cal_int variable to zero.
  if (cal_int != 1000) {
    //Let's take multiple gyro data samples so we can determine the average gyro offset known as bias (calibration).
    // Serial.print("Starting calibration...");           //Print message
    gyro_signalen();
    delay(10);
    gyro_signalen();
    delay(10);
    gyro_signalen();
    delay(10);
    gyro_signalen();
    delay(10);
    gyro_signalen();
    delay(10);
    gyro_signalen();
    delay(10);
    gyro_signalen();
    delay(10);
    gyro_signalen();
    delay(10);
    for (cal_int = 0; cal_int < 1000 ; cal_int ++) {                                  // Take 2000 readings for calibration.
      //  if (cal_int % 25 == 0) digitalWrite(PB4, !digitalRead(PB4));                // Change the led status every 125 readings to indicate calibration.
      gyro_signalen();                                                                // Read the gyro output.
      gyro_roll_cal += gyro_roll;                                                     // Ad roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                                                   // Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                                       // Ad yaw value to gyro_yaw_cal.
     // if(cal_int%100 == 0) Serial.println(gyro_yaw_cal);                              // Print a dot every 100 readings  
      delay(3);                                                                       // Small delay to simulate a 250Hz loop during calibration.
    }
                                                                    //Set output PB3 low.
    // Now that we have 1000 measures, we need to devide by 1000 to get the average gyro offset.
    // Serial.println(" done!");
    gyro_roll_cal /= 1000;                                                            // Divide the roll total by 1000.
    gyro_pitch_cal /= 1000;                                                           // Divide the pitch total by 1000.
    gyro_yaw_cal /= 1000;                                                             // Divide the yaw total by 1000.
  }
}
void gyro_signalen(){
  Wire.beginTransmission(gyro_address);              // Start communication with the gyro (adress 1101001)
  Wire.write(168);                                   // Start reading @ register 28h and auto increment with every read
  Wire.endTransmission();                            // End the transmission
 Wire.requestFrom(gyro_address, 6);                  // Request 6 bytes from the gyro
 while(Wire.available() < 6);                        // Wait until the 6 bytes are received
 lowByte = Wire.read();                              // First received byte is the low part of the angular data
 highByte = Wire.read();                             // Second received byte is the high part of the angular data
 gyro_roll = ((highByte<<8)|lowByte);                // Multiply highByte by 256 and ad lowByte
 gyro_roll *= 1;  
 gyro_roll/= 57.14286;
 if(cal_int == 1000)gyro_roll -= gyro_roll_cal;      // Only compensate after the calibration

 lowByte = Wire.read();                              // First received byte is the low part of the angular data
 highByte = Wire.read();                             // Second received byte is the high part of the angular data
 gyro_pitch = ((highByte<<8)|lowByte);               // Multiply highByte by 256 and ad lowByte
 gyro_pitch *= 1;  
 
 //Invert axis
 gyro_pitch /= 57.14286;
 //gyro_pitch/=57.14286;
 if(cal_int == 1000) gyro_pitch -= gyro_pitch_cal;   // Only compensate after the calibration

 lowByte = Wire.read();                              // First received byte is the low part of the angular data
 highByte = Wire.read();                             // Second received byte is the high part of the angular data
 gyro_yaw = ((highByte<<8)|lowByte);                 // Multiply highByte by 256 and ad lowByte
 gyro_yaw *= 1;
 
 //Invert axis
 gyro_yaw /= 57.14286;
 if(cal_int == 1000) gyro_yaw -= gyro_yaw_cal;       // Only compensate after the calibration
 
  Wire.beginTransmission(ADXL345_address);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_address, 6, true);        // Read 6 registers of all accelerometers axis, each axis value is stored in 2 registers


  acc_x = (Wire.read()  | Wire.read()<< 8);          // Add the low and high byte to the acc_x variable.
  acc_x = acc_x / 65.0;                              // For a range of +-8g, we need to divide the raw values by 64, according to the Gy80 datasheet
  acc_y = (Wire.read()  | Wire.read()<< 8);          // Add the low and high byte to the acc_y variable.
  acc_y = acc_y / 65.0;                              // For a range of +-8g, we need to divide the raw values by 64, according to the datasheet
  acc_z = (Wire.read()  | Wire.read()<< 8);          // Add the low and high byte to the acc_z variable.
  acc_z = acc_z / 62.5;                              // For a range of +-8g, we need to divide the raw values by 64, according to the datasheet
}
String claculate_angles(void){
String Ac;
if ((millis() - lastTime) >= 20) { 
dt = (millis() - lastTime) / 1000.0;    //Time(s) delta t
lastTime = millis(); 
read_compass();
gyro_signalen(); 
                 
  compass_x_axis = (compass_A[0][0]*(compass_x_axis-compass_B[0]))+(compass_A[0][1]*(compass_y_axis-compass_B[1]))+(compass_A[0][2]*(compass_z_axis-compass_B[2])); 
  
  compass_y_axis = (compass_A[1][0]*(compass_x_axis-compass_B[0]))+(compass_A[1][1]*(compass_y_axis-compass_B[1]))+(compass_A[1][2]*(compass_z_axis-compass_B[2])); 
  
  compass_z_axis = (compass_A[2][0]*(compass_x_axis-compass_B[0]))+(compass_A[2][1]*(compass_y_axis-compass_B[1]))+(compass_A[2][2]*(compass_z_axis-compass_B[2])); 

  // The compass values change when the roll and pitch angle of the quadcopter changes. That's the reason that the x and y values need to calculated for a virtual horizontal position.
  // The 0.0174533 value is phi/180 as the functions are in radians in stead of degrees.
  // compass_x_horizontal = (float)compass_x * cos(angle_pitch * -0.0174533) + (float)compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533) - compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533);
 
  
//with accelemete we cam calculate roll and pithc because of graviti put we can not calculate yaw cuause it alwase surfed on earth
  roll_rad_acc = atan2(acc_y,sqrt(acc_x*acc_x+acc_z*acc_z));  // calculated angle in radians    opiset/side
  
  pitch_rad_acc = atan2(acc_x,sqrt(acc_y*acc_y+acc_z*acc_z)); // calculated angle in radians    opiset/side
 
  roll_deg_acc = 180*(atan2(acc_y,sqrt(acc_x*acc_x+acc_z*acc_z)))/PI; // calculated angle in degrees
  
  pitch_deg_acc = 180*(atan2(acc_x,sqrt(acc_y*acc_y+acc_z*acc_z)))/PI; // calculated angle in degrees
  
  // rollFnew_acc = 0.95*rollFold_acc+0.05*roll_deg_acc;
  
  // pitchFnew_acc = 0.95*pitchFold_acc+0.05*pitch_deg_acc;
  
  // rollFold_acc = rollFnew_acc;
  
  // pitchFold_acc = pitchFnew_acc;
 
  pitch = (pitch+gyro_pitch*dt)*0.9+pitch_deg_acc*0.1;
  
  roll = (roll+gyro_roll*dt)*0.9+roll_deg_acc*0.1;

  // thetaG = thetaG+gyro_pitch*dt;
  
  // rollG = rollG+gyro_roll*dt;
//search for it 
  compass_x_horizontal = (float)compass_x_axis * cos(pitch * 0.0174533) + (float)compass_y_axis * sin(roll * 0.0174533) * sin(pitch * 0.0174533) - compass_z_axis * cos(roll * 0.0174533) * sin(pitch * 0.0174533);
  compass_y_horizontal = (float)compass_y_axis * cos(roll * -0.0174533) + (float)compass_z_axis * sin(roll * -0.0174533);
  float declinationAngle = 0.0531;
  
  // float heading = atan2(y_axis, x_axis);
  float heading = atan2(compass_y_horizontal, compass_x_horizontal);


  heading += declinationAngle;

  // Correct for when signs are reversed. from(-180-180) to (0-360)
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  headingDegrees = heading * 180/PI; 
  // print_data();
 Ac=String(headingDegrees);

}
return Ac;
}
void setup_compass(void) {
  magSetting(0x00, B01101000);           // Magnetometer settings associated with Register A. See datasheet for acceptable values. 
  magSetting(0x01, B01100000);           // Magnetometer settings associated with Register B. See datasheet for acceptable values. 
}
void read_compass() {
   magSetting(0x02, 0x01);               // prepare to take reading (Single measurement mode) - this populates the registers with data
  Wire.requestFrom(compass_address, 6);  // Request 6 bytes. Each axis uses 2 bytes.
  if (Wire.available()>5){
     compass_x_axis = readValue()*1.52f;
     compass_x_axis *= 1; 
     compass_z_axis  = readValue()*1.52f;
     compass_z_axis *= 1;    
     compass_y_axis  = readValue()*1.52f;
     compass_y_axis *= 1;        
  } else {
  // Serial.println("****Error: Less than 6 bytes available for reading*****");
  }
}

void magSetting(byte regLoc, byte setting){
  Wire.beginTransmission(compass_address);
  Wire.write(regLoc); 
  Wire.write(setting);
  Wire.endTransmission();
  delay(10);
}

int readValue(){
  int val = Wire.read()<<8; 
      val |= Wire.read();
  return val;}


  void timeSync(unsigned long deltaT)
{
  unsigned long currTime = micros();
  long timeToDelay = deltaT - (currTime - timer);
  if (timeToDelay > 5000)
  {
    delay(timeToDelay / 1000);
    delayMicroseconds(timeToDelay % 1000);
  }
  else if (timeToDelay > 0)
  {
    delayMicroseconds(timeToDelay);
  }
  else
  {
      // timeToDelay is negative so we start immediately
  }
  timer = currTime + timeToDelay;
}