///////////////////////////////// Include all the required libraries ////////////////////////////////
#include <Wire.h>
#include <PID_v1.h>
#include <My_Motors.h>
#include <Encoder.h>
///////////////////////////////// MPU-6050 parameters //////////////////////////////////////////////

long accelX, accelY, accelZ, gyroX, gyroY, gyroZ; // Parameters to record the raw accelerometer / gyro data
float omega_x_gyro, omega_x_calculated;// Parameter to store raw gyro data to converted data into [deg/s]
float mpu_calib[6] = {-2043, 108, 1293,  48, -12, 19}; // Array storing MPU Offset values [accX, accY,accZ,gyroX,gyroY,gyroZ]
float pitch, Theta_prev, Theta_now ; // Parameters for computing angle data from Accelerometer and gyro 
float Theta_correction = 2.5; // Angle to be added to the Theta_now for correcting the upright robot angle to account for total caliberation [deg]                 
double dt_gyro; // Variable to store time difference values for gyro angle calculations 
uint32_t t_gyro_prev, t_gyro_now; // timer for gyro unit
float alpha = 0.98; // Complimentary filter control parameter
float rad2deg = 57.3, deg2rad = 0.01745; // Angle conversion factors

////////////////////////////// MOTOR CONTROL PARAMATERS ////////////////////////////////////////////

short Rmot1 = 7; short Rmot2 = 8; // Pins for Right motor FW/BCK
short Rmot3 = 9; // Pin for Right motor PWM
short Lmot1 = 4; short Lmot2 = 5; // Pins for Left motor FW/BCK
short Lmot3 = 6; // Pins for Right motor PWM
short R_enc_pin1 = 2; short R_enc_pin2 = 3; // right motor encoder pins
short L_enc_pin1 = 18;  short L_enc_pin2 = 19; // left motor encoder pins 

float rpm_limit = 1.0; // RPM below this is considered 0, this value is in RPM and NOT [rad/s]
float avg_pt = 10.0;  // Number of points used for exponentially averaging the RPM signal
short PPR = 330; // Number of pulses per revolution of the encoder (for a gearbox 1:30, this value is 330 seen from the website)
float Final_Rpm_r, Final_Rpm_l; // Motor final averaged out RPM, units can be selected while calling get_RPM function
My_Motors Rmot(&Final_Rpm_r, rpm_limit, avg_pt, PPR); // Right motor object for calculating rotational velocities from encoder data
My_Motors Lmot(&Final_Rpm_l, rpm_limit, avg_pt, PPR); // Left motor object for calculating rotational velocities from encoder data
Encoder myEnc_r(R_enc_pin2, R_enc_pin1); // Make encoder objects to calculate motor velocties
Encoder myEnc_l(L_enc_pin1, L_enc_pin2); // Make encoder objects to calculate motor velocties

///////////////////////////////// Balancing PID parameters ///////////////////////////////////////////////////

double Input_bal, Output_bal, Setpoint_bal; // Input output and setpoint variables defined
double Out_min_bal = -255, Out_max_bal = 255; // PID Output limits, this is the output PWM value
double Kp_bal = 22.0, Ki_bal = 0.0, Kd_bal = 0.60; // Initializing the Proportional, integral and derivative gain constants
double Output_lower_bal = 30.0; // PWM Limit at which the motors actually start to move
PID bal_PID(&Input_bal, &Output_bal, &Setpoint_bal, Kp_bal, Ki_bal, Kd_bal, P_ON_E, DIRECT); // PID Controller for balancing

///////////////////////////////// TRANSLATION PID parameters ///////////////////////////////////////////////////

double Input_trans, Output_trans, Setpoint_trans; // Input output and setpoint variables defined
double Out_min_trans = -15, Out_max_trans = 15; // PID Output limits, this output is in degrees
double Kp_trans = 5.5, Ki_trans = 0.0, Kd_trans = 0.00; // Initializing the Proportional, integral and derivative gain constants
PID trans_PID(&Input_trans, &Output_trans, &Setpoint_trans, Kp_trans, Ki_trans, Kd_trans, P_ON_E, DIRECT); // PID Controller for translating
double Imax = 2.0; // Maximum limit upto which Iterm can rise 

///////////////////////////////// ROBOT PHYSICAL PROPERTIES ////////////////////////////////////////////

/*To correct for the difference between the speeds of the motors, We can use a PID controller here to make corrections, 
but that would unncessarily complicate things. A simple constant correction factor gives good results*/
float motor_cor_fac = 0.98; 
float r_whl = 0.5 * 0.130; // Wheel radius [m]
float l_cog = 0.01075; // Distance of the center of gravity of the upper body from the wheel axis [m] 
short fall_angle = 45; // Angles at which the motors must stop rotating [deg]
float full_speed = 350.0 * (2.0*3.14 / 60.0) * r_whl; // Full linear speed of the robot @ motor rated RPM [here 350 RPM @ 12 V] 
float frac = 0.30; // Factor for calculating fraction of the full linear speed
float speed_steps = 0.08; // Steps in which speed should be incremented in order to get to the full speed
float brake_steps = 0.02; // Steps in which speed should be decremented in order to apply brakes, the smaller the value, the longer the duration of brake application
String mode_prev = "balance", mode_now = "balance"; // To set different modes on the robot
bool moving_fwd_bck; // This is used for resetting the PID controllers Iterms and lastIinput terms when a stop command is sent if the robot is moving

////////////// LED BLINKING PARAMETERS/////////////////////////

long t_led_prev, t_led_now, dt_led; // Time parameters to log times for LED blinking
bool led_state = 0; // Parameter to turn LED from ON / OFF
int pin = 13; // PIN where LED is attached
int blink_rate = 100; // Blink after every [millis]
double t_loop_prev, t_loop_now, dt_loop; // Time parameters to log times for main control loop
double t_loop = 20.0; // Overall loop time [millis]

void setup() {

    Serial.begin(115200);  
    pinMode(pin, OUTPUT);
    
    /////////////////////////////// Motor initialization ///////////////////////////////////////////
  
    pinMode(Rmot1,OUTPUT);pinMode(Rmot2,OUTPUT);pinMode(Rmot3,OUTPUT); // Declaring right motor pins as output  
    pinMode(Lmot1,OUTPUT);pinMode(Lmot2,OUTPUT);pinMode(Lmot3,OUTPUT); // Declaring left motor pins as output
    
    ////////////////////////// BALANCING PID  initialization ////////////////////////////////////////////////////////
        
//  bal_PID.SetSampleTime(t_loop); // Set Loop time for PID [milliseconds]    
    bal_PID.SetMode(AUTOMATIC); // Set PID mode to Automatic
    bal_PID.SetOutputLimits(Out_min_bal, Out_max_bal); // Set upper and lower limits for the maximum output limits for PID loop
    
    ////////////////////////// TRANSLATION PID initialization ////////////////////////////////////////////////////////        

    trans_PID.SetSampleTime(t_loop); // Set Loop time for PID [milliseconds]
    trans_PID.SetMode(AUTOMATIC); // Set PID mode to Automatic        
    trans_PID.SetOutputLimits(Out_min_trans, Out_max_trans); // Set upper and lower limits for the maximum output limits for PID loop
  
    ////////////////////////// MPU initialization ///////////////////////////////////////////////////
    
    Wire.begin(); // Start wire library    
    setupMPU(); // Initializing MPU6050 
//  delay(5000);       
    get_MPU_data(); // Get initial angles of the MPU  
    pitch = (atan2(accelY-mpu_calib[1], accelZ+mpu_calib[2]))*rad2deg; //  Calculate initial pitch angle [deg] and caliberated for error in accY and accZ
    Theta_prev = pitch; // set the total starting angle to this pitch 
    t_gyro_prev = millis(); // Log time for gyro calculations [ms]  
    t_led_prev = millis(); // Log time for led blinking 
    t_loop_prev = millis(); // Log time for overall control loop [ms]
    delay(50);  
}

void loop() {

  t_loop_now = millis();
  dt_loop = t_loop_now - t_loop_prev; // Calculate time change since last loop [millis]
  /*Begin the main computing loop, enter the loop only if the minimum loop time is elapsed*/
  if (dt_loop>=t_loop){  
  
    read_BT(); // Read data from the bluetooth
    Get_Tilt_Angle(); // Update the angle readings to get updated omega_x_calculated, Theta_now
    Lmot.getRPM(myEnc_l.read() / 4.0, "rad/s"); // Get current encoder counts & compute left motor rotational velocity in [rad/s] 
    Rmot.getRPM(myEnc_r.read() / 4.0, "rad/s"); // Get current encoder counts & compute right motor rotational velocity in [rad/s]
    float V_whl = 0.5 * (Final_Rpm_r + Final_Rpm_l) * r_whl; // Linear translation velocity due to the 2 wheels spinning [m/s]
    float V_cog = omega_x_calculated * l_cog * deg2rad; // Linear translation velocity of the COG due to angular falling speed [m/s]
    float V_trans = V_whl;// Calculate the total Robot linear translation velocity [m/s]
    
    ////////////////////////////////////////// COMPUTE BALANCING PID OUTPUT/ //////////////////////////////////////////////////
    
    if (mode_now == "go fwd"){ // If we changed mode to forward now, start increasing the setpoint slowly to avoid jerky behaviour
      Setpoint_trans = Setpoint_trans + speed_steps;
      mode_prev = "go fwd";
      if (Setpoint_trans > frac * full_speed){Setpoint_trans = frac * full_speed;}
    }
    else if (mode_now == "go bck"){// If we changed mode to backward now, start decreasing the setpoint slowly to avoid jerky behaviour
      mode_prev = "go bck";
      Setpoint_trans = Setpoint_trans - speed_steps;
      if (Setpoint_trans < -frac * full_speed){Setpoint_trans = -frac * full_speed;}
      }
    else if (mode_now == "balance"){ // If we changed mode to balance now, we need to apply brakes
      if (mode_prev == "go fwd"){
        Setpoint_trans = Setpoint_trans - brake_steps; // If going in fwd direction, apply brakes by setting the trans setpoint to opposite value
        if (V_trans<=0.0){mode_prev = "balance";} // Set mode_prev to balance so that the robot goes to balancing mode totally
        }
      else if (mode_prev == "go bck"){
        Setpoint_trans = Setpoint_trans + brake_steps; // If going in bck direction, apply brakes by setting the trans setpoint to opposite value
        if (V_trans>=0.0){mode_prev = "balance";} // Set mode_prev to balance so that the robot goes to balancing mode totally
        }
      else if (mode_prev == "balance"){Setpoint_trans = 0.0;} 
      }
  
    Input_trans = V_trans; // Measured value / Input value
    trans_PID.Compute(); // Compute Output_trans of the 1st loop    

    Setpoint_bal = Output_trans; // Set the output [angle in deg] of the translation PID as Setpoint to the balancing PID loop
    Input_bal = Theta_now + Theta_correction; // Set Theta_now as the input / current value to the PID algorithm (The correction is added to correct for the error in MPU calculated angle)             
    double error_bal = Setpoint_bal - Input_bal; // To decide actuator / motor rotation direction      
    bal_PID.Compute_For_MPU(Kp_bal, Ki_bal, Kd_bal, omega_x_gyro);// Compute motor PWM using balancing PID 
//    bal_PID.Compute();  
    Output_bal = map(abs(Output_bal), 0, Out_max_bal, Output_lower_bal, Out_max_bal); // Map the computed output from Out_min to Outmax Output_lower_bal

    if (abs(error_bal)<0.2 && mode_now == "balance"){Output_bal = 0.0;} // To prevent continuous jerky behaviour, the robot starts balancing outside +- 0.2 deg
    if (abs(error_bal)>=fall_angle){ // If error_bal > fall_angle, this means robot has fallen down and we need to stop the motors
       Output_bal = 0.0; // Stop the robot
       trans_PID.Reset_Iterm(); // Now initialise the controller to make the sumintegral terms and lastinput terms to "0"
       bal_PID.Reset_Iterm();
       mode_now = "balance"; // Change mode to balance
       mode_prev = "balance"; // Change mode to balance
       }       
    mot_cont(error_bal, Output_bal); // Apply the calculated output to control the motor
    Blink_Led(); // Blink the LED
    t_loop_prev = t_loop_now; // Set prev loop time equal to current loop time for calculating dt for next loop        
  }  
}

///////////////////////// Function for initializing / getting MPU Data ////////////////////////////////////////////////////////

void Get_Tilt_Angle(){  
  t_gyro_now = millis(); // Log time now [millis]
  get_MPU_data(); // Update / Get Raw data acelX acelY acelZ giroX giroY giroZ  
  dt_gyro = (t_gyro_now - t_gyro_prev) / 1000.0; // calculate time difference since last loop for gyro angle calculations [seconds]
  omega_x_gyro = (gyroX-mpu_calib[3]) / 131.0; // Compute Angular velocity from raw gyroXreading [deg/s] and also correct for the error;    
  /*Since we will only need the ratios of accelerometer readings to calculate accelerometer angles, 
  we do not need to convert raw data to actual data */  
  pitch = (atan2(accelY-mpu_calib[1], accelZ+mpu_calib[2]))*rad2deg; // Angle calculated by accelerometer readings about X axis in [deg]  and caliberated for error in accY and accZ
  Theta_now = alpha * (Theta_prev + (omega_x_gyro * dt_gyro)) + (1-alpha) * pitch; // Calculate the total angle using a Complimentary filter
  omega_x_calculated = (Theta_now - Theta_prev) / dt_gyro; // Calculated omega_x from complimentary filter
  Theta_prev = Theta_now;
  t_gyro_prev = t_gyro_now;  
}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Accelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

///////////////////////// Function for getting MPU data ///////////////////////////////////////////////////////

void get_MPU_data(){

  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
//  while(Wire.available() < 6); // This line is commented out because Wire.requestFrom() is blocking, so there is no point in waiting again
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
//  while(Wire.available() < 6); // This line is commented out because Wire.requestFrom() is blocking, so there is no point in waiting again
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ

}

///////////////////////// Function for motor control ////////////////////////////////////////////////////////

void mot_cont(float e_rr, double Speed){
 
  if (e_rr>0){fwd_bot(Speed);}  
  else if (e_rr<0){back_bot(Speed);}
}

void back_bot(double Speed){
  digitalWrite(Lmot1, LOW);
  digitalWrite(Lmot2, HIGH);
  digitalWrite(Rmot1, LOW);
  digitalWrite(Rmot2, HIGH);  
  analogWrite(Lmot3,motor_cor_fac*Speed);    
  analogWrite(Rmot3,Speed);    
}

void fwd_bot(double Speed){
  digitalWrite(Lmot1, HIGH);
  digitalWrite(Lmot2, LOW);
  digitalWrite(Rmot1, HIGH);
  digitalWrite(Rmot2, LOW);  
  analogWrite(Lmot3,motor_cor_fac*Speed); 
  analogWrite(Rmot3,Speed);   
}

void stop_bot(){
  digitalWrite(Lmot1, LOW);
  digitalWrite(Lmot2, LOW);
  digitalWrite(Rmot1, LOW);
  digitalWrite(Rmot2, LOW);  
}

void rotate_bot(double Speed){
  digitalWrite(Lmot1, HIGH);
  digitalWrite(Lmot2, LOW);
  digitalWrite(Rmot1, LOW);
  digitalWrite(Rmot2, HIGH);  
  analogWrite(Lmot3,Speed); 
  analogWrite(Rmot3,Speed);   
}

///////////////////////////////// READ BLUETOOTH ////////////////////////

void read_BT(){
  if (Serial.available()>0){
    char c = Serial.read();
    if(c =='0'){mode_now = "go bck";Serial.print(mode_now);}
    else if (c =='1'){mode_now = "go fwd";Serial.print(mode_now);}
    else if(c=='2'){mode_now = "stop";Serial.print(mode_now);}
    else if (c =='3'){Kp_bal+=1.0;Serial.print("Kp_bal = "+String(Kp_bal));}
    else if(c=='4'){Kp_bal-= 1.0;Serial.print("Kp_bal = "+String(Kp_bal));}
    else if (c =='5'){Kd_bal+=0.1;Serial.print("Kd_bal = "+String(Kd_bal));}
    else if(c=='6'){Kd_bal-=0.1;Serial.print("Kd_bal = "+String(Kd_bal));}
    else if (c =='7'){Kp_trans+=1.0;Serial.print("Kp_trans = "+String(Kp_trans));}
    else if(c=='8'){Kp_trans-=1.0;Serial.print("Kp_trans = "+String(Kp_trans));}
    else if(c =='9'){mode_now = "balance";Serial.print(mode_now);}
   
    }  
}

///////////////////////////////////// NON BLOCKING FUNCTION TO BLINK LED ///////////////

void Blink_Led(){  
  t_led_now = millis();
  dt_led = t_led_now - t_led_prev;
  if (dt_led>blink_rate){
      if (led_state ==0){digitalWrite(pin, 1);led_state = 1;}
      else if(led_state == 1){digitalWrite(pin, 0);led_state = 0;}
      t_led_prev = t_led_now;   
      }
   }