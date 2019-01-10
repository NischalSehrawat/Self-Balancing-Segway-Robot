///////////////////////////////// Include all the required libraries ////////////////////////////////
#include <Wire.h>
#include <PID_v1.h>
#include <My_Motors.h>
#include <Encoder.h>
///////////////////////////////// MPU-6050 parameters //////////////////////////////////////////////

long accelX, accelY, accelZ, gyroX, gyroY, gyroZ; // They record the raw accelerometer / gyro data
float omega_x;// they record the converted data into [deg/s]
//float A[6] = {-2043, 108, 1293,  48, -12, 19}; // To compensate for the error_now in MPU  
float A[3] = {0.0,0.0,0.0}; // Array containing MPU Offset data accY, accZ, giroX  
double pitch, Theta_prev, Theta_now ; // For storing Angle data from Accelerometer                   
double dt_gyro; // Variable to store time difference values for gyro angle calculations 
uint32_t t_gyro_prev, t_gyro_now; // timer for gyro unit
float alpha = 0.98; // Complimentary filter control parameter
float rad2deg = 57.3, deg2rad = 0.01745; // Angle conversion factors

////////////////////////////// MOTOR CONTROL PARAMATERS ////////////////////////////////////////////

int rmot1 = 7; int rmot2 = 8; // Pins for Right motor FW/BCK
int rmot3 = 9; // Pin for Right motor PWM
int lmot1 = 4; int lmot2 = 5; // Pins for Left motor FW/BCK
int lmot3 = 6; // Pins for Right motor PWM
short enc_pin_r1 = 2;short enc_pin_r2 = 3; // right motor encoder pins
short enc_pin_l1 = 18;short enc_pin_l2 = 19; // left motor encoder pins 

float rpm_limit = 0.0; // RPM below this is considered 0
float avg_pt = 10.0;  // Number of points used for averaging the RPM signal
short PPR = 990; // Number of pulses per revolution of the wheel
float Final_Rpm_r, Final_Rpm_l; // Motor final averaged out RPM, units can be selected while calling get_RPM function
My_Motors rmot(&Final_Rpm_r, rpm_limit, avg_pt, PPR); // Right motor object for calculating rotational velocities from encoder data
My_Motors lmot(&Final_Rpm_l, rpm_limit, avg_pt, PPR); // Left motor object for calculating rotational velocities from encoder data
Encoder myEnc_r(enc_pin_r1, enc_pin_r2); // Make encoder objects to calculate motor velocties
Encoder myEnc_l(enc_pin_l2, enc_pin_l1); // Make encoder objects to calculate motor velocties

///////////////////////////////// Balancing PID parameters ///////////////////////////////////////////////////

double Input_bal, Output_bal, Setpoint_bal; // Input output and setpoint variables defined
double Out_min_bal = -255, Out_max_bal = 255; // PID Output limits, this is the output PWM value
double Kp_bal = 72.0, Ki_bal = 0.0, Kd_bal = 1.4; // Initializing the Proportional, integral and derivative gain constants
double Output_lower_bal = 30.0; // PWM Limit at which the motors actually start to move
PID bal_PID(&Input_bal, &Output_bal, &Setpoint_bal, Kp_bal, Ki_bal, Kd_bal, P_ON_E, DIRECT); // Create a balancing PID instance

///////////////////////////////// TRANSLATION PID parameters ///////////////////////////////////////////////////

double Input_trans, Output_trans, Setpoint_trans; // Input output and setpoint variables defined
double Out_min_trans = -5, Out_max_trans = 5; // PID Output limits, this output is in degrees
double Kp_trans, Ki_trans, Kd_trans; // Initializing the Proportional, integral and derivative gain constants
PID trans_PID(&Input_trans, &Output_trans, &Setpoint_trans, Kp_trans, Ki_trans, Kd_trans, P_ON_E, DIRECT); // Create a balancing PID instance

///////////////////////////////// ROBOT PHYSICAL PROPERTIES ////////////////////////////////////////////

float r_whl = 0.5 * 0.085; // Wheel radius [m]
float l_cog = 0.01075; // Distance of from the wheel axis [m] 
short fall_angle = 45; // Angles at which the motors must stop rotating [deg]

////////////// LED BLINKING PARAMETERS/////////////////////////

long t_led_prev, t_led_now, dt_led; // Time parameters to log times for LED blinking
bool led_state = 0; // Parameter to turn LED from ON / OFF
int pin = 13; // PIN where LED is attached
int blink_rate = 100; // Blink after every [millis]
double t_loop_prev, t_loop_now, dt_loop; // Time parameters to log times for main control loop
double t_loop = 5; // Overall loop time [millis]

void setup() {

    Serial.begin(115200);  
    pinMode(pin, OUTPUT);
    
    /////////////////////////////// Motor initialization ///////////////////////////////////////////
  
    pinMode(rmot1,OUTPUT);pinMode(rmot2,OUTPUT);pinMode(rmot3,OUTPUT); // Declaring right motor pins as output  
    pinMode(lmot1,OUTPUT);pinMode(lmot2,OUTPUT);pinMode(lmot3,OUTPUT); // Declaring left motor pins as output  
  
    ////////////////////////// BALANCING PID  initialization ////////////////////////////////////////////////////////
        
//    bal_PID.SetSampleTime(t_loop); // Set Loop time for PID [milliseconds]
    
    bal_PID.SetMode(AUTOMATIC); // Set PID mode to Automatic    
//    bal_PID.SetTunings(Kp, Ki, Kd);    
    bal_PID.SetOutputLimits(Out_min_bal, Out_max_bal); // Set upper and lower limits for the maximum output limits for PID loop    
	
	////////////////////////// TRANSLATION PID  initialization ////////////////////////////////////////////////////////        
    
    trans_PID.SetMode(AUTOMATIC); // Set PID mode to Automatic        
    trans_PID.SetOutputLimits(Out_min_trans, Out_max_trans); // Set upper and lower limits for the maximum output limits for PID loop
  
    ////////////////////////// MPU initialization ///////////////////////////////////////////////////
    
    Wire.begin(); // Start wire library    
    setupMPU(); // Initializing MPU6050  
    get_MPU_data(); // Get initial angles of the MPU  
    delay(100);    
    pitch = (atan2(accelY - A[1], accelZ + A[2]))*rad2deg; //  Calculate initial pitch angle [deg]    
    Theta_prev = pitch;     // set the total starting angle to this pitch 
//  delay(5000);      
    t_gyro_prev = millis(); // Log time for gyro calculations [ms]  
    t_led_prev = millis(); // Log time for led blinking 
    t_loop_prev = millis(); // Log time for overall control loop [ms]
    delay(50);  
}


void loop() {

  t_loop_now = millis();
  dt_loop = t_loop_now - t_loop_prev;

  if (dt_loop>=t_loop){
  
//	read_BT(); // Read data from the serial bluetooth
    get_tilt_angle(); // Update the angle readings to get updated omega_x, Theta_now
    lmot.getRPM(myEnc_l.read() / 4.0, "rad/s"); // Compute left motor rotational velocity in [rad/s] 
    rmot.getRPM(myEnc_r.read() / 4.0, "rad/s"); // Compute right motor rotational velocity in [rad/s] 
  
  ////////////////// COMPUTE TRANSLATION PID OUTPUT///////////////////////////////////////////////////////
  
    Input_trans = 0.5 * (Final_Rpm_r + Final_Rpm_l) * r_whl + omega_x * l_cog * deg2rad; // Calculate Robot linear translation velocity [m/s]
    Kp_trans = float((1.0 / 1023.0) *analogRead(A0));
    Ki_trans = float((1.0 / 1023.0) *analogRead(A2));
    Kd_trans = float((1.0 / 1023.0) *analogRead(A1));  
    trans_PID.Compute_With_Actual_LoopTime(Kp_trans, Ki_trans, Kd_trans); // Compute output of the 1st loop		  
    
    ////////////////////////////////////////// COMPUTE BALANCING PID OUTPUT/ //////////////////////////////////////////////////
  
    Setpoint_bal = Output_trans; // Set the output [angle in deg] of the translation PID as Setpoint to the balancing PID loop      
    Input_bal = Theta_now; // Setting Theta_now as the input / current value to the PID algorithm              
    double error_bal = Setpoint_bal - Input_bal; // To decide actuator / motor rotation direction      
//  bal_PID.SetTunings(Kp, Ki, Kd); // Adjust the the new parameters          
    bal_PID.Compute_For_MPU(Kp_bal, Ki_bal, Kd_bal, omega_x);      
    Output_bal = map(abs(Output_bal), 0, Out_max_bal, Output_lower_bal, Out_max_bal);          
    mot_cont(error_bal, Output_bal); // Apply the calculated output to control the motor
  
//  Serial.print(Kp_bal);Serial.print(" , ");Serial.print(Ki_bal);Serial.print(" , ");Serial.print(Kd_bal);Serial.print(" , ");
//  Serial.print(Output);Serial.print(" , ");
//  Serial.println(Input);

    Blink_Led(); // Blink the LED
    t_loop_prev = t_loop_now; // Set prev loop time equal to current loop time for calculating dt for next loop
        
  }     
 
}

///////////////////////// Function for initializing / getting MPU Data ////////////////////////////////////////////////////////

void get_tilt_angle(){
  
  t_gyro_now = millis(); // Reading for gyro angle calculations
  get_MPU_data(); // Get Raw data acelX acelY acelZ giroX giroY giroZ  
  dt_gyro = (t_gyro_now - t_gyro_prev) / 1000.0; // Time difference for gyro angle calculations
  omega_x = (gyroX - A[3]) / 131.0; // Take Angular velocity reading for next step [deg/s];
    
  /*Since we will only need the ratios of accelerometer readings to calculate accelerometer angles, 
  we do not need to convert raw data to actual data */
  
  pitch = (atan2(accelY - A[1], accelZ + A[2]))*rad2deg; // Angle calculated by accelerometer readings about X axis in [deg]  
  Theta_now = alpha * (Theta_prev + (omega_x * dt_gyro)) + (1-alpha) * pitch; // Calculate the total angle using a Complimentary filter
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
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ

}

///////////////////////// Function for motor control ////////////////////////////////////////////////////////

void mot_cont(float e_rr, int Speed){

  if (abs(e_rr)>=fall_angle){stop_bot();}  
  else if (e_rr<0){fwd_bot(Speed);}  
  else if (e_rr>0){back_bot(Speed);}
}

void back_bot(int Speed){

  digitalWrite(lmot1, LOW);
  digitalWrite(lmot2, HIGH);
  digitalWrite(rmot1, LOW);
  digitalWrite(rmot2, HIGH);
  
  analogWrite(lmot3,Speed);    
  analogWrite(rmot3,Speed);  
  
}

void fwd_bot(int Speed){

  digitalWrite(lmot1, HIGH);
  digitalWrite(lmot2, LOW);
  digitalWrite(rmot1, HIGH);
  digitalWrite(rmot2, LOW);
  
  analogWrite(lmot3,Speed); 
  analogWrite(rmot3,Speed); 
  
}

void stop_bot(){

  digitalWrite(lmot1, LOW);
  digitalWrite(lmot2, LOW);
  digitalWrite(rmot1, LOW);
  digitalWrite(rmot2, LOW);
  
}

void rotate_bot(int Speed){

  digitalWrite(lmot1, HIGH);
  digitalWrite(lmot2, LOW);
  digitalWrite(rmot1, LOW);
  digitalWrite(rmot2, HIGH);
  
  analogWrite(lmot3,Speed); 
  analogWrite(rmot3,Speed); 
  
}

///////////////////////////////// READ BLUETOOTH ////////////////////////

void read_BT(){

  if (Serial.available()>0){

    char c = Serial.read();
    if (c =='1'){Kp_bal+=0.5;}
    else if(c=='2'){Kp_bal-=0.5;}
    else if (c =='3'){Kd_bal+=0.01;}
    else if(c=='4'){Kd_bal-= 0.01;}
    else if (c =='5'){Ki_bal+=1;}
    else if(c=='6'){Ki_bal-=1;}
    Serial.println(c);    
    }  
}

///////////////////////////////////// BLINK LED ///////////////

void Blink_Led(){
  
  t_led_now = millis();
  dt_led = t_led_now - t_led_prev;

  if (dt_led>blink_rate){

      if (led_state ==0){
       digitalWrite(pin, 1);   // turn the LED on (HIGH is the voltage level) 
       led_state = 1;       
      }
      else if(led_state == 1){
       digitalWrite(pin, 0);   // turn the LED on (HIGH is the voltage level) 
       led_state = 0;        
      }
      t_led_prev = t_led_now;    
  }  
}
