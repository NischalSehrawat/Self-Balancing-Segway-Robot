///////////////////////////////// Include all the required libraries ////////////////////////////////

#include <Wire.h>
#include <PID_v1.h>
#include <My_Motors.h>

///////////////////////////////// MPU-6050 parameters //////////////////////////////////////////////

long accelX, accelY, accelZ, gyroX, gyroY, gyroZ; // They record the raw accelerometer / gyro data

float omega_x_prev, omega_x_now;// they record the converted data into [deg/s]

//float A[6] = {-2043, 108, 1293,  48, -12, 19}; // To compensate for the error_now in MPU  

float A[3] = {0.0,0.0,0.0}; // To compensate for the error_now in MPU  

// Data is printed as: acelX acelY acelZ giroX giroY giroZ

double pitch, Theta_prev, Theta_now ; // For storing Angle data from Accelerometer
                   
double dt_gyro; // Variable to store time difference values for gyro angle calculations 

uint32_t t_gyro_prev, t_gyro_now; // timer for gyro unit

float alpha = 0.98; // Complimentary filter control parameter

////////////////////////////// MOTOR CONTROL PARAMATERS ////////////////////////////////////////////

int rmot1 = 7; int rmot2 = 8; // Pins for Right motor FW/BCK
int rmot3 = 9; // Pin for Right motor PWM

int lmot1 = 4; int lmot2 = 5; // Pins for Left motor FW/BCK
int lmot3 = 6; // Pins for Right motor PWM

float rpm_limit = 1.0; // RPM below this is considered 0

float avg_pt = 20.0;  // Number of points used for averaging the RPM signal

short PPR = 990; // Number of pulses per revolution of the wheel

float Final_Rpm_r, Final_Rpm_l; // Motor final averaged out RPM

volatile long ticks_r, t1_r, t2_r, ticks_l, t1_l, t2_l; // Tiks and times for calculating motor speeds

String units = "123/s"; // units in which RPM to be returned

short enc_pin_r1 = 2;short enc_pin_r2 = 3; // right motor encoder pins
short enc_pin_l1 = 18;short enc_pin_l2 = 19; // left motor encoder pins 

My_Motors rmot(&Final_Rpm_r, rpm_limit, avg_pt, PPR); // Right motor object
My_Motors lmot(&Final_Rpm_l, rpm_limit, avg_pt, PPR); // Left motor object


///////////////////////////////// Balancing PID parameters ///////////////////////////////////////////////////

double Input, Output, Setpoint; // Input output and setpoint variables defined

double Out_min = -255, Out_max = 255; // PID Output limits

double Kp, Ki, Kd; // Initializing the Proportional, integral and derivative gain constants

double Output_lower = 30.0; // PWM Limit at which the motors actually start to move

PID bal_PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, DIRECT); // Create a balancing PID instance

//short t_loop = 5.0; // Loop time

////////////// LED BLINKING PARAMETERS/////////////////////////

long t1_led, t2_led, dt_led;

bool led_state = 0;

int pin = 13;

int blink_rate = 100; // Blink after every millis

double t_loop_prev, t_loop_now, dt_loop;

double t_loop = 5;

void setup() {

    Serial.begin(115200);  

    pinMode(pin, OUTPUT);
    
    /////////////////////////////// Motor initialization ///////////////////////////////////////////
  
    pinMode(rmot1,OUTPUT);pinMode(rmot2,OUTPUT);pinMode(rmot3,OUTPUT); // Declaring right motor pins as output
  
    pinMode(lmot1,OUTPUT);pinMode(lmot2,OUTPUT);pinMode(lmot3,OUTPUT); // Declaring left motor pins as output
  
    pinMode(enc_pin_r1, INPUT_PULLUP); // Setup interrupt functions
    attachInterrupt(digitalPinToInterrupt(enc_pin_r1),countTicks_R1, RISING);
    pinMode(enc_pin_r2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(enc_pin_r2),countTicks_R2, RISING);
    pinMode(enc_pin_l1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(enc_pin_l1),countTicks_L1, RISING);
    pinMode(enc_pin_l2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(enc_pin_l2),countTicks_L2, RISING);
  
    ////////////////////////// PID  initialization ////////////////////////////////////////////////////////
    
    Setpoint = 0.0; // Set point for the angle w.r.t the vertical [degrees]
    
//    bal_PID.SetSampleTime(t_loop); // Set Loop time for PID [milliseconds]
    
    bal_PID.SetMode(AUTOMATIC); // Set PID mode to Automatic
    
    bal_PID.SetTunings(Kp, Ki, Kd);
    
    bal_PID.SetOutputLimits(Out_min, Out_max); // Set upper and lower limits for the maximum output limits for PID loop
    
  
    ////////////////////////// MPU initialization ///////////////////////////////////////////////////
    
    Wire.begin(); // Start wire library
    
    setupMPU(); // Initializing MPU6050
  
    get_MPU_data(); // Get initial angles of the MPU
  
    delay(100);
     
    // Calculate initial pitch angle
    
    pitch = (atan2(accelY - A[1], accelZ + A[2]))*57.32; // Rotation about X axis [deg]
  
    // set the total starting angle to this pitch 
    
    Theta_prev = pitch;

//    delay(5000);
      
    omega_x_prev = (gyroX - A[3]) / 131.0; // initial gyro reading

    t_gyro_prev = millis(); // Reading for gyro angle calculations
  
    t1_led = millis();

    t_loop_prev = millis();

    delay(50);
  
}


void loop() {

      t_loop_now = millis();

      dt_loop = t_loop_now - t_loop_prev;

      if (dt_loop>=t_loop){

          get_tilt_angle(); // Update the tilt angle readings  
        
          ////////////////////////////////////////// PID Action //////////////////////////////////////////////////
          
          Input = Theta_now; // Setting Theta_now as the input to the PID algorithm 
        
          read_BT(); // Read Kp, Ki, Kd from the serial bluetooth
    
          Kp = float((200.0 / 1023.0) *analogRead(A0));Ki = float((200.0 / 1023.0) *analogRead(A2));Kd = float((20 / 1023.0) *analogRead(A1)); 
    
//          Kp = 72.0; Kd = 1.4; Ki = 0.0;
          
          double my_error = Setpoint - Input; // To decide PID controller direction
          
    //      bal_PID.SetTunings(Kp, Ki, Kd); // Adjust the the new parameters 
          
    //      bal_PID.Compute_MPU(omega_x_prev, t_loop); // Compute all the PID actions and generate output
    
          bal_PID.Compute_For_MPU(Kp, Ki, Kd, omega_x_prev);
          
          Output = map(abs(Output), 0, Out_max, Output_lower, Out_max);
    
    //      Output =  map(abs(analogRead(A1) - 517), 0, 517, 30, 200);
          
          mot_cont(my_error, Output); // Apply the calculated output to control the motor
      
    //      Serial.print(Kp);Serial.print(" , ");Serial.print(Ki);Serial.print(" , ");Serial.print(Kd);Serial.print(" , ");
    //      Serial.print(Output);Serial.print(" , ");
    //      Serial.println(Input);
    
          Blink_Led();      
    
         rmot.getRPM(ticks_r, "123");
         
    //     lmot.getRPM(ticks_l, "123");
    
    //     Serial.println(motor_direction_R()*Final_Rpm_r); 
    
            Serial.println(dt_loop);
         
    //     Serial.println(motor_direction_L()*Final_Rpm_l) ; // Motor final averaged out RPM
    
          t_loop_prev = t_loop_now;
            
      }     
 
}

///////////////////////// Function for initializing MPU ////////////////////////////////////////////////////////

void get_tilt_angle(){
  
  t_gyro_now = millis(); // Reading for gyro angle calculations

  get_MPU_data(); // Get Raw data acelX acelY acelZ giroX giroY giroZ
  
  dt_gyro = (t_gyro_now - t_gyro_prev) / 1000.0; // Time difference for gyro angle calculations
    
  /*Since we will only need the ratios of accelerometer readings to calculate accelerometer angles, 
  we do not need to convert raw data to actual data */
  
  pitch = (atan2(accelY - A[1], accelZ + A[2]))*57.32; // Angle calculated by accelerometer readings about X axis in [deg]
  
  Theta_now = alpha * (Theta_prev + (omega_x_prev * dt_gyro)) + (1-alpha) * pitch; // Calculate the total angle using a Complimentary filter

  omega_x_prev = (gyroX - A[3]) / 131.0; // Take Angular velocity reading for next step [deg/s];

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

  if (abs(e_rr)>=45){stop_bot();}

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

////////////// Encoder interrupt service routines ////////////////////

void countTicks_R1(){ticks_r++; t1_r = millis();}

void countTicks_R2(){t2_r = millis();}

void countTicks_L1(){ticks_l++; t1_l = millis();}

void countTicks_L2(){t2_l = millis();}

///////////////// Motor direction determination functions /////////////////

int motor_direction_R(){

  bool check = digitalRead(enc_pin_r1) && digitalRead(enc_pin_r2);
  if (check){
    if (t1_r < t2_r){return -1;}
    else if (t1_r > t2_r){return 1;}    
  }  
}

int motor_direction_L(){

  bool check = digitalRead(enc_pin_l1) && digitalRead(enc_pin_l2);
  if (check){
    if (t1_l < t2_l){return 1;}
    else if (t1_l > t2_l){return -1;}    
  }
  
}


///////////////////////////////// READ BLUETOOTH ////////////////////////

void read_BT(){

  if (Serial.available()>0){

    char c = Serial.read();

    if (c =='1'){Setpoint+=0.5;}

    else if(c=='2'){Setpoint-=0.5;}

    else if (c =='3'){Kd+=0.01;}

    else if(c=='4'){Kd-= 0.01;}

    else if (c =='5'){Ki+=1;}

    else if(c=='6'){Ki-=1;}

    Serial.println(c);
    
    }
  
}


///////////////////////////////////// BLINK LED ///////////////

void Blink_Led(){
  
  t2_led = millis();

  dt_led = t2_led - t1_led;

  if (dt_led>blink_rate){

      if (led_state ==0){

       digitalWrite(pin, 1);   // turn the LED on (HIGH is the voltage level) 

       led_state = 1;
       
      }

      else if(led_state == 1){

       digitalWrite(pin, 0);   // turn the LED on (HIGH is the voltage level) 

       led_state = 0;
        
      }

      t1_led = t2_led;
    
  }

  
}




