///////////////////////////////// Include all the required libraries ////////////////////////////////

#include <Wire.h>

///////////////////////////////// MPU-6050 parameters //////////////////////////////////////////////

long accelX, accelY, accelZ; // They record the raw accelerometer data

float A_X, A_Y, A_Z; // They record the converted data in ["g"]

long gyroX, gyroY, gyroZ; //They record the raw gyro data

float omega_x, omega_y, omega_z;// they record the converted data into [deg/s]

float A[6] = {-2043, 108, 1293,  48, -12, 19}; // To caliberate MPU  

// Data is printed as: acelX acelY acelZ giroX giroY giroZ

/* They store the angles calculated from accelerometer data
   and the TOTAL angles in degrees i.e. theta + omega*dt [rad]
   roll and Theta_x are rotation angles about X axis
   pitch and Theta_y are rotation angles about Y axis */

double roll, pitch, Theta_x, Theta_y;  
									 
double dt_gyro; // Variable to store time difference values for gyro angle calculations 

uint32_t timer_gyro; // timer for gyro unit

float alpha = 0.98; // Complimentary filter control parameter

///////////////////////////////// PID parameters ///////////////////////////////////////////////////

double Kp, Ki = 3.0, Kd; // Initializing the Proportional, integral and derivative gain constants

double Kp_upper = 20, Kd_upper = 2.0, Output_lower = 30; // Upper and lower limits for variable mapping

int p_pin = A0; // Pin to be used for reading Kp from potentiometer

int d_pin = A1; // Pin to be used for reading Ki from potentiometer

int led = 13; // LED pin

////////////////////////////// MOTOR CONTROL PARAMATERS ////////////////////////////////////////////

int rmot1 = 7; int rmot2 = 8; // Pins for Right motor FW/BCK
int rmot3 = 9; // Pin for Right motor PWM

int lmot1 = 4; int lmot2 = 5; // Pins for Left motor FW/BCK
int lmot3 = 6; // Pins for Right motor PWM

float mot_diff_fac = 0.78; // Since the motors may have different characteristics, this factor tried to equalize their speeds. 

//////////////////////////////// All TIMINGS AND DELAYS///////////////////////////////////////////////////////////

int mot_delay = 20; // Minimum time required by the motor to respond to a change in direction [mS]

float sen_del = 10; // Sensor delay [mS]

//////////////////////// VARIABLES FOR DATA EXCHANGE TO / FROM RASPBERRY PI //////////////////////

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

// variables to hold the parsed data
char messageFromPC[numChars] = {0};
float Output = 0.0; // Variable to store PID output returned from Rpi for turning motors
float error = 0.0; // Variable to store error returned from Rpi for determing motor turning direction

boolean newData = false;

///////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  pinMode(led, OUTPUT); // pin where LED is attached
	
  /////////////////////////////// Motor initialization ///////////////////////////////////////////

  pinMode(rmot1,OUTPUT);pinMode(rmot2,OUTPUT);pinMode(rmot3,OUTPUT); // Declaring right motor pins as output

  pinMode(lmot1,OUTPUT);pinMode(lmot2,OUTPUT);pinMode(lmot3,OUTPUT); // Declaring left motor pins as output

  pinMode(p_pin, INPUT); pinMode(d_pin, INPUT); // Declaring potentiometer pins for reading Kp and Ki as input pins

  Serial.begin(115200); // Start serial monitor
  
  ////////////////////////// MPU initialization ///////////////////////////////////////////////////
  
  Wire.begin(); // Start wire library
  
  setupMPU(); // Initializing MPU6050

  get_MPU_data(); // Get initial angles of the MPU

  delay(100);
   
  /* calculate initial Roll and Pitch angles. Rotation about X axis is called ROLL, 
    Rotation about Y axis is called PITCH. My MPU X axis is aligned with the motor axis */
  
  roll = (atan2(accelY + A[1], accelZ + A[2]))*57.32; // Rotation about X axis [deg]
  pitch = (atan2(-accelX + A[0], accelZ + A[2]))*57.32; // Rotation about Y axis [deg] although we do not need this value

  // set the total starting angle to this pitch and roll
  
  Theta_x = roll;
  Theta_y = pitch;

  //Take a time reading in microseconds when the compiler passes this line
  
  timer_gyro = millis(); // Reading for gyro angle calculations

  delay(500);
  
}


void loop() {

  get_MPU_data(); // Raw data acelX acelY acelZ giroX giroY giroZ
  
  delay(sen_del);
  
  omega_x = (gyroX + A[3]) / 131.0; // Converting raw gyro data to Angular velocity [deg/s]

  dt_gyro = ((millis() - timer_gyro)/1000.0); // Time difference for gyro angle calculations
  
  timer_gyro = millis(); // Take a reading again at this step.
  
  /*Since we will only need the ratios of accelerometer readings to calculate accelerometer angles, 
  we do not need to convert raw data to actual data */
  
  roll = (atan2(accelY + A[1], accelZ + A[2]))*57.32; // Angle calculated by accelerometer readings about X axis in [rad]
  
  Theta_x = alpha * (Theta_x + (omega_x * dt_gyro)) + (1-alpha) * roll; // Calculate the total angle using a Complimentary filter
  
  
  ////////////////////////////////////////// PID ACTION //////////////////////////////////////////////////
  
  Kp = (float)(Kp_upper/1024.0)*analogRead(p_pin); // Read Kp values from potentiometer and map it to a value that is less than Kp_upper
  
  Kd = (float)(Kd_upper/1024.0)*analogRead(d_pin); // Read Kd values from potentiometer and map it to a value that is less than Kd_upper

  send_data(); // Send data to Python / Raspberry Pi
  
  recvWithStartEndMarkers(); // Recieve data from Python / Raspberry Pi
  
  if (newData == true) {
      strcpy(tempChars, receivedChars);
          /* this temporary copy is necessary to protect the original data
             because strtok() used in parseData() replaces the commas with \0 */
			 
      parseData(); // Parse recieved data
	  
//      showParsedData();
      newData = false;

  }

// mot_cont(Output, error); // Apply the calculated output to control the motor

}
////////////////////////// FUNCTIONS FOR SENDING / RECEIVING / PARSING SERIAL DATA TO / FROM PYTHON / RASPBERRY PI /////////////////

/*All the data recieving and parsing functions have been taken from (http://forum.arduino.cc/index.php?topic=396450)

 and modified as per my needs*/

void send_data(){

  Serial.print('<');Serial.print(',');Serial.print(Kp);Serial.print(',');
  Serial.print(Kd);Serial.print(',');Serial.print(Ki);Serial.print(',');
  Serial.print(Theta_x);Serial.print(',');Serial.println('>');
  
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}


void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    Output = atof(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ",");
    error = atof(strtokIndx);     // convert this part to a float

}

void showParsedData() {
    Serial.print("Message ");
    Serial.println(messageFromPC);
    Serial.print("Output ");
    Serial.println(Output);
    Serial.print("Error ");
    Serial.println(error);
}


///////////////////////////////////////////// BLINK LED ///////////////////////////////////////////////


void blink_led(){
      for (int i = 0; i<5; i++){
      digitalWrite(led, HIGH);
      delay(5);
      digitalWrite(led, LOW);
      delay(5);          
    
  }
}

///////////////////////// FUNCTION FOR INITIALISING MPU & GETTING MPU DATA ////////////////////////////////////////////////

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

///////////////////////// FUNCTIONS FOR MOTOR CONTROL ///////////////////////////////////////////////////

void mot_cont(float controller_output, float e_rr){

  if (abs(e_rr)>=75){ // If the robot has fallen down, stop the motors
    stop_bot();
  }

  else if (e_rr<0){

    fwd_bot(abs(controller_output));
  }

   else if (e_rr>0){

    back_bot(abs(controller_output));
  }

  
}

void fwd_bot(int mot_speed){

  digitalWrite(lmot1, LOW);
  digitalWrite(lmot2, HIGH);
  digitalWrite(rmot1, LOW);
  digitalWrite(rmot2, HIGH);
  
  analogWrite(lmot3,mot_speed); // 10 is added to compensate for the motor sluggishness  
  analogWrite(rmot3,mot_diff_fac*mot_speed); // mot_diff_fac accounts for motor speed difference characteristics 

  
}

void back_bot(int mot_speed){

  digitalWrite(lmot1, HIGH);
  digitalWrite(lmot2, LOW);
  digitalWrite(rmot1, HIGH);
  digitalWrite(rmot2, LOW);
  
  analogWrite(lmot3,mot_speed); 
  analogWrite(rmot3,mot_diff_fac*mot_speed); // mot_diff_fac accounts for motor speed difference characteristics
  
}

void stop_bot(){

  digitalWrite(lmot1, LOW);
  digitalWrite(lmot2, LOW);
  digitalWrite(rmot1, LOW);
  digitalWrite(rmot2, LOW);
  
}





