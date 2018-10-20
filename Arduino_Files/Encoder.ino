
const int enc_pin_r = 2;

volatile long ticks_r = 0;

long n_prev_r, n_now_r, t_prev_mot, t_now_mot = 0; // Parameters to calculate motor RPM

float inst_rpm_r, rpm_now_r, rpm_prev_r; // Variable to keep track of RPM

float Motor_RPMs[] = {0};

int rpm_limit = 1; // Below this rpm, return "0"

float avg_points = 10.0; // Number of points to be considered for taking averages

float beta; // Variable for calculating exponential averages

short ppr = 990; // Pulses per revolution for each motor encoder 

float rpm_2_rad = 2.0*3.14/60.0 ; // RPM to rad / s conversion factor

//=============================

byte rmot1 = 7; byte rmot2 = 8; // Pins for Right motor FW/BCK
byte rmot3 = 9; // Pin for Right motor PWM

void setup() {

  Serial.begin(9600);
  pinMode(enc_pin_r, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(enc_pin_r),countTicks_r, RISING);
  beta = 1 - 1/ avg_points;
  t_prev_mot = millis(); // Log time here to calculate dt_mot
  pinMode(13, OUTPUT);
  
}

void loop() {
  
  get_rpm("rad/s");

  }

void countTicks_r(){
  
  ticks_r++;  

 
}

void get_rpm(String units){  

  t_now_mot = millis(); // log what time is it now in milliseconds 
  
  int dt_mot = t_now_mot - t_prev_mot; // Calculate change in time [mS] 

  if (dt_mot!=0){ // To prevent Nan and inf values

  //================= FOR RIGHT MOTOR =================== 
      
  n_now_r = ticks_r;  // Log down the present tick count   
  
  int dn_r = n_now_r - n_prev_r; // Calculate change in ticks

  inst_rpm_r = (1000.0 * dn_r / dt_mot)*(60.0 / ppr); // Calculate instantaneous RPM
    
  rpm_now_r = beta*rpm_prev_r + (1 - beta)*inst_rpm_r; // Do exponential averaging

  if (rpm_now_r < rpm_limit){

     rpm_now_r = 0.0;
  }

  if (units == "rad/s"){

    Motor_RPMs[0] = rpm_2_rad * rpm_now_r;
        
  }

  else {

    Motor_RPMs[0] = rpm_now_r;
    
  }

  Serial.print(Motor_RPMs[0]);Serial.print(" , "); Serial.println(rpm_2_rad*inst_rpm_r);
  
  n_prev_r = n_now_r; // Reassign parameters for next loop calculations

  rpm_prev_r = rpm_now_r;
  
  t_prev_mot = t_now_mot;  
  
  }
      
}
  


void back_bot(){

  digitalWrite(rmot1, HIGH);
  delay(10);
  digitalWrite(rmot2, LOW);  
  delay(10);

  analogWrite(rmot3,255); // mot_diff_fac accounts for motor speed difference characteristic
  delay(10);

  
}
