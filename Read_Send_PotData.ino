
int p_pin = A0; // Pin to be used for reading Kp from potentiometer

int d_pin = A1; // Pin to be used for reading Kd from potentiometer


///////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  pinMode(p_pin, INPUT); pinMode(d_pin, INPUT); // Declaring potentiometer pins for reading Kp and Ki as input pins

  Serial.begin(115200); // Start serial monitor
  
  delay(500);
  
}


void loop() {

  
  Kp = analogRead(p_pin); // Read Kp values from potentiometer and map it to a value that is less than Kp_upper
  
  Kd = analogRead(d_pin); // Read Kd values from potentiometer and map it to a value that is less than Kd_upper

  send_data(); // Send data to Python / Raspberry Pi
  
}
////////////////////////// FUNCTIONS FOR SENDING / RECEIVING / PARSING SERIAL DATA TO / FROM PYTHON / RASPBERRY PI /////////////////


void send_data(){

  Serial.print('<');Serial.print(',');Serial.print(Kp);Serial.print(',');
  Serial.print(Kd);Serial.print(',');Serial.print(Ki);Serial.print(',');
  Serial.println('>');
  
}

