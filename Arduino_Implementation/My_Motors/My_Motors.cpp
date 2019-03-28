/**********************************************************************************************

 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <My_Motors.h>

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
 
My_Motors::My_Motors(float *Final_rpm, float rpm_lim, float avg_pt, short PPR)

{	
	my_final_rpm = 	Final_rpm;

	rpm_limit = rpm_lim;	
	
	avg_points = avg_pt;
	
	ppr = PPR; // Pulses per revolution for each motor encoder 
	
	rpm_2_rad = 2.0*3.14/60.0 ; // RPM to rad / s conversion factor	
	
	beta = 1 - 1/ avg_points; // Variable for calculating exponential averages	
	
	n_prev = 0; // Parameters for encoder increment	
	
	t_prev = millis(); // Parameters to keep track of time		 
	
	rpm_now, rpm_prev, inst_rpm  = 0; 
}

void My_Motors::getRPM(long ticks, String units){  

  long t_now = millis(); // log what time is it now in milliseconds 
  
  int dt = t_now - t_prev; // Calculate change in time [mS] 

  if (dt!=0){ // To prevent Nan and inf values
    
	long n_now = ticks;  // Log down the present tick count   
  
	int dn = n_now - n_prev; // Calculate change in ticks

	inst_rpm = (0.25 * 1000.0 * dn / dt)*(60.0 / ppr); // Calculate instantaneous RPM
    
	rpm_now = beta*rpm_prev + (1 - beta)*inst_rpm; // Do exponential averaging

	if (abs(rpm_now) < rpm_limit ){rpm_now = 0.0;}
	
	if (units == "rad/s"){*my_final_rpm =  rpm_2_rad * rpm_now;}

	else {*my_final_rpm =  rpm_now;}
  
	n_prev = n_now; // Reassign parameters for next loop calculations

	rpm_prev = rpm_now;
  
	t_prev = t_now;   
  
	}
      
}


float My_Motors::getInstRpm(){return inst_rpm;}