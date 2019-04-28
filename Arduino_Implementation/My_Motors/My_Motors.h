#ifndef My_Motors_h
#define My_Motors_h
#define LIBRARY_VERSION	1.1.1

class My_Motors{

  public:

  //commonly used functions **************************************************************************
  
    My_Motors(float *Final_rpm, float rpm_lim, float avg_pt, short PPR);	// Constructor for My_Motor class                                           

    void getRPM(long ticks, String units);            // * Calculates motor RPM
	
	float getInstRpm();                                // Return instantaneous RPM

	void set_Ninit(int ticks); 									// Set initial encoder ticks
	
	int get_Dn(int enc_count); 									// Get the difference between initial encoder count and encoder count now

  private:
	
	long n_prev; // Parameters for encoder increment	
	long t_prev; // Parameters to keep track of time
	 

	float inst_rpm; // Parameters to keep track of RPM
	float rpm_now; 
	float rpm_prev; 
	float *my_final_rpm;

	int n_init; // Encoder count initialiser for driving straight

	float rpm_limit; // Below this rpm, return "0"

	float avg_points; // Number of points to be considered for taking averages

	float beta; // Variable for calculating exponential averages

	short ppr; // Pulses per revolution for each motor encoder 

	float rpm_2_rad; // RPM to rad / s conversion factor
	
};
#endif

