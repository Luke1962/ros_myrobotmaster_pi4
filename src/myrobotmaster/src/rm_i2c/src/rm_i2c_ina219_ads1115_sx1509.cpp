/*
 Versione che usa i seguenti  dispositivi
 * ina219: current-voltage sensor
 * sx1509: GPIO Expander
 * 
* 
* build : catkin_make  -DCATKIN_WHITELIST_PACKAGES="rm_i2c"
* run: sudo -E /home/pi/ros/src/myrobotmaster/src/rm_i2c/launch/rmaa.sh;
* 
* 
* mraa  ===============================00000
Public Member Functions
 	I2c (int bus, bool raw=false) 
 	I2c (void *i2c_context) 
	Result 		frequency (I2cMode mode) 
	Result 		address (uint8_t address) 
	uint8_t 	readByte () 
	int 		read (uint8_t *data, int length) 
	uint8_t 	readReg (uint8_t reg) 
	uint16_t 	readWordReg (uint8_t reg) 
	int 		readBytesReg (uint8_t reg, uint8_t *data, int length) 
	Result 		writeByte (uint8_t data) 
	Result 		write (const uint8_t *data, int length) 
	Result 		writeReg (uint8_t reg, uint8_t data) 
	Result 		writeWordReg (uint8_t reg, uint16_t data)
 */
  #define NODENAME "rm_i2c"
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///	INCLUDES
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <math.h> /// per fabs()
#include <stdlib.h>


#include "mraa/common.hpp"
#include "mraa/i2c.hpp"

#include <dbg.h>




#include <cstdlib>
#include <pthread.h>

using namespace std;

/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///  AD CONVERTER                       
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////

/// ADC --------------------
int i2cHandler_adc; 
/// GPIO Expander--------------------
int i2cHandler_gpio; 



/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///  SYSINFO UPTIME                       
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
#if 1


	//~ struct sysinfo {
		//~ long uptime;             /* Seconds since boot */
		//~ unsigned long loads[3];  /* 1, 5, and 15 minute load averages */
		//~ unsigned long totalram;  /* Total usable main memory size */
		//~ unsigned long freeram;   /* Available memory size */
		//~ unsigned long sharedram; /* Amount of shared memory */
		//~ unsigned long bufferram; /* Memory used by buffers */
		//~ unsigned long totalswap; /* Total swap space size */
		//~ unsigned long freeswap;  /* swap space still available */
		//~ unsigned short procs;    /* Number of current processes */
		//~ unsigned long totalhigh; /* Total high memory size */
		//~ unsigned long freehigh;  /* Available high memory size */
		//~ unsigned int mem_unit;   /* Memory unit size in bytes */
		//~ char _f[20-2*sizeof(long)-sizeof(int)]; /* Padding for libc5 */
	//~ };
	//~ struct sysinfo pi3info;  /// richiede include sys/sysinfo.h

	#include <linux/kernel.h>
	//~ #include <linux/sys.h>
	#include <stdio.h>

	#include <stdlib.h>	//per itoa ?
	//~ #include <sys/sysinfo.h>
	#include  "sys/sysinfo.h"

	int uptimeSec()
	{
		/* Conversion constants. */
		const long minute = 60;
		const long hour = minute * 60;
		const long day = hour * 24;
		const double megabyte = 1024 * 1024;
		/* Obtain system statistics. */
		struct sysinfo si;
		sysinfo (&si);		

		return si.uptime;
	}

	int sysInfo ()
	{
		/* Conversion constants. */
		const long minute = 60;
		const long hour = minute * 60;
		const long day = hour * 24;
		const double megabyte = 1024 * 1024;
		/* Obtain system statistics. */
		struct sysinfo si;
		sysinfo (&si);
		/* Summarize interesting values. */
		printf ("system uptime : %ld days, %ld:%02ld:%02ld\n", 
		 	si.uptime / day, (si.uptime % day) / hour, 
		 	(si.uptime % hour) / minute, si.uptime % minute);
		printf ("total RAM   : %5.1f MB\n", si.totalram / megabyte);
		printf ("free RAM   : %5.1f MB\n", si.freeram / megabyte);
		printf ("process count : %d\n", si.procs);
		return 0;
	}


#endif



/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///  A / D  C O N V E R T E R   ADS1115    4 canali 16bit 860sps                          
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
#if 1

	#include <stdio.h>
	#include <stdint.h>	
	#include <ADS1115.h>
	#include <fstream> // per operazioni su file ofstream
	//using namespace std;


	#define ADC_PIN_BASE 100
	
	///canali AD converter
	#define AD0 0
	#define AD1 1
	#define AD2 2
	#define AD3 3

	void delay(int ms){
	//todo
	}
	int analogRead(int channel){
	
	}

	/// --------------------------------------------
	/// lettura tensione da uno dei 4 canali
	/// --------------------------------------------
	float_t adc_read_volt(int channel)
	{
		double volt = -1.0;
		if ((channel >= 0 ) && (channel <=3 ) ) /// ok 4 canali
		{
			int16_t value = (int16_t) analogRead(ADC_PIN_BASE + channel );
			volt = value * (4.096 / 32768);
			
		}
		
		return volt;
	}
	
	double adc2batteryvolt=5 ; //fattore di conversione
	
	
	float adc_read_battery_volt()
	{
		// 15.9 è la tensione misurata col tester all'uscita del Battery Monitor MAX741
		//  3.17 è la tensione misurata in ingresso all'AD

		float vbat = 0.0;
		while (vbat < 5.0)
		{
			//v= adc_read_volt(AD0)*(16.0/3.2);
			vbat= adc_read_volt(AD0)*(adc2batteryvolt);
			
		}
		
		return vbat;
	}

	
	float adc_read_battery_ampere()
	{
		float a =0.0;
		while (a == 0.0){
			a= adc_read_volt(AD1);
		}
		// Il Max741 fornisce 1V per Ampere quindi non ha bisogno di fattori di conversione
		return a;
	}

	
	float adc_read_battery_Watt()
	{
		// Il Max741 fornisce 1V per Ampere
		return adc_read_battery_volt()*adc_read_battery_ampere();
	}

	float  WattHours =0.0;
	
	void updateWattHours(float watt, float_t sec){
		float dWh = watt*sec/(float)3600.0;
		WattHours += dWh;
		printf("\nWattHours %f,  dWh %f @sec %f",WattHours, dWh, sec);
		//return WattHours;
	}



	



	
#endif


/// ////////////////////////////////////////////////////////////////////////////////////////////	
/// ////////////////////////////////////////////////////////////////////////////////////////////	
/// I N A 2 1 9    S E T U P 
/// ////////////////////////////////////////////////////////////////////////////////////////////	
/// ////////////////////////////////////////////////////////////////////////////////////////////	
#include <ina219.h>

Adafruit_INA219 ina219;
void setup_INA219( ){
  
}

/// ////////////////////////////////////////////////////////////////////////////////////////////	
/// ////////////////////////////////////////////////////////////////////////////////////////////	
/// SX1509 GPIO EXPANDER   S E T U P 
/// ////////////////////////////////////////////////////////////////////////////////////////////	
/// ////////////////////////////////////////////////////////////////////////////////////////////	
#include <sx1509.h>

// SX1509 I2C address (set by ADDR1 and ADDR0 (00 by default):
SX1509 gpioexp; // Create an SX1509 object to be used throughout

// SX1509 pin definitions:
// The breathable pins are 4, 5, 6, 7, 12, 13, 14, 15 only.
const byte PIN_LED_R = 5; // LED connected to pin 15
const byte PIN_LED_G = 6; // LED connected to pin 15
const byte PIN_LED_B = 7; // LED connected to pin 15
const byte PIN_LIDAR_ON = 3;

void setup_gpioexp_sx1509(mraa::I2c *i2c){
   // Use the internal 2MHz oscillator.
  // Set LED clock to 500kHz (2MHz / (2^(3-1)):
  	gpioexp.init(i2c);

  	gpioexp.clock(INTERNAL_CLOCK_2MHZ, 3);
			
	//gpioexp.init(&i2c7);
	// Call io.begin(<address>) to initialize the SX1509. If it
	// successfully communicates, it'll return 1.
	if (!gpioexp.begin(SX1509_ADDRESS))
	{
		ROS_WARN("SX1509 FAILED INITIALIZATION...");
	}

	// Call io.pinMode(<pin>, <mode>) to set an SX1509 pin as
	// an output:
	gpioexp.pinMode(PIN_LED_R, OUTPUT);
	gpioexp.pinMode(PIN_LED_G, OUTPUT);
	gpioexp.pinMode(PIN_LED_B, OUTPUT);

}
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
/// BATTERIA	
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
#if 1
	#define BATTERY_CAPACITY_WH  41.0f // Energia quando la batteria è completamente carica (3.7V x 3s x 1.8 Ah per battery x 4p = 80Wh circa) 

	#define END_CHARGE_CURRENT_THRESHOLD 0.009
	
	#define MAX_EXPECTED_CURRENT_A   2.0 
 
	#define SHUNT_RESISTENCE 0.1f //(scritta R100)

	#define FILE_CHARGE "/home/pi/ros/src/myrobotmaster/src/rm_i2c/charge_wh.txt"

	class Battery {
		public:
			Battery();//Battery(mraa::I2c *i2c_bus);
			void init(mraa::I2c *i2c_bus);
			float volts;
			float current;
			float shuntV;
			float watt;
			float whOut;			
			float charge; //livello di carica in %
			bool isPlugged; //true quando la corrente < 0
			bool isChargeCompleted; //true quando la corrente di carica va sotto una certa soglia
			bool blWarnChargingStatusChanged; //passa a true se cambia lo stato carica-scarica
			void updateReadings(float  update_interval_sec);
			void calibrateSensor(float maxAmpere);
			uint8_t volt2charge(float vbat,  uint16_t minVoltage, uint16_t maxVoltage);
			void saveCharge()	;
			void restoreCharge();
			std_msgs::String msg_speech;
			
			
		private:
			mraa::I2c *i2c;
			float volt2charge(float v );
	};
	
	
	
	void Battery::calibrateSensor(float maxAmpere = MAX_EXPECTED_CURRENT_A){
	
		/*
		float Current_LSB = maxAmpere / (2^15); //2^15= 32768
		printf("\nCurrent_LSB = MAX_EXPECTED_CURRENT_A / (2^15) = %f",Current_LSB );


		int Calib_Value = trunc(0.04096 / (Current_LSB * SHUNT_RESISTENCE)); //67108
		printf("\nCalib_Value = trunc(0.04096 / (Current_LSB * SHUNT_RESISTENCE) = %d\n",Calib_Value );

		i2c->writeWordReg(0x05,(int)(2*Calib_Value));
		*/
		ina219.powerSave(false);
		ina219.setCalibration_32V_2A();
	}
	void Battery::updateReadings(float  update_interval_sec){
	
		//Volt
		volts = ina219.getBusVoltage_V();
		shuntV =(float)ina219.getShuntVoltage_raw()/100000;
		
		//Current
		current =shuntV/SHUNT_RESISTENCE ;//ina219.getCurrent_mA() /1000.0;
		//float shuntV =  ina219.getShuntVoltage_mV()*1000 ;
		//printf("\n--- getCurrent_raw: %d | shunt V %f | A %f---\n",ina219.getCurrent_raw(),shuntV,shuntV* 67108);
		
		//watt
		watt =volts*fabs(current);  // ina219.getPower_mW()/1000 ;//
		
		// carica completa?
		if (fabs(current)< END_CHARGE_CURRENT_THRESHOLD){
			isChargeCompleted = true;
		} else
		{
			isChargeCompleted = false;
		}
		
		
		//calcolo whOut
		if (current< 0) //sta caricando
		{
			
			whOut -= watt * update_interval_sec/3600.0f;			
		}else if (current> 0) // in scarica
		{
			whOut += watt * update_interval_sec/3600.0f;	
		}else //carica completata
		{
			whOut = BATTERY_CAPACITY_WH;
		}
 

		charge = volt2charge(volts, 9, 12.55);
		//charge= 100*whOut/BATTERY_CAPACITY_WH;
		bool prevChargingStatus = isPlugged;
		
		isPlugged = ((current <= 0) || (  isChargeCompleted  ) )? true: false;
		
		if (prevChargingStatus != isPlugged)
		{
			// presa attaccata o staccata, alzo il flag
			blWarnChargingStatusChanged =true;
		}
	}
	
	/*Battery::Battery(mraa::I2c *i2c_bus){
		i2c = i2c_bus;
		calibrateSensor();
		restoreCharge();
		updateReadings(uptimeSec());
	}*/
	Battery::Battery(){}
	void Battery::init(mraa::I2c *i2c_bus){
		i2c = i2c_bus;
		calibrateSensor();
		restoreCharge();
		updateReadings(uptimeSec());
	}
		 

	float Battery::volt2charge(float vbat){
		// 100* (v-vmin)/(vmax-vmin)
		//Batterie al litio 3S: 100% = v4.2 per cella X 3 = 12.6v 
		float c=0.0;
		float vcell = vbat/3.0;

		if (vcell >= 4.2){ //2.15v  per elemento = 100%
			c=  100 ;
		}else  if (vcell > 4.16) { //2.1v  per elemento = 80%
			c=100;
		}else  if (vcell > 3.95) { //2.1v  per elemento = 80%
			c=80;
		}else if (vcell > 3.85) { //2.05v  per elemento = 60%
			c=60;
		}else if (vcell > 3.8) { //2.03v  per elemento = 50%
			c=50;
		}else if (vcell > 3.75) { //1.983v  per elemento = 40%
			c=40;
		}else if (vcell > 3.7) { //1.983v  per elemento = 30%
			c=30;
		}else if (vcell > 3.65) { //1.913v  per elemento = 20%
			c=20;
		}else if (vcell > 3.4) { //1.853v  per elemento = 10%
			c=10;
		}else if (vcell > 3.0) { //1.853v  per elemento = 10%
			c=5;
		}else {
			c=0;
		}
		return c;
	}

	/**
	 * Asymmetric sigmoidal approximation
	 * https://www.desmos.com/calculator/oyhpsu8jnw
	 *
	 * c - c / [1 + (k*x/v)^4.5]^3
	 */
 	uint8_t Battery::volt2charge(float voltage,  uint16_t minVoltage, uint16_t maxVoltage){
		//static inline uint8_t asigmoidal(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {
		uint8_t result = 101 - (101 / pow(1 + pow(1.33 * (voltage - minVoltage)/(maxVoltage - minVoltage) ,4.5), 3));
		return result >= 100 ? 100 : result;
	}

	void Battery::saveCharge(){
		std::ofstream file(FILE_CHARGE, ios::out ); //std::ofstream file(FILE_CHARGE, ios::out | ios::binary);
		//ofstream file;
		//file.open (FILE_CHARGE);
		file << whOut;
		file.close();
		printf("\nSaved charge level Wh: %f to file: %s\n",whOut, FILE_CHARGE);
	}

	void Battery::restoreCharge(){
		std::ifstream file;
		std::string line;
		file.open (FILE_CHARGE, ios::in );  //file.open (FILE_CHARGE, ios::in | ios::binary);
		if (file.is_open()){
			std::getline(file, line);
			 
			whOut= (float)std::stoi(line);
			file.close();
			printf("\nRestored charge level Wh: %f from file: %s\n",whOut, FILE_CHARGE);
		}else
		{
			printf("Error restoring whOut");
		}
	}	
	
#endif



/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
/// SPEECH MACRO	
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
static std::string lastSpeech;
#define SPEECH(S) 	msg_chatter.data = S ; 	pub_chatter.publish(msg_chatter); lastSpeech=S;
#define SPEECH_NOREPEAT(S) if(! S.compare(lastSpeech) == 0) {SPEECH(S); }
#define SPEECH_CODED(S) 	msg_chatter.data = S ; 	pub_speech_coded.publish(msg_chatter); lastSpeech=S;
#define SPEECH_CODED_NOREPEAT(S) if(! S.compare(lastSpeech) == 0) {SPEECH_CODED(S); }

/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///	Ros_i2c
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
#if 1
	class Ros_i2c
	{
	public:
		Ros_i2c(ros::NodeHandle &nh);
		double rate ; // 5Hz main loop rate

	private:

		
		// methods
		//void cbk_lidar_enable(std_msgs::Bool);
		void nodeShutdown();
		void ledStatusBatUpdate( Battery *bat);
		
		//vars 
		int node_rate_hz;
		//objects
		Battery bat;	
			
		// Subscribes
		//ros::Subscriber sub_lidar_enable;

	};


	////////////////////////////////////////////////////////////////////////////////////
	// Costruttore
	////////////////////////////////////////////////////////////////////////////////////
	Ros_i2c::Ros_i2c(ros::NodeHandle &nh)
	{
		ROS_INFO("####### rm_i2c ############");
		//-----------------------------------------------------------------
		// Leggo i parametri (launch file)
		//-----------------------------------------------------------------

		#if 1  ///S E T U P 	 HW, ROS
		
			/// ////////////////////////////////////////////////////////////////////////////////////////////	
			/// ////////////////////////////////////////////////////////////////////////////////////////////	
			/// R O S  S E T U P 
			/// ////////////////////////////////////////////////////////////////////////////////////////////	
			/// ////////////////////////////////////////////////////////////////////////////////////////////	
			//ros::init(argc, argv, NODENAME); ///must be called before creating a node handle
			//ros::NodeHandle nh; 
			ros::NodeHandle _nh("~");
			/// ///////////////////////////////////////////////////////////////
			/// ///////////////////////////////////////////////////////////////
			///								
			///	LOAD PARAMETERS					
			///								
			/// ///////////////////////////////////////////////////////////////
			/// ////////////////////////////////////////////////////////////////
			int battery_print_interval_sec=2;
			_nh.getParam("battery_print_interval_sec",battery_print_interval_sec);

			rate = 10.0;		
			node_rate_hz=2;
			_nh.getParam("node_rate_hz",node_rate_hz);
			float node_loop_interval_sec=(float)1/node_rate_hz;
			ROS_INFO("Node Hz %d ( interval: %f sec)",node_rate_hz,node_loop_interval_sec);

			// mraa setup ------------------------------
			ROS_INFO("Inizializzazione mraa....");
			#define I2C_BUS 0  /*0 corrisponde al bus 7 I2C del RockPi4*/
			mraa::I2c i2c7(I2C_BUS);		
			//uint8_t rx_tx_buf[MAX_BUFFER_LENGTH];
			
			
			// ==============================================================================
			//setup HW 
			// ==============================================================================
			
			
			i2c7.address(SX1509_ADDRESS);
			// sx1509  ----------------------------------------------------------------------
			setup_gpioexp_sx1509(&i2c7);
			
			// giro di led
			gpioexp.digitalWrite(PIN_LED_R,1); delay(1000) ;gpioexp.digitalWrite(PIN_LED_R, 0);
			gpioexp.digitalWrite(PIN_LED_G,1); delay(1000) ;gpioexp.digitalWrite(PIN_LED_G, 0);
			gpioexp.digitalWrite(PIN_LED_B,1); delay(1000) ;gpioexp.digitalWrite(PIN_LED_B, 0);
			
			// To breathe an LED, make sure you set it as an
			// ANALOG_OUTPUT, so we can PWM the pin:
			//gpioexp.pinMode(PIN_LED_B, ANALOG_OUTPUT);

			gpioexp.pinMode(PIN_LIDAR_ON, OUTPUT);
			
			//gpioexp.digitalWrite(PIN_LED_B, 1);
			gpioexp.digitalWrite(PIN_LIDAR_ON, 0);

			// Use the internal 2MHz oscillator.
			// Set LED clock to 500kHz (2MHz / (2^(3-1)):
			gpioexp.clock(INTERNAL_CLOCK_2MHZ, 3);


			// Breathe an LED: 1000ms LOW, 500ms HIGH,
			// 500ms to rise from low to high
			// 250ms to fall from high to low
			//gpioexp.breathe(PIN_LED_B, 1000, 500, 500, 250);
			// The timing parameters are in milliseconds, but they 
			// aren't 100% exact. The library will estimate to try to 
			// get them as close as possible. Play with the clock 
			// divider to maybe get more accurate timing.

	  
	  
	  		//---------------------------------------------------
			// ina219 (voltage current sensor) 
			//--------------------------------------------------
			i2c7.address(INA219_ADDRESS);	
			ina219.begin(&i2c7);
			ina219.setCalibration_32V_2A();
			ROS_INFO("INA219 inizialization...");
			ROS_INFO("Measuring voltage and current with INA219 ...");	
				
			bat.init(&i2c7);
			//bat.calibrateSensor(2.0);
			bat.updateReadings((float)node_loop_interval_sec);
			
			printf("\n %f Wh Estimated from power on @%d sec ago\n",bat.whOut, uptimeSec());
			
				



			
			
			///---------------------------------------------------------------
			/// ROS Publisher 
			///---------------------------------------------------------------
			//The second parameter to advertise() is the size of the message queue
			// per vederlo in un pannello mqtt modificare il file pi3-mqtt.yaml
			std_msgs::Float64 msg_battery_volt;
			ros::Publisher pub_battery_volt = nh.advertise<std_msgs::Float64>("/bat_volt", 1);
			msg_battery_volt.data = 0x0;

			std_msgs::Float64 msg_battery_ampere;
			ros::Publisher pub_battery_ampere = nh.advertise<std_msgs::Float64>("/bat_ampere", 1);
			msg_battery_ampere.data = 0.0;
			
			std_msgs::Float64 msg_battery_watt;
			ros::Publisher pub_battery_watt = nh.advertise<std_msgs::Float64>("/bat_watt", 1);
			msg_battery_watt.data = 0.0;

			std_msgs::Float64 msg_battery_watthours;
			ros::Publisher pub_battery_watthours = nh.advertise<std_msgs::Float64>("/bat_watthours", 1);
			msg_battery_watthours.data = 0.0;
			
			
			std_msgs::Float64 msg_battery_charge;
			ros::Publisher pub_battery_charge = nh.advertise<std_msgs::Float64>("/bat_charge", 1);
			msg_battery_charge.data = 0.0;		
			
			std_msgs::Bool msg_battery_isPlugged;
			ros::Publisher pub_battery_isPlugged = nh.advertise<std_msgs::Bool>("/bat_isPlugged",1);
			msg_battery_isPlugged.data = false;		


			std_msgs::Bool msg_battery_isChargeCompleted;
			ros::Publisher pub_battery_isChargeCompleted = nh.advertise<std_msgs::Bool>("/bat_ischargecompleted",1);
			msg_battery_isChargeCompleted.data = false;		

			std_msgs::String msg_chatter;
			ros::Publisher pub_chatter = nh.advertise<std_msgs::String>("/chatter", 1); // mia aggiunta

			std_msgs::String msg_speech;
			ros::Publisher pub_speech_coded = nh.advertise<std_msgs::String>("/speech_coded", 1); // mia aggiunta
			ros::Publisher pub_speech_once = nh.advertise<std_msgs::String>("/speech_once", 1); // mia aggiunta




			///---------------------------------------------------------------
			/// ROS SUBSCRIBES 
			///---------------------------------------------------------------
			
			//sub_lidar_enable = nh.subscribe("/lidar_enable", 1, &Ros_i2c::cbk_lidar_enable, this,
			//					  ros::TransportHints().tcpNoDelay());



			
			///---------------------------------------------------------------
			/// Vars init
			///---------------------------------------------------------------
			/// inizializza il consumo ipotizzando un consumo costante fino a questo momento
			//initial_test();
			

		

		#endif
		

		
		/// ////////////////////////////////////////////////////////////////////////////////////////////	
		/// ////////////////////////////////////////////////////////////////////////////////////////////	
		/// R O S  L O O P 
		/// ////////////////////////////////////////////////////////////////////////////////////////////	
		/// ////////////////////////////////////////////////////////////////////////////////////////////	
		ros::Duration battery_interval(5.0); // 5sec intervallo di stampa su terminale
		ros::Time battery_time_last = ros::Time::now();
		ros::Time current_loop_time = ros::Time::now();
		ros::Time  g_last_loop_time;
		g_last_loop_time =current_loop_time;
		


		
		while(nh.ok() ){
			ros::spinOnce();
			ros::Time current_loop_time = ros::Time::now();
			 
					
			//ros::Duration updateInterval = time_now -battery_time_last;
			ros::Time time_now = ros::Time::now();
			ros::Duration updateInterval = time_now - g_last_loop_time;

			g_last_loop_time =time_now;
			
				
			///----------------------------------------
			/// aggiorno lettura sensori
			///----------------------------------------
			float sec = updateInterval.toSec();
			bat.updateReadings(sec);
			///----------------------------------------
			
			// compilo i messaggi
			msg_battery_volt.data 			= bat.volts;
			msg_battery_ampere.data			= bat.current;
			msg_battery_watt.data 			= bat.watt;
			msg_battery_watthours.data 		= bat.whOut;
			msg_battery_charge.data 		= bat.charge;
			msg_battery_isPlugged.data  	= bat.isPlugged;
			msg_battery_isChargeCompleted.data 	= bat.isChargeCompleted;
			
			
			//pubblico 
			pub_battery_volt.publish(msg_battery_volt);
			pub_battery_ampere.publish(msg_battery_ampere);
			pub_battery_watt.publish(msg_battery_watt);
			pub_battery_watthours.publish(msg_battery_watthours);
			pub_battery_charge.publish(msg_battery_charge);
			pub_battery_isPlugged.publish(msg_battery_isPlugged);
			pub_battery_isChargeCompleted.publish(msg_battery_isChargeCompleted);
			
			
			
			
			
			if (time_now > battery_time_last + ros::Duration(battery_print_interval_sec ))
			{
				
				//----------------------------------------------------------------------------
				// gestione printout
				//----------------------------------------------------------------------------
				ROS_INFO("\r\nLiIon Bat:  %s |%s |%.2f%% | %.2fV (%.2f per cell) |Ampere:%.4f | shuntV %.4f | W:%.2f | Wh %.2f | @%d sec ( %d ', %d h)",
					msg_battery_isPlugged.data? "CAVO COLLEGATO ":" IN SCARICA ",
					bat.isChargeCompleted? "CARICA COMPLETA":"",
					msg_battery_charge.data, 
					msg_battery_volt.data,	
					msg_battery_volt.data/3, 
					bat.current, 
					bat.shuntV,
					msg_battery_watt.data, 
					msg_battery_watthours.data,uptimeSec() , uptimeSec()/60, 	uptimeSec()/3600);
				float ch = (float)(100.0f*bat.whOut / BATTERY_CAPACITY_WH );
				
				//ROS_INFO("\ncarica = %.2f%%, basata su monitor whOut %f / capacity di %f WH", ch, bat.whOut, BATTERY_CAPACITY_WH);	
				
				//----------------------------------------------------------------------------
				// gestione speech
				//----------------------------------------------------------------------------
				std::string s;
				if (bat.isPlugged) //presa collegata
				{
					if (!bat.isChargeCompleted)
					{
						s = "evviva la pappa";
						SPEECH_NOREPEAT(s);

					}	else
					{
						s ="speech_bat_charged";
						SPEECH_CODED_NOREPEAT(s);
					}
				}else //presa scollegata
				{
					s= "evviva sono libero di muovermi"; 

									
					if ((bat.charge < 70)&(bat.charge >= 30))
					{
						s ="Batteria carica al " + std::to_string(10*int(bat.charge/10)) + " per cento";
						SPEECH_NOREPEAT(s);
					
					}
					else if (bat.charge < 30)
					{
						s ="speech_bat_low";
						SPEECH_CODED_NOREPEAT(s);
					}	
					else if (bat.charge < 20)
					{
						s ="speech_bat_empty";
						SPEECH_CODED(s);
					}
				}
				 

				// check attacco stacco cavo di ricarica
				if (bat.blWarnChargingStatusChanged)
				{
					if (bat.isPlugged)
					{
						s = "speech_plug_on";
						SPEECH_CODED_NOREPEAT(s) ;
						
					}else
					{
						s = "speech_plug_off";
						SPEECH_CODED_NOREPEAT(s) ;
					}
					bat.blWarnChargingStatusChanged = false;
					
				}
				
							 
				//----------------------------------------------------------------------------
				// gestione LED di stato batteria
				//----------------------------------------------------------------------------
				ledStatusBatUpdate(&bat);



				battery_time_last =time_now;
			}
			


			
			
			ros::spinOnce();
			//g_last_loop_time = current_loop_time;
			ros::Duration(node_loop_interval_sec).sleep() ; // Sleep for n seconds
			
		}
		
		nodeShutdown();

	}


	#define ON_INTENSITY 255
	#define OFF_INTENSITY 0
	#define MAX_CURRENT 2
	
	void Ros_i2c::ledStatusBatUpdate( Battery *bat ){
		int tTonToff = 2000; //ms
		int tOn ; 		
		int tOff;


/*
		// testo prima se la carica è completata perchè ho notato che la corrente in questo caso può anche essere di pochi mA in scarica (>0)
		if (bat->isChargeCompleted)
		{
		
		}else // in carica o scarica
		{
			if (bat->current < 0) //presa collegata
			{
			}else // presa scollegata
			{
				
			}			
		}

*/


		if (bat->isPlugged) //presa collegata
		{
			// Spengo il led ROSSO
			gpioexp.pinMode(PIN_LED_R, OUTPUT);
			gpioexp.digitalWrite(PIN_LED_R, 0);			
			gpioexp.pinMode(PIN_LED_G, OUTPUT);
			gpioexp.digitalWrite(PIN_LED_G, 0);	
						
			
			if (!bat->isChargeCompleted)
			{
			
				// hearthbreate VERDE inversamente proporzionale alla corrente
				
				// Breathe an LED: 1000ms LOW, 500ms HIGH,
				// 500ms to rise from low to high
				// 250ms to fall from high to low

				// The timing parameters are in milliseconds, but they 
				// aren't 100% exact. The library will estimate to try to 
				// get them as close as possible. Play with the clock 
				// divider to maybe get more accurate timing.				
			
				tTonToff = 2000; //ms
				tOn =  (MAX_CURRENT - bat->current)/MAX_CURRENT *  tTonToff; // mappa 100 - 0 % in 0-2000
				tOff = tTonToff - tOn;

				// parametri          (byte pin, unsigned long tOn, unsigned long tOff, unsigned long rise, unsigned long fall, byte onInt, byte offInt, bool log
				gpioexp.pinMode(PIN_LED_G, ANALOG_OUTPUT);
				gpioexp.breathe(PIN_LED_G, tOn, tOff, 1000, 1000, ON_INTENSITY , OFF_INTENSITY);
				//printf("PIN_LED_G tOn %d, tOff   %d", tOn, tOff);
				
			}else
			{
				// carica completa -> luca dverde Fissa
				gpioexp.pinMode(PIN_LED_G, OUTPUT);
				gpioexp.digitalWrite(PIN_LED_G, 1);				
			}
			
		}else //in scarica
		{
			// Spengo il led VERDE
			gpioexp.pinMode(PIN_LED_G, OUTPUT);
			gpioexp.digitalWrite(PIN_LED_G, 0);	
						
			gpioexp.pinMode(PIN_LED_R, OUTPUT);
			gpioexp.digitalWrite(PIN_LED_G, 0);				

			
			if (bat->charge > 20)
			{
				// heartbreathe ROSSO inversamente proporzionale alla percentuale di carica
				tTonToff = 2000; //ms
				tOn = (int)( (float)tTonToff*(  1.0 - (float)bat->charge/100.0  )  ); // mappa 100 - 0 % in 0-2000
				tOff = tTonToff - tOn;
				gpioexp.pinMode(PIN_LED_R, ANALOG_OUTPUT);
				// parametri          ( pin, tOn, tOff, rise, fall, onInt,  offInt, bool log
				gpioexp.breathe(PIN_LED_R, tOn, tOff, 1500, 1500, ON_INTENSITY , OFF_INTENSITY);
				//gpioexp.blink(PIN_LED_R, tOn, tOff,  ON_INTENSITY , OFF_INTENSITY);
				//printf("PIN_LED_R tOn %d, tOff   %d", tOn, tOff);
			}
			else{  // batteria scarica -> led Rosso Fisso
			
				// carica completa -> luca dverde Fissa		
				gpioexp.pinMode(PIN_LED_R, OUTPUT);
				gpioexp.digitalWrite(PIN_LED_R, 1);				
				
			}
		}
	
	}
	
	void Ros_i2c::nodeShutdown(){
		bat.saveCharge();	
		gpioexp.pinMode(PIN_LED_R, OUTPUT);
		gpioexp.digitalWrite(PIN_LED_R, 0);				
		gpioexp.pinMode(PIN_LED_G, OUTPUT);
		gpioexp.digitalWrite(PIN_LED_G, 0);				
		gpioexp.pinMode(PIN_LED_B, OUTPUT);
		gpioexp.digitalWrite(PIN_LED_B, 0);				
		gpioexp.reset(true);
	}



#endif















/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///	GLOBAL FUNCTIONS
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
#if 1


	/// ////////////////////////////////////////////////////////////////////////////////////////////	
	/// ////////////////////////////////////////////////////////////////////////////////////////////	
	/// H A R D W A R E  S E T U P 
	/// ////////////////////////////////////////////////////////////////////////////////////////////	
	/// ////////////////////////////////////////////////////////////////////////////////////////////	

	void setup_HW(){
	}

 



	void  initial_test() {
		int16_t value;
		float_t volt, ampere;
		printf("\nSome readings on INA219----");
		for (int i = 0; i < 5; i++)
		{
			printf("\nAD0 : %f Volt \t %f mA  ",ina219.getBusVoltage_V(), ina219.getCurrent_mA() );
			delay(300);
		}


		

		printf("\n\nStarting I2C ADC module test - Start---------\n");
		for (int i=0;i<10; i++) {
			volt = adc_read_battery_volt() ;
			ampere = adc_read_battery_ampere();		

			printf("\nLoad Supply: Volt: %f  , \t Ampere %f , \t Watt %f ",volt, ampere, volt*ampere);
			delay(500);
		}
		printf("\ni2c ADC module test - End------------\n\n");

	}

#endif











//----------------------------------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "rm_i2c");

	ros::NodeHandle nh("~");

	Ros_i2c node = Ros_i2c(nh);

	return 0;
}
