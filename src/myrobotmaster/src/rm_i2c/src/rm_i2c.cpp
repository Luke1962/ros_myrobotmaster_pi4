/*

 * 
* 
* build : catkin_make  -DCATKIN_WHITELIST_PACKAGES="rm_i2c"
* run: sudo .rmaa.sh 
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


/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
/// BATTERIA	
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
#if 1
	enum speechBat_e
	{
		UNKNOWN,
		EMPTY,
		DISCHARGED,
		DISCHARGING,
		CHARGING,
		CHARGED
	};

	class Battery {
		public:
			Battery(mraa::I2c *i2c_bus);
			float volts;
			float current;
			float shuntV;
			float watt;
			float whOut;			
			float charge; //livello di carica in %
			bool isCharging; //true quando la corrente < 0
			bool isChargeCompleted; //true quando la corrente di carica va sotto una certa soglia
			bool blWarnChargingStatusChanged; //passa a true se cambia lo stato carica-scarica
			void updateReadings(float  update_interval_sec);
			void calibrateSensor(float maxAmpere);
			uint8_t volt2charge(float vbat,  uint16_t minVoltage, uint16_t maxVoltage);
			void saveCharge()	;
			void restoreCharge();
			speechBat_e status;
		private:
			mraa::I2c *i2c;
			float volt2charge(float v );
	};
	
	
	#define BATTERY_CAPACITY_WH  41.0f // Energia quando la batteria è completamente carica (3.7V x 3s x 1.8 Ah per battery x 4p = 80Wh circa) 
	
	#define MAX_EXPECTED_CURRENT_A   2.0 
 
	#define SHUNT_RESISTENCE 0.1f //(scritta R100)
	
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
		if (fabs(current)< 0.005){
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
		bool prevChargingStatus = isCharging;
		isCharging = (current <= 0)? true: false;
		
		if (prevChargingStatus != isCharging)
		{
			// presa attaccata o staccata, alzo il flag
			blWarnChargingStatusChanged =true;
		}
	}
	
	Battery::Battery(mraa::I2c *i2c_bus){
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

	#define FILE_CHARGE "/home/pi/ros/src/myrobotmaster/src/rm_i2c/charge_wh.txt"
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
/// SPEECH STATUS BATTERIA	
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
speechBat_e lastWarningBatteryStatus;

void speech(std::string msg) {
 std_msgs::String msg_chatter;
  msg_chatter.data = msg;
  pub_chatter.publish(msg_chatter);
  //ROS_INFO("[SPEECH] %s", msg.c_str());
}

//avvisa una sola volta 
void speechOnce( speechBat_e  batterystatus){

	if (batterystatus != lastWarningBatteryStatus) {

		switch (batterystatus) {
		

		case UNKNOWN:
			speech("stato batteria sconosciuto");
			ROS_INFO("SPEEECH: stato batteria sconosciuto");		
			break;
			
		case CHARGING:
			speech("Grazie per ricaricarmi");
			ROS_INFO("SPEEECH:  Grazie per ricaricarmi");
			break;

		case DISCHARGING:
			speech("ora vado a batteria. Stacca il cavo così posso muovermi");
			ROS_INFO("SPEEECH: okei ora posso muovermi");		  	
			break;
		
		case CHARGED:
			speech("sono carico. puoi staccare la presa");
			ROS_INFO("SPEEECH: sono carico. puoi staccare la presa");    	
			break;
			
		case DISCHARGED:
			speech("sono scarico. è ora di ricaricarmi");
			ROS_INFO("SPEEECH: sono scarico. è ora di ricaricarmi");   	
			break;
			
		case EMPTY:
			speech("BATTERIA COMPLETAMENTE SCARICA");
			ROS_INFO("SPEEECH:!!! BATTERIA COMPLETAMENTE SCARICA!!!");   	
			break;
			
 	
		default:
			break;
		}

		lastWarningBatteryStatus = batterystatus;
	}
}



/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///	R O S
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
#if 1
	//ros::NodeHandle nh; deve essere dichiarato dopo ros:::Init()

	double rate = 10.0; // 5Hz main loop rate

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

	void HWshutdown(int i){
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





	/// ///////////////////////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////
	/// GESTIONE KEYBOARD INPUT NON BLOCCANTE	
	/// ///////////////////////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////
	#if 1	
		#include <stdlib.h>
		#include <string.h>
		#include <unistd.h>
		#include <sys/select.h>
		#include <termios.h>
		
		struct termios orig_termios;
		
		void reset_terminal_mode()
		{
			tcsetattr(0, TCSANOW, &orig_termios);
		}
		
		void set_conio_terminal_mode()
		{
			struct termios new_termios;
		
			/* take two copies - one for now, one for later */
			tcgetattr(0, &orig_termios);
			memcpy(&new_termios, &orig_termios, sizeof(new_termios));
		
			/* register cleanup handler, and set the new terminal mode */
			atexit(reset_terminal_mode);
			cfmakeraw(&new_termios);
			tcsetattr(0, TCSANOW, &new_termios);
		}
		
		int kbhit()
		{
			struct timeval tv = { 0L, 0L };
			fd_set fds;
			FD_ZERO(&fds);
			FD_SET(0, &fds);
			return select(1, &fds, NULL, NULL, &tv);
		}
		
		int getch()
		{
			int r;
			unsigned char c;
			if ((r = read(0, &c, sizeof(c))) < 0) {
				return r;
			} else {
				return c;
			}
		}
		
		// utilizzo:
		// setup:    set_conio_terminal_mode();
		//loop:   while (!kbhit()) {	        /* do some work */	    }
	#endif	

static std::string lastSpeech;
#define SPEECH(S) 	msg_chatter.data = S ; 	pub_chatter.publish(msg_chatter); lastSpeech=S;
#define SPEECH_NOREPEAT(S) if(! S.compare(lastSpeech) == 0) {SPEECH(S); }





/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///	M A I N
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
int main(int argc, char** argv){
	ros::init(argc, argv, "rm_i2c");

	ROS_INFO("####### rm_i2c ############");

	#if 1  ///S E T U P 	 HW, ROS
	
		/// ////////////////////////////////////////////////////////////////////////////////////////////	
		/// ////////////////////////////////////////////////////////////////////////////////////////////	
		/// R O S  S E T U P 
		/// ////////////////////////////////////////////////////////////////////////////////////////////	
		/// ////////////////////////////////////////////////////////////////////////////////////////////	
		ros::init(argc, argv, NODENAME); ///must be called before creating a node handle
		ros::NodeHandle nh; 
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

		int node_rate_hz=2;
		_nh.getParam("node_rate_hz",node_rate_hz);
		float node_loop_interval_sec=(float)1/node_rate_hz;
		ROS_INFO("Node Hz %d ( interval: %f sec)",node_rate_hz,node_loop_interval_sec);

		// mraa setup ------------------------------
		ROS_INFO("Inizializzazione mraa....");
		#define I2C_BUS 0
		mraa::I2c i2c7(I2C_BUS);		
		//uint8_t rx_tx_buf[MAX_BUFFER_LENGTH];
		
		
		ROS_INFO("INA219 inizialization...");
		
		//setup HW 
		i2c7.address(INA219_ADDRESS);	
		ina219.begin(&i2c7);
		ina219.setCalibration_32V_2A();
		
		


		ROS_INFO("Measuring voltage and current with INA219 ...");	
		Battery bat(&i2c7);	
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
		
		std_msgs::Bool msg_battery_ischarging;
		ros::Publisher pub_battery_ischarging = nh.advertise<std_msgs::Bool>("/bat_ischarging",1);
		msg_battery_ischarging.data = false;		


		std_msgs::Bool msg_battery_isChargeCompleted;
		ros::Publisher pub_battery_isChargeCompleted = nh.advertise<std_msgs::Bool>("/bat_ischargecompleted",1);
		msg_battery_isChargeCompleted.data = false;		

		std_msgs::String msg_chatter;
		ros::Publisher pub_chatter = nh.advertise<std_msgs::String>("/chatter", 1); // mia aggiunta
		
		///---------------------------------------------------------------
		/// Vars init
		///---------------------------------------------------------------
		/// inizializza il consumo ipotizzando un consumo costante fino a questo momento
		//initial_test();
		


		if (msg_battery_ischarging.data)
		{
			speechOnce(CHARGING);
		}else
		{
			speechOnce(DISCHARGING);
		}

		if(msg_battery_charge.data <20){
			msg_chatter.data="sono scarico";
			pub_chatter.publish(msg_chatter);
		}
		if(msg_battery_charge.data <10){
 			speechOnce(EMPTY);

		}		

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
	
	// imposta la keyboard per killare il nodo
	set_conio_terminal_mode();
	
	while(nh.ok() && !kbhit()){
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
		msg_battery_ischarging.data 	= bat.isCharging;
		msg_battery_isChargeCompleted.data 	= bat.isChargeCompleted;
		
		
		//pubblico 
		pub_battery_volt.publish(msg_battery_volt);
		pub_battery_ampere.publish(msg_battery_ampere);
		pub_battery_watt.publish(msg_battery_watt);
		pub_battery_watthours.publish(msg_battery_watthours);
		pub_battery_charge.publish(msg_battery_charge);
		pub_battery_ischarging.publish(msg_battery_ischarging);
		pub_battery_isChargeCompleted.publish(msg_battery_isChargeCompleted);
			
		if (time_now > battery_time_last + ros::Duration(battery_print_interval_sec ))
		{
			
			//stampo
			ROS_INFO("\r\nLiIon Bat:  %s |%s |%.2f%% | %.2fV (%.2f per cell) |Ampere:%.4f | shuntV %.4f | W:%.2f | Wh %.2f | @%d sec ( %d ', %d h)",
				msg_battery_ischarging.data? "In carica":"in SCARICA ",
				bat.isChargeCompleted? "CARICA COMPLETA":"",
				msg_battery_charge.data, 
				msg_battery_volt.data,	
				msg_battery_volt.data/3, 
				bat.current, 
				bat.shuntV,
				msg_battery_watt.data, 
				msg_battery_watthours.data,uptimeSec() , uptimeSec()/60, 	uptimeSec()/3600);
			float ch = (float)(100.0f*bat.whOut / BATTERY_CAPACITY_WH );
			
			ROS_INFO("\ncarica = %.2f%%, basata su monitor whOut %f / capacity di %f WH", ch, bat.whOut, BATTERY_CAPACITY_WH);	
	
			battery_time_last =time_now;
			
			/*
				if (!bat.isCharging)
				{
					if (bat.charge <20)
					{
						msg_chatter.data ="Sono scarico" ; 
						pub_chatter.publish(msg_chatter); 
				
					}
				}
			*/
			std::string s;
			speechOnce(bat.status);
			// check attacco stacco cavo di ricarica
			if (bat.blWarnChargingStatusChanged)
			{
				if (bat.isCharging)
				{
					s = "Evviva la pappa";
					SPEECH_NOREPEAT(s) ;
					
				}else
				{
					s = "Evviva sono libero di muovermi";
					SPEECH_NOREPEAT(s) ;
				}
				bat.blWarnChargingStatusChanged = false;
				
			}


		}
		


		
		
		ros::spinOnce();
		//g_last_loop_time = current_loop_time;
		ros::Duration(node_loop_interval_sec).sleep() ; // Sleep for n seconds
		
	}
	(void)getch(); /* consume the character */
	HWshutdown(0);
	bat.saveCharge();
}
