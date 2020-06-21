/**
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
 
// compila:		catkin_make -DCATKIN_WHITELIST_PACKAGES="rm_speech"
// run:			rosrun rm_speech rm_speech_node
// test manuale: espeak -v mb-it3 -a30 -s190 -p40 'ciao vinicia' 2 --stdout | aplay -D 'default'
 
#include <speech.hpp>
//#include "espeak/speak_lib.h"
//#include "speech.hpp"



int g_node_rate=1;
int g_speech_volume;
int g_speech_rate;
int g_speech_pitch;
int g_speech_range;
std::string g_speech_voice;
std::string g_wellcome_speech;
std::string	g_aplay_device;




int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "rm_speech");
 
 	// inizializzo la funzione random 	 
 	srand (time(NULL));
 	
    // Get the handle to the ROS node
    ros::NodeHandle node;

    // Start the worker node
    RPSpeechNode worker_node(node);
}


RPSpeechNode::RPSpeechNode(ros::NodeHandle& node) :  node(node) {
	// Initialize display message subscriber
	sub_chatter = node.subscribe("/chatter", 1, &RPSpeechNode::messageCallback, this);

	sub_speech_once = node.subscribe("/speech_once", 1, &RPSpeechNode::cbk_speech_once, this);

	sub_speech_coded = node.subscribe("/speech_coded", 1, &RPSpeechNode::cbk_speech_coded, this);


	/// ///////////////////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////
	///								
	///	LOAD PARAMETERS					
	///								
	/// ///////////////////////////////////////////////////////////////
	/// ////////////////////////////////////////////////////////////////
	#if 1

		ros::NodeHandle nh("~");
		
	
		///Loading parameters --------------------------------------------------------------	
		nh.param<std::string>("aplay_device", g_aplay_device,"default");
		nh.param("node_rate", g_node_rate,3);
		nh.param("speech_rate", g_speech_rate, 150); //range consigliato 150 -190
		nh.param("speech_pitch", g_speech_pitch, 50); // tra 10 e 90
		nh.param("speech_range", g_speech_range, 90);
		nh.param("speech_volume", g_speech_volume, 10); // -a tra 0 e 20
		nh.param<std::string>("g_speech_voice", g_speech_voice, "mb-it3" );
		nh.param<std::string>("wellcome_speech", g_wellcome_speech, "ciao ciao sono pronto");

		
		
		//--------------------------------------------------------------------------------
		// carica le frasi codificate
		//--------------------------------------------------------------------------------
	
		// Carico l'elenco delle frasi in speech_coded_list (di tipo XmlRpc::XmlRpcValue)
		if( !nh.getParam("speech_coded_list", speech_coded_list) )
			ROS_ERROR("parametro speech_coded_list non trovato...");
		ROS_ASSERT(speech_coded_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
		ROS_INFO("Numero di frasi codificate in speech_coded_list: %i",speech_coded_list.size());
		
		// Per ciascuna carico la lista delle possibili utterance
		std::string curr_speech_code;
		for (int32_t i = 0; i < speech_coded_list.size(); ++i) 
		{
			ROS_ASSERT(i < MAX_SPEECH_CODES);
			
			// speech_code corrente
			curr_speech_code = static_cast<std::string>(speech_coded_list[i]);
		  	ROS_INFO("Processing speech_coded: %s -------------------",curr_speech_code.c_str());
			


			//carico il parametro contenente la lista delle frasi di curr_speech_code  in curr_speech_phrases_xml
			XmlRpc::XmlRpcValue curr_speech_phrases_xml; //lista
			if( !nh.getParam(curr_speech_code,curr_speech_phrases_xml ) )
				ROS_ERROR("\tFrasi non trovate per speech_code: %s", curr_speech_code.c_str());
			ROS_ASSERT(curr_speech_phrases_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
			
			//carico la lista frasi
			speech_codes_xml[i] = curr_speech_phrases_xml;

		  	
		  	//ROS_INFO("\t %s - %d alternatives: %s",curr_speech_code.c_str() ,curr_speech_phrases_xml.size(),   curr_speech_phrases.c_str() );
		  	ROS_INFO("\t %s - %d alternatives",curr_speech_code.c_str() ,curr_speech_phrases_xml.size() );
			
			// faccio il parsing del contenuto del parametro per metterle in  speech_codes[i , 0...n-1 ]
			/*
			for (int32_t j = 0; j <speech_codes_n[i] ; ++i) 
			{	
				ROS_ASSERT(j < MAX_SPEECH_ALTERNATIVES);
		  		std::string s = static_cast<std::string>(curr_speech_phrases_xml[j]);
		  		speech_codes[i , j ] = s;
		  		ROS_INFO("\tLoaded @ %d  : %s", j, speech_codes[i , j ].c_str());
		  	
			}
			* */
		}
		ROS_INFO("End Speech codes-----------------------------------------------------------");
		//---------------------------------------------------------------------------------------
		

		
	#endif


	// inizializzazione 
	//espeak_SetParameter(espeakCAPITALS,  3, 0);

	//espeak_SetParameter(espeakRANGE,  g_speech_range, 0);			
	//espeak_SetVoiceByName(g_speech_voice.c_str());

	// Get the parameters from the parameter server
	//getOverridableParameters();

    
	speech_coded(0);
	ros::Rate rate(g_node_rate);
	
	while (ros::ok())
	{
		if (blPendingSpeech)
		{
			speechMessage(strPendingSpeech);
		}

		ros::spinOnce();
		rate.sleep();
	}
};

int RPSpeechNode::randomInt(int min, int max){
	
	if (min < max)
	{
		int range = max - min;
		int r = min +( rand() % range );
		return r;	
	}else
	{
		return 0;
	}

} 

//assegna solo la frase in message e imposta il flag blPendingSpeech=true
void RPSpeechNode::messageCallback(const std_msgs::String::ConstPtr& input_message)
{
	std::string s  = input_message->data;
	speech(s);
	ROS_INFO("Received Voice msg: [%s]", s.c_str());
}

/// Non ripete la stringa di testo ricevuta
void RPSpeechNode::cbk_speech_once(const std_msgs::String::ConstPtr& input_message)
{
	std::string s =input_message->data;
	speech_once(s);
	
	ROS_INFO("Received cbk_speech_once msg: [%s]", s.c_str());
}

//-------------------------------------------------------------------
// imposta lastutterance alla frase da dire e il flag blPendingSpeech=true
// espeak viene chiamato nel loop principale
//--------------------------------------------------------------------
void RPSpeechNode::speech(std::string s){
	message_mutex.lock();
	{			 
		strPendingSpeech = s;
	}
	blPendingSpeech = true;		
	message_mutex.unlock();
	
}

void RPSpeechNode::speech_once(std::string s){
	if(! s.compare(lastUtterance) == 0)   
	{
		speech(s);
	}else
	{
		blPendingSpeech = false;		
	}
}

void RPSpeechNode::speech_coded(int i){
	XmlRpc::XmlRpcValue	sc = speech_codes_xml[i];  //lista frasi alternative
	
	// pesco a caso 
	int j = randomInt( 0, sc.size() );
	std::string s = static_cast<std::string>(sc[j]);
	speech_once(s);
}

/// Riproduce un messaggio codificato. Se ci sono più frasi con lo stesso codice ne pronuncia una a caso
void RPSpeechNode::cbk_speech_coded(const std_msgs::String::ConstPtr& input_message)
{
	std::string s; 
	
	// individuo l'indice del messaggio codificato
	
	for (int32_t i = 0; i < speech_coded_list.size(); ++i) 
	{
		s = static_cast<std::string>(speech_coded_list[i]);
		if (input_message->data.compare(s) == 0)
		{
			speech_coded(i);	
		}
	}
	
}
void RPSpeechNode::speechMessage(std::string s)
{
	message_mutex.lock();
	{
		blPendingSpeech = false;
		std::string strCmd    = "espeak ";
		std::string strVoice  = " -v " + g_speech_voice;
		std::string strVolume = " -a " + std::to_string(g_speech_volume);
		std::string strSpeed  = " -s " + std::to_string(g_speech_rate);
		std::string strPitch  = " -p " + std::to_string(g_speech_pitch); 
		std::string strCmdEnd = std::string("' 2 --stdout | aplay -D '")+ g_aplay_device +std::string("'");
		//std::string command = "espeak -v mb-it3 -a20 -s150 '" + message +"' 2>/dev/null";
		//std::string command = "espeak -v mb-it3 -a20 -s150 '" + message +"' 2 --stdout | aplay -D 'default'";
		std::string command = strCmd + strVoice + strVolume +strSpeed + strPitch +" -m '" + s +strCmdEnd;

		ROS_INFO("Run command: : %s",command.c_str());
		system(command.c_str());
		//espeak_Synth(message.c_str(), message.length() + 1, 0, POS_CHARACTER, 0, espeakCHARS_AUTO, NULL, NULL);
		//espeak_Synchronize();
		lastUtterance = s;
	}
	message_mutex.unlock();
}
