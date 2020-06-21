/**
 * @class      RPSpeechNode
 *
 * @brief      Robot photographer's text-to-speech synthesis node, which vocalizes the input
 *             status messages using the Espeak library.
 *
 * @copyright  Manfredas Zabarauskas, 2013. All rights reserved. This project is released under
 *             CC BY-NC-SA (Creative Commons Attribution-NonCommercial-ShareAlike) license.
 */
#ifndef SPEECH_HPP_
#define SPEECH_HPP_

// ROS includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <time.h>

//#include <random.h>
// Boost includes
#include <boost/thread/mutex.hpp>
//#include "speak_lib.h" 
     	const int  MAX_SPEECH_CODES=50;
    	const int MAX_SPEECH_ALTERNATIVES=10;
class RPSpeechNode
{
    private:

        ros::NodeHandle& node;                      /**< Node handle.                            */

        ros::Subscriber sub_chatter;         /**< Message subscriber.                     */
		ros::Subscriber sub_speech_once; 
 		ros::Subscriber sub_speech_coded; 
 		
        std::string strPendingSpeech;                        /**< Message to be said.                     */
        
        XmlRpc::XmlRpcValue speech_coded_list; 		/* lista codici*/      
        XmlRpc::XmlRpcValue speech_codes_xml[MAX_SPEECH_CODES];
        
        //std::array<std::string, MAX_SPEECH_CODES,MAX_SPEECH_ALTERNATIVES-1 >  speech_codes  ; 
        // assumes using std::vector for brevity
		//std::vector<std::vector<std::string>> speech_codes( std::vector<std::string>(MAX_SPEECH_ALTERNATIVES));
		std::vector<std::vector<std::string>> speech_codes;
		// poi fare: speech_codes.resize(MAX_SPEECH_CODES, std::vector<std::string>(MAX_SPEECH_ALTERNATIVES, "" ));
		
		
        int speech_codes_n[MAX_SPEECH_CODES]; 		/* per ogni elemento della lista registra il numero di alternative */
        
        
        boost::mutex message_mutex;                 /**< Message's mutex.                        */

        volatile bool blPendingSpeech;                  /**< Flag whether the message is received
          
        /**
         * Callback for the message to vocalize.
         * @param message Input message.
         */
        void messageCallback(const std_msgs::String::ConstPtr& message);
        void cbk_speech_once(const std_msgs::String::ConstPtr& message);
        void cbk_speech_coded(const std_msgs::String::ConstPtr& message);
		std::string lastUtterance; //salva l'ultima frase per cbk_speech_once
        /**
         * Vocalizes the current message.
         */
        void speechMessage(std::string s);
        
		int randomInt(int min, int max);
		void speech_coded(int i);
		void speech_once(std::string s);
		void speech(std::string s);
		
    public:
        /**
         * Default speech node's constructor.
         * @param node Handle to ROS node.
         */
        RPSpeechNode(ros::NodeHandle& node);
 		int g_speech_rate;
        int g_speech_pitch;
        int g_speech_range;
		std::string g_wellcome_speech;
		std::string g_speech_voice;
        
};

#endif /* SPEECH_HPP_ */
