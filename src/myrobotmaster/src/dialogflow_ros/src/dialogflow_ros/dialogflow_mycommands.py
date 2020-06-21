#!/usr/bin/env python3


# mie annotazioni-------------------------------------------
# se vengono Warnings da ALSALIB commenta le seguenti linee in /usr/share/alsa/alsaconf
###pcm.rear cards.pcm.rear
###pcm.center_lfe cards.pcm.center_lfe
###pcm.side cards.pcm.side
###pcm.surround21 cards.pcm.surround21
###pcm.surround40 cards.pcm.surround40
###pcm.surround41 cards.pcm.surround41
###pcm.surround50 cards.pcm.surround50
###pcm.surround51 cards.pcm.surround51
###pcm.surround71 cards.pcm.surround71

#-----------------------------------------------------------


# Dialogflow

import sys
import signal #gestione ctrl-c

#sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')# in order to import cv2 under python3
sys.path.append('/home/luca/ros/src/dialogflow_ros/src/dialogflow_ros/utils')#per importare utils
import utils


#import dialogflow_v2beta1
import dialogflow_v2beta1
from dialogflow_v2beta1.types import Context, EventInput, InputAudioConfig, \
    OutputAudioConfig, QueryInput, QueryParameters, \
    StreamingDetectIntentRequest, TextInput
from dialogflow_v2beta1.gapic.enums import AudioEncoding, OutputAudioEncoding
import google.api_core.exceptions




import time
from uuid import uuid4
from yaml import load, YAMLError

# ROS
import rospy
import rospkg
from std_msgs.msg import String , Float64, Bool
from geometry_msgs.msg import Twist

# ROS-Dialogflow
from dialogflow_ros.msg import *

# Use to convert Struct messages to JSON
# from google.protobuf.json_format import MessageToJson
sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages')# in order to import cv2 under python3
# ========================================= #
#           ROS Utility Functions           #
# ========================================= #

def isNumber(s):
    try:
        float(s)
        return True
    except TypeError:
        print("Stai cercando di convertire questo in numero:", s)
        return False

    except ValueError:
        return False

class DialogflowCommandExecutor(object):
    def __init__(self, language_code='it-IT', last_contexts=None):
        """Initialize all params and load data"""
        """ Constants and params """

        self.DEBUG = rospy.get_param('/dialogflow_client/debug', False)

        # Register Ctrl-C sigint
        signal.signal(signal.SIGINT, self._signal_handler)

        """ Dialogflow setup """
        # Get hints/clues
        rp = rospkg.RosPack()
        file_dir = rp.get_path('dialogflow_ros') + '/config/context.yaml'
        with open(file_dir, 'r') as f:
            try:
                self.phrase_hints = load(f)
            except YAMLError:
                rospy.logwarn("DF_CLIENT: Unable to open phrase hints yaml file!")
                self.phrase_hints = []

        # Dialogflow params
        project_id = rospy.get_param('/dialogflow_client/project_id', 'robot-15de1')
        session_id = str(uuid4())  # Random
        self._language_code = language_code
        #self.last_contexts = last_contexts if last_contexts else []



        """ ROS Setup """

        #rospy.logdebug("DF_CLIENT: Last Contexts: {}".format(self.last_contexts))
 
 
        # subscribe tutti i messaggi di cui devo poter dare info a voce 
        self.bat_charge = 0
        rospy.Subscriber('/bat_charge', Float64, self.cbk_bat_charge)

        # qui tutti i comandi eseguibili a voce
        self.pub_chatter = rospy.Publisher('/chatter',String,queue_size=10)
        self.pub_faretto = rospy.Publisher("/faretto", Bool, queue_size=1)
        self.pub_laser = rospy.Publisher("/laser", Bool, queue_size=1)
        self.pub_start_followme = rospy.Publisher("/start_followme", Bool, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel",Twist, queue_size=1)
        self.pub_targetPose = rospy.Publisher("/target_pose",Twist, queue_size=1)

    # ========================================= #
    #           speech macro                    #
    # ========================================= #
    def speech(self, msg):
        self.pub_chatter.publish(msg)

    # ========================================= #
    #           Battery                         #
    # ========================================= #
    def cbk_bat_charge(self, msg):
        self.bat_charge = msg.data


    # ==================================== #
    #           Utility Functions          #
    # ==================================== #

    def _signal_handler(self, signal, frame):
        rospy.logwarn("\nDF_CLIENT: SIGINT caught!")
        self.exit()



    # ========================================= #
    #           Command parser                  #
    # ========================================= #
    
    def command_parser(self, intent, parameters):
        # fai riferimento alle voci dell'entity @command in DialogFlow
        # https://cloud.google.com/dialogflow/docs/reference/rpc/google.cloud.dialogflow.v2beta1#queryparameters
        pName =""
        pVal = ""
        devName=""
        devVal= False

        #--------------------------------------------------------------------------        
        if intent=='cmd_set':
            #parametri associati all'intent
            devName=""
            devVal =False

            print('####  Comando: {}'.format(intent))
            for p in parameters:
                pName = p
                pVal = parameters[p]
                #print( "p={}: parameters[p]={}\n\t".format(pName,pVal ))
                
                if (pName=="device"):
                    devName = pVal
                if (pName=="status_on_off"):
                    if parameters[p]=="on":
                        devVal = True
                    else:
                        devVal = False


            # a questo punto ho le info per generare il comando
            if devName=="faretto":
                self.pub_faretto.publish(devVal)
            elif devName=="laser":
                self.pub_laser.publish(devVal)
            print("_______Device:{} set:{}___________".format(devName,devVal))

        #--------------------------------------------------------------------------        
        elif intent=='cmd_move':
            #parametri associati all'intent
            direction= 1
            angle=0.0
            unitFactor = 0.01
            displacement =0.0

            print('Comando: {}'.format(intent))
            for p in parameters:
                pName = p
                pVal = parameters[p]
                print( "pName:{}, pVal:{}\n\t".format(pName,pVal ))
                
                if (pName=="direction"):    # avanti , indietro, destra , sinistra
                    if pVal =="avanti":
                        direction = 1
                    else:
                        direction = -1
                if (pName=="displacement"):
                    if isNumber(pVal):
                        displacement = float(pVal)
                    else:
                        displacement = 0.0
                if (pName=="unit_length"):
                    if pVal =="m":
                        unitFactor = 1
                    elif pVal == "dm":
                         unitFactor = 0.1
                    elif pVal == "cm":
                         unitFactor = 0.01
                    elif pVal == "mm":
                         unitFactor = 0.001                        
            msg_targetPose = Twist()
            msg_targetPose.linear.x = direction * displacement * unitFactor
            msg_targetPose.linear.y = 0.0
            msg_targetPose.linear.z = 0.0
            msg_targetPose.angular.x = 0.0
            msg_targetPose.angular.y = 0.0
            msg_targetPose.angular.z = 0.0
            #ruota di x gradi. Richiede nodo rm_cmd_move su rockpi4
            self.pub_targetPose.publish(msg_targetPose)
        #--------------------------------------------------------------------------        
        elif intent=='cmd_rotate':
            #parametri associati all'intent
            direction= 1
            angle=0.0

            print('Comando: {}'.format(intent))
            for p in parameters:
                pName = p
                pVal = parameters[p]
                #print( "p={}: parameters[p]={}\n\t".format(pName,pVal ))
                
                if (pName=="direction"):
                    if pVal =="destra":
                        direction = -1
                    else:
                        direction = 1
                if (pName=="angle"):
                    if isNumber(pVal):
                        print("\n =====> pVal:", pVal)
                        angle = float(pVal)
                        if abs(angle) > 360:
                            speech("sono troppi")
                            
                    else:
                        angle = 0.0

            msg_targetPose = Twist()
            msg_targetPose.linear.x = 0.0
            msg_targetPose.linear.y = 0.0
            msg_targetPose.linear.z = 0.0
            msg_targetPose.angular.x = 0.0
            msg_targetPose.angular.y = 0.0
            msg_targetPose.angular.z = direction * angle
            #ruota di x gradi. Richiede nodo rm_cmd_move su rockpi4
            self.pub_targetPose.publish(msg_targetPose)
        #--------------------------------------------------------------------------        
        elif intent=='cmd_stop':
            print('Comando: {}'.format(intent))
            msg_targetPose = Twist()
            msg_targetPose.linear.x = 0.0
            msg_targetPose.linear.y = 0.0
            msg_targetPose.linear.z = 0.0
            msg_targetPose.angular.x = 0.0
            msg_targetPose.angular.y = 0.0
            msg_targetPose.angular.z = 0.0
            #ruota di x gradi. Richiede nodo rm_cmd_move su rockpi4
            self.pub_targetPose.publish(msg_targetPose)        
        #--------------------------------------------------------------------------        
        elif intent=='cmd_follower':
            print('Comando: {}'.format(intent))
            self.pub_start_followme.publish(True)


        #--------------------------------------------------------------------------        
        elif intent=='get':
            print('Comando: {}'.format(intent))


        #--------------------------------------------------------------------------        
        elif intent=='cmd_explore':
            print('Comando: {}'.format(intent))


        #--------------------------------------------------------------------------        
        elif intent=='cmd_surveil':
            print('Comando: {}'.format(intent))


        #--------------------------------------------------------------------------        
        elif intent=='cmd_unset':
            print('Comando: {}'.format(intent))


        #--------------------------------------------------------------------------        
        else:
            print('Comando non gestito: {}'.format(intent))
            



 
 







    
