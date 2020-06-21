#!/usr/bin/env python3.6
import sys

import edgetpu.detection.engine
import PIL
import numpy as np
from cv_bridge import CvBridge, CvBridgeError # Funziona con Python 2.7. Con 3.6 ha problemi
bridge = CvBridge()
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo

import time

import rospy
import roslib
import yaml

roslib.load_manifest("edge_tpu")
from dnn_detect.msg import DetectedObject, DetectedObjectArray

#print(sys.path)
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')# in order to import cv2 under python3
import cv2
# Print version string
print("OpenCV version :  {0}".format(cv2.__version__))

#sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages') # append back in order to import rospy




#usage: image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")

'''
------------------------------------------------------------------------------------------------------
TOOLS
------------------------------------------------------------------------------------------------------
'''

'''
Black: \u001b[30m
Red: \u001b[31m
Green: \u001b[32m
Yellow: \u001b[33m
Blue: \u001b[34m
Magenta: \u001b[35m
Cyan: \u001b[36m
White: \u001b[37m
Reset: \u001b[0m

Sequenza per il controllo del colore del tessto:  Esc[XXm , dove
#Esc =  \033    in ottale oppure \e
 XX =
39: Default (usually green, white or light gray): echo -e "Default \e[39mDefault"
30: Black: echo -e "Default \e[30mBlack" (best combined with a background colour: echo -e "Default \e[30;107mBlack on white")
31: Red (don't use with green background)
32: Green
33: Yellow
34: Blue
35: Magenta/Purple
36: Cyan
37: Light Gray
90: Dark Gray
91: Light Red
92: Light Green
93: Light Yellow
94: Light Blue
95: Light Magenta/Pink
96: Light Cyan
97: White
'''


class color:
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'

# Python program to convert a list to string using join() function 

# Function to convert 
def listToString(s): 

    # initialize an empty string 
    str1 = " "

    # return string 
    return (str1.join(s)) 

# Driver code
#s = ['Geeks', 'for', 'Geeks'] 
#print(listToString(s)) 


class tpu_detector:
    blShowDetections =1
    cv_image =None     # immagine da elaborare acquisita via callback o direttamente da cv.capture
    blImageAvailable = False # flag immagine disponibile. Resettato dopo elaborazione TPU
    blImg_from_cv2 = False
    blPublishImg = False
    blShowDetections = True
    #cap = None
    topic_img_in =''
    topic_img_out=''
    
    
    def __init__(self, model, labels, threshold=0.5, device_path=None,_topic_img_in="/in",_topic_img_out="/out", _blShowDetections= 1, _blImg_from_cv2=False , _blPublishImg=False , _deviceId = 0 , _camera_info_url=""):
        print("------------------")
        print("sys.version:", sys.version)
        print("------------------")
        self.bridge = CvBridge()
        rospy.loginfo("Loading model {}".format(model))
        
        # passo i parametri alle variabili del nodo
        self.blImg_from_cv2 = _blImg_from_cv2
        self.blPublishImg= _blPublishImg
        self.blShowDetections= _blShowDetections
        self.topic_img_in = _topic_img_in
        self.topic_img_out = _topic_img_out
        self.deviceId = _deviceId
        self.camera_info_url = _camera_info_url
        #------------------------------------
        # Subscriptions
        #------------------------------------
        if self.blImg_from_cv2==True:
            print(color.MAGENTA+"IMMAGINI ACQUISITE DIRETTAMENTE DA OPENCV "+color.END)
            #self.image_sub = rospy.Subscriber(  "/camera/compressed", Image, self.callback)
            self.cap = cv2.VideoCapture(self.deviceId)
            ret = self.cap.set(3,320) 
            ret = self.cap.set(4,240)

            # Limito buffersize a 1  per ridurre il lag
            a=self.cap.get(cv2.CAP_PROP_BUFFERSIZE) # CV_CAP_PROP_BUFFERSIZE
            print('\nOPENCV Buffersize was', a) 
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE,1)
            a=self.cap.get(cv2.CAP_PROP_BUFFERSIZE) # CV_CAP_PROP_BUFFERSIZE
            print('OPENCV Buffersize is', a) 

            # exposure time
            a = self.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
            print('\nOPENCV CAP_PROP_AUTO_EXPOSURE  was', a) 
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) 
            a = self.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)
            print('OPENCV CAP_PROP_AUTO_EXPOSURE  is', a,"\n") 
             
            # Frames per second
            a = self.cap.get(cv2.CAP_PROP_FPS)
            print('\nOPENCV CAP_PROP_FPS  was', a) 
            self.cap.set(cv2.CAP_PROP_FPS, 60) 
            a = self.cap.get(cv2.CAP_PROP_FPS)            
            print('OPENCV CAP_PROP_FPS  is', a,"\n") 
            
            
            
            ''''
                        # exposure time
                        a = self.cap.get(cv2.CAP_PROP_EXPOSURE)
                        print('\nOPENCV CAP_PROP_EXPOSURE  was', a) 
                        self.cap.set(cv2.CAP_PROP_EXPOSURE, 20) 
                        a = self.cap.get(cv2.CAP_PROP_EXPOSURE)
                        print('\nOPENCV CAP_PROP_EXPOSURE  is', a) 
                        
                        # gain
                        a = self.cap.get(cv2.CAP_PROP_GAIN)
                        print('\nOPENCV CAP_PROP_GAIN  was', a) 
                        self.cap.set(cv2.CAP_PROP_GAIN, 0) 
                        a = self.cap.get(cv2.CAP_PROP_GAIN)
                        print('\nOPENCV CAP_PROP_GAIN  is', a) 
             
                        #subprocess.check_call("v4l2-ctl -d /dev/video0 -c exposure_absolute=40",shell=True)
            '''
            self.cameraInfo_pub = rospy.Publisher("camera_info", CameraInfo, queue_size=10)
            print("\nCamera Info URL:", self.camera_info_url)
            self.camera_info_msg = self.yaml_to_CameraInfo(self.camera_info_url)
        else:
            #self.image_sub = rospy.Subscriber("/camera/compressed", CompressedImage, self.cbk_camera,  queue_size=1)
            self.image_sub = rospy.Subscriber(self.topic_img_in, CompressedImage, self.cbk_camera,  queue_size=1)
            print(color.BLUE +"IMMAGINI DA MESSAGGIO ROS" + color.END)
            print("\nSubscribing to ", self.topic_img_in)
        #------------------------------------
        #carica la rete neurale
        #------------------------------------
        self.threshold = threshold
        rospy.loginfo("Soglia di riconoscimento {}".format(threshold))
        self.engine = edgetpu.detection.engine.DetectionEngine(model) #self.engine = edgetpu.detection.engine.DetectionEngine(model,device_path)
        rospy.loginfo("\n********\nEngine loaded\n*********")
        self.load_labels(labels)
        
        #------------------------------------
        # Publishers
        #------------------------------------
        # sostituiti con il mio tipo di messaggio
        if self.blPublishImg==True:
            #self.pub_img = rospy.Publisher( self.topic_img_out,CompressedImage , queue_size=1)  #CompressedImage|Image
            self.pub_img = rospy.Publisher( self.topic_img_out,Image , queue_size=1)  #CompressedImage|Image
        #self.detection_pub = rospy.Publisher( 'detections', Detection2DArray, queue_size=10)
        self.dnn_detection_pub = rospy.Publisher('/dnn_objects', DetectedObjectArray, queue_size=1)
        self.dnn_persondetection_pub = rospy.Publisher('/dnn_persons', DetectedObjectArray, queue_size=1)

 

    def load_labels(self, labels):
        print ("loading labels from: ", labels)
        with open(labels, 'r', encoding="utf-8") as f:
            pairs = (l.strip().split(maxsplit=1) for l in f.readlines())
            self.labels = dict((int(k), v) for k, v in pairs)
        print(self.labels)




    # pubblica l'immagine elaborata'
    def pub_cv2_imgCompressed(self, cv_image):
        #### Create CompressedIamge ####
        try:
            # OK Funziona
            msg_img = CompressedImage()
            msg_img.header.stamp = rospy.Time.now()
            msg_img.format = "jpeg"
            msg_img.data = np.array(cv2.imencode('.jpg', cv_image)[1]).tostring()
            self.pub_img.publish(msg_img)
                   
        except CvBridgeError as e:
            print(e)

    # pubblica l'immagine elaborata'
    def pub_cv2_imgRaw(self, cv_image):
        #### Create CompressedIamge ####
        try:
           
            br = CvBridge()
            self.pub_img.publish(br.cv2_to_imgmsg(cv_image))          
        except CvBridgeError as e:
            print(e)



    # draw the bounding box and label on the image
    # da https://github.com/google-coral/examples-camera/blob/master/opencv/detect.py
    def displayDetections(self, cv2_im, objs, labels, _blShowWindow):
        #print("image res (y x channels):", cv2_im.shape)
        #height, width  = cv2_im.shape 
        height, width, channels = cv2_im.shape
        for obj in objs:
            x0, y0, x1, y1 = obj.bounding_box.flatten().tolist()
            #print("Detect. Before scaling x :", x0,  x1 )
            x0, y0, x1, y1 = int(
                x0*width), int(y0*height), int(x1*width), int(y1*height)
            #print("Detect.                                      After scaling x0,x1 , xcenter:", x0,  x1 , (x0+x1)/2 )
            percent = int(100 * obj.score)
            label = '%d%% %s' % (percent, labels[obj.label_id])
            # coord = ' [x0 %d, y0 %d, x1 %d, y1 %d]' % ( x0, y0, x1, y1)
            
            # print(label, coord)
            centerX =  int((x0+x1)/2)
            centerY =  int((y0+y1)/2)
            
            

            #colore in base alla detection
            if obj.label_id==0:
                color = (0, 255,  0) # green
            else:
                color = (255, 50, 0) 
            #contorno alla detection            
            cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), color, 2)
            cv2_im = cv2.circle(cv2_im, ( centerX ,centerY ),3, (0, 0,255 ), 4)
            #testo della detection
            cv2_im = cv2.putText(cv2_im, label, (x0, y0+30),   cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                
            #linea rossa orizzontale lunga quanto la distanza dal centro
            cv2_im = cv2.line(cv2_im,  ( int( width/2.0 ), centerY), (centerX, centerY), (0, 0, 255), 1)
            #testo distanza dal centro
            label = '%d' % ( (int)(centerX-width/2.0))
            cv2_im = cv2.putText(cv2_im, label, ( int( width/2.0 ), centerY ), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0,255 ), 1)
        #rettangolo blu della zona considerata al centro
        center_zone_halfwidth=30
        a0 = int( width/2.0 -center_zone_halfwidth)
        a1 = int( width/2.0 +center_zone_halfwidth )
        cv2_im = cv2.rectangle(cv2_im, ( a0, 0), ( a1, height), ( 255,0, 0), 1)
        
        
        if _blShowWindow:
            cv2.imshow("edge_tpu result", cv2_im)
            cv2.waitKey(1) #necessario per visualizzare l'immagine
            


    ########################################################################
    ## Pubblica se presenti le detections che poi vengono sottoscritte dal follower
    ########################################################################
    def publishDnnDetections(self, results, cv_image, frame_id):
        # Creo il messaggio ROS in uscita
        #detections = Detection2DArray()
        dnn_detections=DetectedObjectArray()
        dnn_persondetections=DetectedObjectArray()
        now = rospy.get_rostime()

        for detection in results:

            top_left, bottom_right = detection.bounding_box
            min_x, min_y = top_left
            max_x, max_y = bottom_right

            #scaling da 0..1 alle dimensioni dell'immagine
            imheight, imwidth, _ = cv_image.shape # immagine a colori
            #imheight, imwidth = cv_image.shape # immagine B.N

            min_x *= imwidth
            max_x *= imwidth
            min_y *= imheight
            max_y *= imheight

            centre_x = (max_x+min_x)/2.0
            centre_y = (max_y+min_y)/2.0
            height = max_y-min_y
            width = max_x-min_x

            if height <= 0 or width <= 0:
                continue

            object = DetectedObject()
            object.class_name = self.labels[detection.label_id]
            object.confidence = detection.score
            object.x_min = min_x
            object.x_max = max_x
            object.y_min = min_y
            object.y_max = max_y

            dnn_detections.objects.append(object)


            # Print detected object
            if ((object.class_name =='persona') or (object.class_name =='person')):
                printcolor = color.GREEN
                dnn_persondetections.objects.append(object)
            else:
                 printcolor = color.BOLD
                 self.dnn_detection_pub.publish(dnn_detections)
                 
            #label = '%f%% %s' % (object.confidence, object.class_name)
            label = '\n %s %f%% %s %s' % (printcolor, object.confidence, object.class_name, color.END)
            coord = ' [w: %d,h: %d] [x: %d, y: %d]' % (
                int(object.x_max - object.x_min),
                int(object.y_max - object.y_min), 
                int((object.x_min +  object.x_max)/2), 
                int((object.y_min  +  object.y_max)/2)
                )

            print(label+ coord) #print(label, coord)



        # pubblico solo se presenti detections
        if len(results) > 0:
            self.dnn_detection_pub.publish(dnn_detections)

        if len(dnn_persondetections.objects) > 0:
            self.dnn_persondetection_pub.publish(dnn_persondetections)
       
    ########################################################################
    ## Image callback
    ## converte l'immagine pervenuta in self.cv_image e ne segnala la disponibilità
    ########################################################################
    def cbk_camera(self, data):
        if self.blImg_from_cv2==False:
            
            try:
                #cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
                #cv_image = self.bridge.imgmsg_to_cv2(data, CV_8UC3)

                # da ROS Compressed a Numpy Array
                np_arr = np.fromstring(data.data, np.uint8)
                # da Numpy Array a CV2
                self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                self.blImageAvailable = True
                
                
            except CvBridgeError as e:
                rospy.logerr(e)
                print('.ros data format:',data.format)




    def cv2_capture(self):
        try:
            ret, self.cv_image =self.cap.read() 
            self.blImageAvailable = True
            
        except KeyboardInterrupt:
            self.blImageAvailable = False
            print("Excepition in CV2 capture")
            #cv2.destroyAllWindows()


    def yaml_to_CameraInfo(self, yaml_fname):
        """
        Parse a yaml file containing camera calibration data (as produced by 
        rosrun camera_calibration cameracalibrator.py) into a 
        sensor_msgs/CameraInfo msg.
        
        Parameters
        ----------
        yaml_fname : str
            Path to yaml file containing camera calibration data

        Returns
        -------
        camera_info_msg : sensor_msgs.msg.CameraInfo
            A sensor_msgs.msg.CameraInfo message containing the camera calibration
            data
        """
        # Load data from file
        print("\nReading camera info YAML from:", yaml_fname)
        with open(yaml_fname, "r") as file_handle:
            calib_data = yaml.load(file_handle)
        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg

    def run_detections(self,noderate_hz):
        try:
           
            rate = rospy.Rate(noderate_hz) # 10hz  

 
            while not rospy.is_shutdown():

                if self.blImg_from_cv2==True:
                    self.cv2_capture()
                    self.cameraInfo_pub.publish(self.camera_info_msg)


                if self.blImageAvailable==True:
                    self.blImageAvailable==False

                    ########################################################################
                    ## cuore della detection
                    ########################################################################
                    results = self.engine.DetectWithImage(PIL.Image.fromarray(
                        self.cv_image), top_k=1, threshold=self.threshold, keep_aspect_ratio=True, relative_coord=True)
                    ########################################################################
                    #print(results)
                    
                    # pubblica il messaggio ROS se ci sono detections        
                    self.publishDnnDetections(results, self.cv_image, "usbcam_link")

                    # visualizza le detection.  
                    self.displayDetections(self.cv_image, results, self.labels, self.blShowDetections)

                    # Pubblica l'immagine con le detections se abilitato
                    if self.blPublishImg:
                        #self.pub_cv2_imgCompressed(self.cv_image)
                        self.pub_cv2_imgRaw(self.cv_image)
                rate.sleep()

            # When everything done, release the capture
            self.cap.release()
            cv2.destroyAllWindows()
        except KeyboardInterrupt:
            print("Shutting down")
            cv2.destroyAllWindows()


def main( args):

    rospy.init_node('detect', anonymous=True)

    model_path = rospy.get_param('~model_path', default='/home/pi/ros/src/edgetpu_ros/models/mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite')
    label_path = rospy.get_param('~label_path', default='/home/pi/ros/src/edgetpu_ros/models/coco_labels.txt')
    threshold = rospy.get_param('~threshold', default=0.5)
    device_path = rospy.get_param('~device_path', default=None)
    blShowDetections = rospy.get_param('~show_detections', default=1)
    rate = rospy.get_param('~rate', default=10)
    
    #se true non sottoscrive immagine ros ma acquisisce direttamente da opencv
    blImg_from_cv2 = rospy.get_param('~img_from_cv2', default=False)
    if blImg_from_cv2:
        # se acquisisco da OpenCV carico il file YAML della webcam
        camera_info_url = rospy.get_param('~camera_info_url', default="").strip()
        
     #se true pubblice l'immagine con le detection
    blPublishImg = rospy.get_param('~publish_img', default=False)

    topic_img_in = rospy.get_param('~topic_img_in', default='/camera/compressed')
    topic_img_out = rospy.get_param('~topic_img_out', default='/robot/usbcam/image_raw')
    deviceId = rospy.get_param('~device_id', default='0')
   
    if blShowDetections == 0:
        print("Opencv Detection Window disabled")

    detector = tpu_detector(model_path, label_path, threshold, device_path,topic_img_in, topic_img_out, blShowDetections, blImg_from_cv2, blPublishImg ,deviceId, camera_info_url)
    detector.run_detections(rate)
    
    #rospy.spin()


if __name__ == "__main__":
    main( sys.argv)
