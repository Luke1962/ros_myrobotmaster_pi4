#!/usr/bin/env python3
import sys
#ubuntu on PC sys.path.append('/opt/ros/melodic/lib/python3.6/dist-packages')
sys.path.append('/usr/local/lib/python2.7/dist-packages') #rockpi4
sys.path.append('/usr/local/lib/python3.6/dist-packages') #rockpi4
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types

# da https://gitlab.cs.duke.edu/fengyu.xie/tech-check-model/blob/bb75d46f36340fc677490f3b8851856487aecb23/media/TranscribeAudio.py
from google.api_core.exceptions import OutOfRange
from google.api_core.exceptions import Unknown



import pyaudio
import queue
import rospy
from std_msgs.msg import String
# Audio recording parameters
SOUND_DEV_INDEX = 3
STREAMING_LIMIT = 10000
SAMPLE_RATE = 44100 #16000
CHUNK_SIZE = int(SAMPLE_RATE / 10)  # 100ms

RED = '\033[0;31m'
GREEN = '\033[0;32m'
YELLOW = '\033[0;33m'



class GspeechClient(object):
    def __init__(self):
        # Audio stream input setup
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = SAMPLE_RATE #16000
        self.CHUNK = CHUNK_SIZE #4096

        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(format=FORMAT, 
                                      channels=CHANNELS,
                                      rate=RATE, input=True,
                                      input_device_index = SOUND_DEV_INDEX,
                                      frames_per_buffer=self.CHUNK,
                                      # Run the audio stream asynchronously to fill the buffer object.
                                      # This is necessary so that the input device's buffer doesn't
                                      # overflow while the calling thread makes network requests, etc.
                                      stream_callback=self._get_data
                                    )
        self._buff = queue.Queue()  # Buffer to hold audio data
        self.closed = False

        # ROS Text Publisher
        text_topic = rospy.get_param('/text_topic', '/dialogflow_text')
        self.text_pub = rospy.Publisher(text_topic, String, queue_size=10)

    def _get_data(self, in_data, frame_count, time_info, status):
        """Daemon thread to continuously get audio data from the server and put
         it in a buffer.
        """
        # Uncomment this if you want to hear the audio being replayed.
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def _generator(self):
        """Generator function that continuously yields audio chunks from the buffer.
        Used to stream data to the Google Speech API Asynchronously.
        """
        while not self.closed:
            # Check first chunk of data
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Read in a stream till the end using a non-blocking get()
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except queue.Empty:
                    break

            yield b''.join(data)

    def _listen_print_loop(self, responses):
        """Iterates through server responses and prints them.
        The responses passed is a generator that will block until a response
        is provided by the server.
        Each response may contain multiple results, and each result may contain
        multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
        print only the transcription for the top alternative of the top result.
        """
        try:
            for response in responses:
                # If not a valid response, move on to next potential one
                if not response.results:
                    continue

                # The `results` list is consecutive. For streaming, we only care about
                # the first result being considered, since once it's `is_final`, it
                # moves on to considering the next utterance.
                result = response.results[0]
                if not result.alternatives:
                    continue

                # Display the transcription of the top alternative.
                transcript = result.alternatives[0].transcript
                transcript_unicode = transcript.encode('utf-8')

                # Parse the final utterance
                if result.is_final:
                    rospy.loginfo("Google Speech result: {}".format(result))
                    # Received data is Unicode, convert it to string
                    #transcript = transcript.encode('utf-8')
                    
                    # Strip the initial space, if any
                    if transcript.startswith(' '):  #if transcript.startswith(' '):
                        transcript = transcript[1:]
                    # Exit if needed
                    if transcript.lower() == 'exit':
                        self.shutdown()
                    # Send the rest of the sentence to topic
                    self.text_pub.publish(transcript)
        except OutOfRange:#exceeded time limit
            #this should never happen but in case it does ensure that the thread doesnt die
            print('exceeded time limit')
        except Unknown as e:
            print('speech to text transcription encountered the following error:',e) #not sure why this happens so just catch and print
        except Exception as e:
            if response.error.code:
                #google cloud speech returned an error
                errorCode=response.error.code 
                if errorCode==11:
                    #the radio stream gives the 5ish seconds before you make the request very rapidly at the beginning
                    #this causes audio to be sent to google too fast
                    #if this happens simply sleep for a little to let google catch up
                    print('too fast, slowing down')
                    sleep(0.8)
                else:
                    raise RuntimeError('[MP] unknown google speech error: '+str(response))
            else:
                #not a google error so just raise the original exception
                raise e


    def gspeech_client(self):
        """Creates the Google Speech API client, configures it, and sends/gets
        audio/text data for parsing.
        """
        language_code = 'it-IT'
        client = speech.SpeechClient()
        config = types.RecognitionConfig(
            encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=44100, #16000,
            language_code=language_code)
        streaming_config = types.StreamingRecognitionConfig(
            config=config,
            interim_results=True)
        # Hack from Google Speech Python docs, very pythonic c:
        requests = (types.StreamingRecognizeRequest(audio_content=content) for content in self._generator())
        responses = client.streaming_recognize(streaming_config, requests)
        self._listen_print_loop(responses)

    def shutdown(self):
        """Shut down as cleanly as possible"""
        rospy.loginfo("Shutting down")
        self.closed = True
        self._buff.put(None)
        self.stream.close()
        self.audio.terminate()
        exit()

    def start_client(self):
        """Entry function to start the client"""
        try:
            rospy.loginfo("Starting Google speech mic client")
            self.gspeech_client()
        except KeyboardInterrupt:
            self.shutdown()


if __name__ == '__main__':
    rospy.init_node('dialogflow_mic_client')
    g = GspeechClient()
    g.start_client()

