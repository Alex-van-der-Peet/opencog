#!/usr/bin/env python

"""
    ITFDemo.py allows giving instructions to, chatting with, and listening to our mobile robot base with simple speech commands/phrases.
    Based on the voice_nav.py script by Patrick Goebel in the ROS by Example tutorial.
"""

import roslib; roslib.load_manifest('speech')
import rospy
import math
import os
import subprocess
import re
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

#speech_path = roslib.packages.get_pkg_dir('speech')
#sphinx_path = roslib.packages.get_pkg_dir('sphinx')

speech_path = '/home/ros/ros/sail-ros-pkg/semistable/audio/speech/'
sphinx_path = '/home/ros/ros/sail-ros-pkg/semistable/3rdparty/sphinx'

listFile = 'stdin' # file containing list of .raw files that livepretend should decode
argFile = speech_path + '/sphinx/ispy.arg' # aruments to livepretend
dir = speech_path + '/raw/' # directory of .raw files
program = sphinx_path + '/bin/sphinx4_liveadapt' # ASR decoder
print "Sphinx path: " + program
#TODO: Remove the speakerFile?
speakerFile = sphinx_path + '/raw/adam_train_read.raw' # pre-recorded audio for us in adaptation

class ITFDemo:
    def __init__adapt(self, train):
        """ Used for speaker adaptation initialization.
            Start the sphinx speech recognition process, run adaptation on train file and discard results from proc.stdout
        """
        #self.pub = rospy.Publisher('tts', String) # parrot
        self.pub = rospy.Publisher('nlu', String) # parrot
        cmd = [rpogram, listFile, dir, argFile]
        rospy.loginfo(cmd)
        self.proc = subprocess.Popen(cmd, shell=False, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

        rospy.loginfo("Adapting on %s\n" % train.string('.raw'))
        #self.proc.stdin.write(train.strip('.raw'))
        #self.proc.stdin.write('adam_train_read') #XXX: uncomment for adaptation
        self.proc.stdin.write('\n')
        end_utt_re = re.compile("sec CPU,")
        hyp_re = re.compile("PARTIONAL_HYP: (.*)$")
        hyp = ""
        #time.sleep(10)
        while 1:
            line = self.proc.stdout.readline()
            if line:
                rospy.loginfo(line.strip())
            m = hyp_re.search(line)
            if m:
                hyp = m.group(1)
            elif end_utt_re.search(line):
                rospy.loginfo("Adaptation complete.")
                break
         rospy.loginfo("hyp: %s\n" % hyp.strip())

    def __init__(self):
        """ Plain Vanilla Startup """
        rospy.on_shutdown(self.cleanup)
		
        # Initailize data members from parameters
        self.max_speed = rospy.get_param("~max_speed", 0.4)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.5)
        self.speed = rospy.get_param("~start_speed", 0.1)
        self.angular_speed = rospy.get_param("~start_angular_speed", 0.5)
        self.linear_increment = rospy.get_param("~linear_increment", 0.05)
        self.angular_increment = rospy.get_param("~angular_increment", 0.4)
        self.rate = rospy.get_param("~rate", 5)
        r = rospy.Rate(self.rate)
        self.pauseListen = False
        self.pauseFollow = True
	self.pauseSpeech = False

        self.voice = rospy.get_param("~voice", "voice_don_diphone")
        self.wavepath = rospy.get_param("~wavepath", "")

        # Create the sound client object
        self.soundhandle = SoundClient()
		
        rospy.sleep(1)
        self.soundhandle.stopAll()

        # Initialize the Twist message we will publish.
        self.msg = Twist()

        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)

        # Subscribe to the recognizer output
        rospy.Subscriber('/recognizer/output', String, self.listen)

        # A mapping from keywords to commands.
        self.keywords_to_command = {
             'stop': ['stop', 'halt', 'abort', 'kill', 'panic', 'off', 'freeze', 'shut down', 'turn off', 'help', 'help me'],
             'slower': ['slow down', 'slower'],
             'faster': ['speed up', 'faster'],
             'forward': ['forward', 'ahead', 'straight'],
             'backward': ['back', 'backward', 'back up'],
             'rotate left': ['rotate left'],
             'rotate right': ['rotate right'],
             'turn left': ['turn left'],
             'turn right': ['turn right'],
             'quarter': ['quarter speed'],
             'half': ['half speed'],
             'full': ['full speed'],
             'stop listen': ['pause listen', 'stop listen'],
             'start listen': ['continue listen', 'resume listen', 'start Listen'],
             'start follow': ['continue follow', 'resume follow', 'start follow'],
	     'stop follow': ['pause follow', 'stop follow']}

        # Announce that we are ready for input
        self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        rospy.sleep(1)
        self.soundhandle.say("Ready", self.voice)

	if not(self.pauseListen):
            rospy.loginfo("Listening for voice commands")

        if not(self.pauseFollow):
            rospy.loginfo("Following current target")

        if not(self.pauseSpeech):
            rospy.loginfo("Speaking responses to target")

        # Setup sphinx listening process
        self.pub = rospy.Publisher('nlu', String) # parrot
        cmd = [program, listFile, dir, argFile]
        rospy.loginfo(cmd)
        self.proc = subprocess.Popen(cmd, shell=False, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        line = ""
        while (not line.startswith("INFO: s3_decode.c(264): Partial hypothesis WILL be dumped")):
            line  = self.proc.stdout.readline()
            print line.strip()

        rospy.loginfo("Done with initialization")


    def startup():
        rospy.loginfo("Sphinx ASR startup\n")
        # remove old file list
        # doit = sphinxwrapper(speakerFile)
        doit = ITFDemo()
        rospy.init_node('itf_demo')
        rospy.Subscriber("audio_clip_loc", String, doit.asr)
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.msg)
            r.sleep()     


    def get_command(self, data):
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    return command


    def talkback(self, msg):
        # Print the recognized words on the screen
        rospy.loginfo(msg.data)
		
        # Speak the recognized words in the selected voice
        self.soundhandle.say(msg.data, self.voice)

        # Uncomment to play one of the built-in sounds
        #rospy.sleep(2)
        #self.soundhandle.play(5)

        # Uncomment to play a wave file
        #rospy.sleep(2)
        #self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")


    def listen(self, msg):        
        command = self.get_command(msg.data)
        
        rospy.loginfo("Command: " + str(command))
        
        if command == 'stop listen':
            self.pauseListen = True
            rospy.loginfo("Not listening for voice commands")
        elif command == 'start listen':
            self.pauseListen = False
            rospy.loginfo("Listening for voice commands")
            
        if self.pauseListen:
            return       
        
        if command == 'forward':    
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0
            rospy.loginfo("Moving Forward")
            
        elif command == 'rotate left':
            self.msg.linear.x = 0
            self.msg.angular.z = self.angular_speed
            rospy.loginfo("Rotating Left")
                
        elif command == 'rotate right':  
            self.msg.linear.x = 0      
            self.msg.angular.z = -self.angular_speed
            rospy.loginfo("Rotating Right")
            
        elif command == 'turn left':
            if self.msg.linear.x != 0:
                self.msg.angular.z += self.angular_increment
            else:        
                self.msg.angular.z = self.angular_speed
            rospy.loginfo("Turning Left")
                
        elif command == 'turn right':    
            if self.msg.linear.x != 0:
                self.msg.angular.z -= self.angular_increment
            else:        
                self.msg.angular.z = -self.angular_speed
            rospy.loginfo("Turning Right")
                
        elif command == 'backward':
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0
            rospy.loginfo("Moving Backward")
            
        elif command == 'stop': 
            # Stop the robot!  Publish a Twist message consisting of all zeros.         
            # TODO: do we want to stop all movement?  Including follow?
            self.msg = Twist()
            rospy.loginfo("Stopping all movement")
        
        elif command == 'faster':
            self.speed += self.linear_increment
            self.angular_speed += self.angular_increment
            if self.msg.linear.x != 0:
                self.msg.linear.x += copysign(self.linear_increment, self.msg.linear.x)
            if self.msg.angular.z != 0:
                self.msg.angular.z += copysign(self.angular_increment, self.msg.angular.z)
            rospy.loginfo("Moving Faster")
            
        elif command == 'slower':
            self.speed -= self.linear_increment
            self.angular_speed -= self.angular_increment
            if self.msg.linear.x != 0:
                self.msg.linear.x -= copysign(self.linear_increment, self.msg.linear.x)
            if self.msg.angular.z != 0:
                self.msg.angular.z -= copysign(self.angular_increment, self.msg.angular.z)
            rospy.loginfo("Moving Slower")
                
        elif command in ['quarter', 'half', 'full']:
            if command == 'quarter':
                self.speed = copysign(self.max_speed / 4, self.speed)
                rospy.loginfo("Moving Quarter speed")
        
            elif command == 'half':
                self.speed = copysign(self.max_speed / 2, self.speed)
                rospy.loginfo("Moving Half speed")
            
            elif command == 'full':
                self.speed = copysign(self.max_speed, self.speed)
                rospy.loginfo("Moving Full speed")
            
            if self.msg.linear.x != 0:
                self.msg.linear.x = copysign(self.speed, self.msg.linear.x)

            if self.msg.angular.z != 0:
                self.msg.angular.z = copysign(self.angular_speed, self.msg.angular.z)

        elif command == 'start follow':
            # start follow module
            #TODO: call module function
            self.pauseFollow = False
            rospy.loginfo("Following current target")
        
        elif command == 'stop follow':
            # stop follow module, maybe send an empty Twist?
            #TODO: call module function
            self.pauseFollow = True
            rospy.loginfo("Not Following current target")

        elif command == 'stop speech':
            # stop chatbot module
            #TODO: call module function
            self.pauseSpeech = True
            rospy.loginfo("Speaking responses to target")

        elif command == 'start speech':
            # start chatbot module
            #TODO: call module function
            self.pauseSpeech = False
            rospy.loginfo("Not speaking responses to target")

        elif msg.data != "" and self.pauseSpeech:
            # send to chatbot module
            #TODO: call module function
            rospy.loginfo("Sending message to chatbot")
                
        else:
            return

	# Cap our linear and angular speed
        self.msg.linear.x = min(self.max_speed, max(-self.max_speed, self.msg.linear.x))
        self.msg.angular.z = min(self.max_angular_speed, max(-self.max_angular_speed, self.msg.angular.z))

    def asr(self, data):
        rospy.loginfo("Sphinx ASR: %s\n" % data.data)
        self.proc.stdin.write(data.data.strip('.raw'))
        self.proc.stdin.write('\n')
        end_utt_re = re.compile("sec CPU,") # last line of utterance output
        hyp_re = re.compile("PARTIAL_HYP: (.*)$")
        hyp = ""
        while 1: # file object for input
            line = self.proc.stdout.readline()
            if (line != ""):
                rospy.loginfo("Line: " + line)
            m = hyp_re.search(line)
            if m:
                hyp = m.group(1)
            elif end_utt_re.search(line):
                break

        rospy.loginfo("Recognized: %s\n" % hyp.strip())
        self.pub.publish(String(hyp.strip()))

    def cleanup(self):
        # When shutting down be sure to stop the robot!  Publish a Twist message consisting of all zeros.
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        rospy.loginfo("Shutting down itf_demo node...")


if __name__=="__main__":
    '''The main entry point...'''
        startup()


