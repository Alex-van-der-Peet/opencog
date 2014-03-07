#!/usr/bin/env python

"""
    ITFDemo.py allows giving instructions to, chatting with, and listening to our mobile robot base with simple speech commands/phrases.
    Based on the voice_nav.py script by Patrick Goebel in the ROS by Example tutorial.
"""

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

class ITFDemo:
    def __init__(self):
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


    def cleanup(self):
        # When shutting down be sure to stop the robot!  Publish a Twist message consisting of all zeros.
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        rospy.loginfo("Shutting down itf_demo node...")


if __name__=="__main__":
    '''The main entry point...'''

    # Initialization
    rospy.init_node('itf_demo')

    try:
        # Just need to instantiate, initialization should handle the rest
        ITFDemo()
    except:
        pass

