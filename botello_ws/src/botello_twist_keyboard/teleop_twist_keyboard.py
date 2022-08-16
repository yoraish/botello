#!/usr/bin/env python

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty



msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
"""
# Move bindings of form (x,y,z,yaw).
moveBindings = {
        'i':(0,1,0,0), # Pitch +
        ',':(0,-1,0,0), # Pitch -
        'j':(-1,0,0,0), # Roll -
        'l':(1,0,0,0), # Roll +
        's':(0,0,0,-1), # Yaw +
        'f':(0,0,0,1), # Yaw -
        'e':(0,0,1,0), # Height +
        'x':(0,0,-1,0), # Height -
        'c':(0,0,-1,0), # Height -, redundant key for comfort.
    }

speedBindings={}
    #     'q':(1.1,1.1),
    #     'z':(.9,.9),
    #     'w':(1.1,1),
    #     'x':(.9,1),
    #     'e':(1,1.1),
    #     'c':(1,.9),
    # }


cmdBindings={
    'T': 'takeoff',
    't': 'land',
}

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('tello/cmd_vel', Twist, queue_size = 1)
        self.takeoff_publisher = rospy.Publisher('tello/takeoff', Empty, queue_size = 1)
        self.land_publisher = rospy.Publisher('tello/land', Empty, queue_size = 1)

        # Publish an empty message to block any other messages from getting to /tello/cmd_vel from other packages.
        self.manual_override_pub = rospy.Publisher('botello/manual_override', Empty, queue_size = 1)


        self.empty_msg = Empty()
        
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.cmd = None
        self.condition = threading.Condition()
        self.done = False
        self.recent_movement_update = [self.x, self.y, self.z, self.th, self.speed, self.turn]

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn, cmd):
 
        self.condition.acquire()
        # If the current movememt command is similar to the recent movement command, then make that one more aggresive (add the values instead of setting them).
        if [x, y, z, th, speed, turn] == self.recent_movement_update:
            self.speed += speed
            self.turn += turn
        else:
            self.speed = speed
            self.turn = turn

        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.cmd = cmd
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

        # Update the recent values with the current values.
        self.recent_movement_update = [x, y, z, th, speed, turn]


    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0, None)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)
            self.manual_override_pub.publish(self.empty_msg)
            rospy.loginfo("\nMANUAL Override.\n")

            # Check for new commands.
            if self.cmd is not None:
                rospy.loginfo("\nMANUAL Command " + self.cmd + ".\n")
                # If takeoff or land, also send stop velocities.
                if self.cmd == "takeoff" or self.cmd == "land":
                    # Publish stop message when taking off or landing.
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.linear.z = 0
                    twist.angular.x = 0
                    twist.angular.y = 0
                    twist.angular.z = 0
                    self.publisher.publish(twist)
                    self.manual_override_pub.publish(self.empty_msg)

                    if self.cmd == "takeoff":
                        self.takeoff_publisher.publish(self.empty_msg)
                    elif self.cmd == "land":
                        self.land_publisher.publish(self.empty_msg)
                    self.cmd = None


                else:
                    print("Unknown command " + self.cmd + " received.")
                self.cmd = None

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() # returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('botello_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    cmd = None

    try:
        # pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn, cmd)

        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15

            elif key in cmdBindings.keys():
                cmd = cmdBindings[key]

            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break
 
            pub_thread.update(x, y, z, th, speed, turn, cmd)

            # Reset cmd.
            cmd = None

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)

