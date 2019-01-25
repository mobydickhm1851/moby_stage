#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Ini.
# All rights reserved.
# Modified for the use of intersection game, LiuYC SOLab, 2019
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Control Your Solabot!
---------------------------
Moving around:
        w    
   a    s    d
        x    
space key, q : force stop
no key input : stop smoothly
CTRL-C to quit
"""

multiplier = 3
speed = .5*multiplier
turn = 1


moveBindings = {
        'w':(1*multiplier,0),
        's':(0,0),
        'x':(-1*multiplier,0),
        'a':(0,0),
        'd':(0,0),
           }


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('car_control', anonymous=True)
    car = rospy.get_param('~car_name', 'car0')
    pub = rospy.Publisher('/{0}/cmd_vel'.format(car), Twist, queue_size=5)
    rate = rospy.Rate(50) # default is 10

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.02
    brake = 0.04
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
                if key == 's':
                    brake_mode = 1
                else:
                    brake_mode = 0

            elif key == ' ':
                x = -1
                th = 0
	
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th


            if target_speed > control_speed:
                if brake_mode:
                    control_speed = min( target_speed, control_speed + brake )
                else:
                    control_speed = min( target_speed, control_speed + acc )
            elif target_speed < control_speed:
                if brake_mode:
                    control_speed = max( target_speed, control_speed - brake )
                else:
                    control_speed = max( target_speed, control_speed - acc )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x = 0; twist.linear.y = control_speed; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            pub.publish(twist)

            rate.sleep()

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
