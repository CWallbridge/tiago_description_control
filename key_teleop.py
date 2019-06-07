#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Modification of the key_teleop.py code under Copyright (c) 2013 PAL 
# Robotics SL that is Released under the BSD License originally by 
# Siegfried-A. Gevatter
#
# Released under the BSD License
# Author:
# Christopher D. Wallbridge

import sys
import curses
import math
import rospy
import geometry_msgs.msg
import tf
import time
#import moveit_commander
#import moveit_msgs.msg

#from moveit_commander.conversions import pose_to_list
from math import pi
from std_msgs.msg import String
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from geometry_msgs.msg import Twist

#barrel_width = .1355
#barrel_height = .205

#rest_pose = geometry_msgs.msg.Pose()
#quaternion = tf.transformations.quaternion_from_euler(-0.011, 1.57, 0.037)
#rest_pose.orientation.x = quaternion[0]
#rest_pose.orientation.y = quaternion[1]
#rest_pose.orientation.z = quaternion[2]
#rest_pose.orientation.w = quaternion[3]
#rest_pose.position.x = 0.6
#rest_pose.position.y = -0.3
#rest_pose.position.z = 0.67
#intm_pose = geometry_msgs.msg.Pose()
#intm_pose.orientation.x = quaternion[0]
#intm_pose.orientation.y = quaternion[1]
#intm_pose.orientation.z = quaternion[2]
#intm_pose.orientation.w = quaternion[3]
#intm_pose.position.x = 0.6
#intm_pose.position.y = 0.0
#intm_pose.position.z = 0.67
#grab_pose = geometry_msgs.msg.Pose()
#grab_pose.orientation.x = quaternion[0]
#grab_pose.orientation.y = quaternion[1]
#grab_pose.orientation.z = quaternion[2]
#grab_pose.orientation.w = quaternion[3]
#grab_pose.position.x = 0.6
#grab_pose.position.y = 0.0
#grab_pose.position.z = 0.465

#quaternion = tf.transformations.quaternion_from_euler(-0.011, 1.57, 0.037)
#rest_pose.orientation.x = quaternion[0]
#rest_pose.orientation.y = quaternion[1]
#rest_pose.orientation.z = quaternion[2]
#rest_pose.orientation.w = quaternion[3]
#rest_pose.position.x = 0.4
#rest_pose.position.y = -0.3
#rest_pose.position.z = 0.26

class Velocity(object):

    def __init__(self, min_velocity, max_velocity, num_steps):
        assert min_velocity > 0 and max_velocity > 0 and num_steps > 0
        self._min = min_velocity
        self._max = max_velocity
        self._num_steps = num_steps
        if self._num_steps > 1:
            self._step_incr = (max_velocity - min_velocity) / (self._num_steps - 1)
        else:
            # If num_steps is one, we always use the minimum velocity.
            self._step_incr = 0

    def __call__(self, value, step):
        """
        Takes a value in the range [0, 1] and the step and returns the
        velocity (usually m/s or rad/s).
        """
        if step == 0:
            return 0

        assert step > 0 and step <= self._num_steps
        max_value = self._min + self._step_incr * (step - 1)
        return value * max_value

class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError, 'lineno out of bounds'
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()

class KeyTeleop():

    _interface = None

    _linear = None
    _angular = None

    def __init__(self, interface):
        self._interface = interface
        self._pub_cmd = rospy.Publisher('key_vel', Twist)

        self._hz = rospy.get_param('~hz', 10)

        self._num_steps = rospy.get_param('~turbo/steps', 4)

        forward_min = rospy.get_param('~turbo/linear_forward_min', 0.5)
        forward_max = rospy.get_param('~turbo/linear_forward_max', 1.0)
        self._forward = Velocity(forward_min, forward_max, self._num_steps)

        backward_min = rospy.get_param('~turbo/linear_backward_min', 0.25)
        backward_max = rospy.get_param('~turbo/linear_backward_max', 0.5)
        self._backward = Velocity(backward_min, backward_max, self._num_steps)

        angular_min = rospy.get_param('~turbo/angular_min', 0.7)
        angular_max = rospy.get_param('~turbo/angular_max', 1.2)
        self._rotation = Velocity(angular_min, angular_max, self._num_steps)

    def run(self):
        self._linear = 0
        self._angular = 0

        rate = rospy.Rate(self._hz)
        while True:
            keycode = self._interface.read_key()
            if keycode:
                if self._key_pressed(keycode):
                    self._publish()
            else:
                self._publish()
                rate.sleep()

    def _get_twist(self, linear, angular):
        twist = Twist()
        if linear >= 0:
            twist.linear.x = self._forward(1.0, linear)
        else:
            twist.linear.x = self._backward(-1.0, -linear)
        twist.angular.z = self._rotation(math.copysign(1, angular), abs(angular))
        return twist

    def _key_pressed(self, keycode):
        movement_bindings = {
            curses.KEY_UP:    ( 1,  0),
            curses.KEY_DOWN:  (-1,  0),
            curses.KEY_LEFT:  ( 0,  1),
            curses.KEY_RIGHT: ( 0, -1),
        }
        speed_bindings = {
            ord(' '): (0, 0),
        }
        if keycode in movement_bindings:
            acc = movement_bindings[keycode]
            ok = False
            if acc[0]:
                linear = self._linear + acc[0]
                if abs(linear) <= self._num_steps:
                    self._linear = linear
                    ok = True
            if acc[1]:
                angular = self._angular + acc[1]
                if abs(angular) <= self._num_steps:
                    self._angular = angular
                    ok = True
            if not ok:
                self._interface.beep()
        elif keycode in speed_bindings:
            acc = speed_bindings[keycode]
            # Note: bounds aren't enforced here!
            if acc[0] is not None:
                self._linear = acc[0]
            if acc[1] is not None:
                self._angular = acc[1]

        elif keycode == ord('q'):
            rospy.signal_shutdown('Bye')
        else:
            return False

        return True

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(2, 'Linear: %d, Angular: %d' % (self._linear, self._angular))
        self._interface.write_line(5, 'Use arrow keys to move, space to stop, q to exit.')
        self._interface.refresh()

        twist = self._get_twist(self._linear, self._angular)
        self._pub_cmd.publish(twist)


class SimpleKeyTeleop():
    def __init__(self, interface):
        self._interface = interface
        self._pub_cmd = rospy.Publisher('key_vel', Twist)

        self._hz = rospy.get_param('~hz', 10)

        self._forward_rate = rospy.get_param('~forward_rate', 0.4)
        self._backward_rate = rospy.get_param('~backward_rate', 0.25)
        self._rotation_rate = rospy.get_param('~rotation_rate', 0.5)
        self._last_pressed = {}
        self._angular = 0
        self._linear = 0
        self._arm_pos = "right"
        self._drive_mode = "normal"

        self._move_arm('stretch')
        self._move_arm('unfold_arm')
        self._move_arm('idle_pos')
        
        self._state = 'drive'

    movement_bindings = {
        curses.KEY_UP:    ( 1,  0),
        curses.KEY_DOWN:  (-1,  0),
        curses.KEY_LEFT:  ( 0,  1),
        curses.KEY_RIGHT: ( 0, -1),
    }

    def _wait_for_valid_time(self, timeout):
        """Wait for a valid time (non-zero), this is important
        when using a simulated clock"""
        # Loop until:
        # * ros master shutdowns
        # * control+C is pressed (handled in is_shutdown())
        # * timeout is achieved
        # * time is valid
        start_time = time.time()
        while not rospy.is_shutdown():
            if not rospy.Time.now().is_zero():
                return
            if time.time() - start_time > timeout:
                rospy.logerr("Timed-out waiting for valid time.")
                exit(0)
            time.sleep(0.1)
        # If control+C is pressed the loop breaks, we can exit
        exit(0)


    def _get_status_string(self, status_code):
        return GoalStatus.to_string(status_code)

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._set_velocity()
            self._publish()
            rate.sleep()

    def _get_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        return twist

    def _set_velocity(self):
        now = rospy.get_time()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < 0.4:
                keys.append(a)
        linear = 0.0
        angular = 0.0
        for k in keys:
            l, a = self.movement_bindings[k]
            linear += l
            angular += a
        if linear > 0:
            linear = linear * self._forward_rate
        else:
            linear = linear * self._backward_rate
        angular = angular * self._rotation_rate
        self._angular = angular
        self._linear = linear
        
    def _move_arm(self, motion):
        #rospy.loginfo("Starting run_motion_python application...")
        #_wait_for_valid_time(10.0)

        client = SimpleActionClient('/play_motion', PlayMotionAction)

        #rospy.loginfo("Waiting for Action Server...")
        client.wait_for_server()

        goal = PlayMotionGoal()
        goal.motion_name = motion
        goal.skip_planning = False
        goal.priority = 0  # Optional

        #rospy.loginfo("Sending goal with motion: " + motion)
        client.send_goal(goal)

        #rospy.loginfo("Waiting for result...")
        action_ok = client.wait_for_result(rospy.Duration(30.0))

        state = client.get_state()

        #if action_ok:
        #    rospy.loginfo("Action finished succesfully with state: " + str(_get_status_string(state)))
        #else:
        #    rospy.logwarn("Action failed with state: " + str(_get_status_string(state)))

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._move_arm('home')
            self._running = False
            rospy.signal_shutdown('Bye')
        
        if self._state == 'drive':
            if keycode == ord('g'):
                self._move_arm('over_pos')
                self._move_arm('grab_pos')
                #self._move_arm('over_pos')
                #self._move_arm('idle_pos')
                self._state = 'grab'
                keycode = ord('a')
                
            elif keycode == ord('h'):
                if self._arm_pos == "right":
                    self._arm_pos = "left"
                    self._move_arm('idle_pos_left')
                else:
                    self._arm_pos = "right"
                    self._move_arm('idle_pos')
                keycode = ord('a')
            
            elif keycode == ord('f'):
                if self._drive_mode == "normal":
                    self._drive_mode = "precision"
                    self._forward_rate = rospy.get_param('~forward_rate', 0.1)
                    self._backward_rate = rospy.get_param('~backward_rate', 0.1)
                    self._rotation_rate = rospy.get_param('~rotation_rate', 0.1)
                else:
                    self._drive_mode = "normal"
                    self._forward_rate = rospy.get_param('~forward_rate', 0.4)
                    self._backward_rate = rospy.get_param('~backward_rate', 0.25)
                    self._rotation_rate = rospy.get_param('~rotation_rate', 0.5)
                
            elif keycode in self.movement_bindings:
                self._last_pressed[keycode] = rospy.get_time()
                
        if self._state == 'grab':
            if keycode == ord('g'):
                self._move_arm('over_pos')
                if self._arm_pos == "right":
                    self._move_arm('idle_pos')
                else:
                    self._move_arm('idle_pos_left')
                self._state = 'drive'
                keycode = ord('a')

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(2, 'Linear: %f, Angular: %f, Arm Position : %s, Drive Mode : %s' % (self._linear, self._angular, self._arm_pos, self._drive_mode))
        if self._state == 'drive':
            self._interface.write_line(5, 'Use arrow keys to move, g to go to grab position, h to switch arm positions, q to exit.')
            self._interface.write_line(6, 'g to go to grab position')
            self._interface.write_line(7, 'h to switch arm positions')
            self._interface.write_line(8, 'f to switch to toggle fine position control')
            self._interface.write_line(9, 'q to exit')
        if self._state == 'grab':
            self._interface.write_line(5, 'Press g to leave grab position and resume driving, q to exit.')
        self._interface.refresh()

        twist = self._get_twist(self._linear, self._angular)
        self._pub_cmd.publish(twist)


def main(stdscr):
    #moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('key_teleop')
    app = SimpleKeyTeleop(TextWindow(stdscr))
    app.run()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
