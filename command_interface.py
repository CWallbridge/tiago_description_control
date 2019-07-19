#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import curses
import math
import rospy
import geometry_msgs.msg
import tf
import time

from std_msgs.msg import String

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
        
class CommandTeleop():
    def __init__(self, interface):
        self._interface = interface
        self._pub_cmd = rospy.Publisher('tiago/place_desc/command', String, queue_size=1)

        self._hz = rospy.get_param('~hz', 10)

        self._last_pressed = {}
        self._angular = 0
        self._linear = 0
        self._condition = "N-D"
        self._map = 0
        self._target = 0
        self._state = "Setup"


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
            #self._set_velocity()
            self._publish()
            rate.sleep()

    def _key_pressed(self, keycode):
        
        if keycode == ord('q'):
            self._running = False
            rospy.signal_shutdown('Bye')
        
        if self._state == "Setup":
            if keycode == ord('d'):
                self._pub_cmd.publish("0-D-N")
                self._condition = 'D-N'
            if keycode == ord('n'):
                self._pub_cmd.publish("0-N-D")
                self._condition = 'N-D'
            if keycode == ord('r'):
                self._pub_cmd.publish("0-R-R")
                self._condition = "Pilot"
            if keycode == ord('s'):
                self._pub_cmd.publish("tutorial")
                self._state = "Describing"
            keycode = ord('a')
                
        if self._state == "Describing":
            if keycode == ord('n'):
                self._pub_cmd.publish("wait")
                self._state = "Waiting"
            keycode = ord('a')
                
        if self._state == "Waiting":
            if keycode == ord('n'):
                self._pub_cmd.publish("success")
                self._target = self._target + 1
                if self._target < 6:
                    self._state = "Describing"
                else:
                    self._target = 0
                    if self._map == 0:
                        self._state = "End1"
                    else:
                        self._state = "End2"  
            if keycode == ord('p'):
                self._pub_cmd.publish("resume")
            keycode = ord('a')
        
        if self._state == "End1":
            if keycode == ord('s'):
                self._map = 1
                self._pub_cmd.publish("placement2")
                self._state = "Describing"
            keycode = ord('a')
                
        if self._state == "End2":
            if keycode == ord('s'):
                self._map = 0
                self._state = "Setup"
            keycode = ord('a')
            
        #elif keycode in self.movement_bindings:
            #self._last_pressed[keycode] = rospy.get_time()

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(2, 'Condition: %s' % (self._condition))
        self._interface.write_line(3, 'Map: %d Target: %d State: %s' % (self._map, self._target, self._state))
        if self._state == "Setup":
            self._interface.write_line(5, 'Set condition to Dynamic first (D-N) = d')
            self._interface.write_line(6, 'Set condition to Non-Ambiguous first (N-D) = n')
            self._interface.write_line(7, 'Set condition to recording (Pilot) = r')
            self._interface.write_line(8, 'Start the study = s')
        if self._state == "Describing":
            self._interface.write_line(5, 'Press n when the correct object has been picked up')
        if self._state == "Waiting":
            self._interface.write_line(5, 'Press n to begin the next description after the previous object is placed')
            self._interface.write_line(6, 'Press p to resume previous description')
        if self._state == "End1":
            self._interface.write_line(5, 'First set of descriptions completed.')
            self._interface.write_line(6, 'Ask participant to fill in second section of questionnaire.')
            self._interface.write_line(7, 'Press s to start next set of descriptions.')
        if self._state == "End2":
            self._interface.write_line(5, 'Second set of descriptions completed.')
            self._interface.write_line(6, 'Ask participant to fill in last section of questionnaire.')
            self._interface.write_line(7, 'Press s to return to setup.')
        self._interface.write_line(9, 'Press q to quit')
        self._interface.refresh()

        #self._pub_cmd.publish(twist)

def main(stdscr):
    #moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cmd_interface')
    app = CommandTeleop(TextWindow(stdscr))
    app.run()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
