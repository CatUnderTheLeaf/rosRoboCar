#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Short and a little bit edited version of `key_teleop.py` from `teleop_tools` package
# https://github.com/ros-teleop/teleop_tools
# Copyright (c) 2013 PAL Robotics SL.
# Released under the BSD License.

import curses

import rospy

from i2cpwm_board.srv import StopServos

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
            raise ValueError('lineno out of bounds')
        height, width = self._screen.getmaxyx()
        y = int((height / self._num_lines) * lineno)
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()

class OnKeyPress():
    def __init__(self, interface):
        self._interface = interface

        self._hz = rospy.get_param('~hz', 10)

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)            
            rate.sleep()

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            rospy.loginfo("shutting down")
            self.stop()
            self._running = False
            rospy.signal_shutdown('Bye')

    def stop(self):
        """
        Call for a `stop_servos` service to turn off servos when shutting down
        """
        rospy.wait_for_service('stop_servos')
        try:
            stop = rospy.ServiceProxy('stop_servos', StopServos)
            stop()
            rospy.loginfo("stopped")
            rospy.signal_shutdown("stopped motors")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

def main(stdscr):
    rospy.init_node('stop_car_node', log_level=rospy.INFO)
    app = OnKeyPress(TextWindow(stdscr))
    app.run()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass