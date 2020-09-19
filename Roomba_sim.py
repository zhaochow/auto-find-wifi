#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: Zhao CHOW
# Date: May 1 2018

from threading import Thread
import time

class Roomba_sim(Thread):
    """Class simulating a Roomba"""
    def __init__(self):
        super(Roomba_sim, self).__init__()
        self.distance = 0 # Distance traveled
        self.angle = 0 # Angle traveled
        self.move = False # Enable moving, by default stopped
        self.rot = False # Enable rotating, by default stopped
        self.up_down = 1
        self.left_right = 1

    def run(self):
        while True: # Infinite loop
            if self.move:
                self.distance += self.up_down*2*3 # Tune coefficient to have a faster or slower movement
            if self.rot:
                self.angle += self.left_right*0.5*3 # Tune coefficient to have a faster or slower rotation
            time.sleep(0.01)

    def getDistance(self):
        """Return then reset the distance traveled."""
        tmp = self.distance
        self.distance = 0
        return tmp

    def getAngle(self):
        """Return then reset the angle traveled."""
        tmp = self.angle
        self.angle = 0
        return tmp

    def getBumpers(self):
        """Return bumpers."""
        return False,False # Always return False

    def move_straight(self,val):
        """Enable moving straight if val != 0."""
        if val < 0:
            self.move = True
            self.up_down = -1 # Moving backward
        elif val > 0:
            self.move = True
            self.up_down = 1 # Moving forward
        else:
            self.move = False

    def turn(self,val):
        """Enable rotating if val != 0."""
        if val < 0:
            self.rot = True
            self.left_right = -1 # Rotating right
        elif val > 0:
            self.rot = True
            self.left_right = 1 # Rotating left
        else:
            self.rot = False
