#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: Zhao CHOW
# Date: May 1 2018
#
# This code is inspired from Create2_TetheredDrive.py of iRobot.

from threading import Thread, Lock
from Queue import Queue
import struct
import serial
import time
import math as m
from copy import deepcopy

import Roomba_sim

class Exploration(Thread):
    """Thread handling the movement of the Roomba"""
    def __init__(self, roomba_sim=None):
        super(Exploration, self).__init__()
        self.__running = False # Indicate if the thread is running or not
        self.__q = Queue() # Waiting list for tasks to be executed
        self.__results = [] # Results returned by the tasks

        self.__connection = None
        self.__roomba_sim = roomba_sim # Object simulating the Roomba
        self.__oldL = 0 # Previous left encoder
        self.__oldR = 0 # Previous right encoder
        self.__angle_abs = 0   # Absolute angle since beginning with initial angle being 0
        self.__X_abs = 0       # Absolute X since begining with initial position being (0,0)
        self.__Y_abs = 0       # Absolute Y since begining with initial position being (0,0)

        if self.__roomba_sim == None: # If there is no simulation object
            self.__onConnect() # Connect to the Roomba
            self.__sendCommandASCII('128') # Set Roomba in Passive Mode
        
    def run(self):
        """Function executed by the thread. Execute the tasks in the waiting list if there is any.
           Wait otherwise."""
        self.__running = True
        if self.__roomba_sim == None: # If there is no simulation object
            self.__sendCommandASCII('132') # Set Roomba in Full Mode

        while self.__running:
            if not self.__q.empty(): # If queue contains a task
                item = self.__q.get()
                res = item() # Execute the task and get results
                self.__results.append(res) # Add results to the list of results
                self.__q.task_done() # Report the task as finished
        
        if self.__roomba_sim == None: # If there is no simulation object
            self.__sendCommandASCII('128') # Set back Roomba in Passive Mode

    def stop(self):
        """Stop the thread"""
        self.__running = 0

    def move_straight(self,val):
        """Add a new task to the waiting list: move straight on [val] mm. Function to be called
           by other threads."""
        self.__q.put(lambda: self._move_straight2(val))

    def turn(self,val):
        """Add a new task to the waiting list: turn [val] degrees. Function to be called
           by other threads."""
        self.__q.put(lambda: self._turn2(val))

    def get_coord(self):
        """Return absolute position X,Y of the robot since it started running."""
        return self.__X_abs,self.__Y_abs

    def wait_finish(self):
        """Wait all the tasks in waiting list to finish and return the results."""
        # print 'Waiting tasks to finish'
        self.__q.join()
        # print 'Tasks finished'
        tmp = deepcopy(self.__results) # Deepcopy the results into a temporary variable
        del self.__results[:] # Reset the list of results
        return tmp

    def __onConnect(self):
        """Connect to the Roomba"""
        if self.__connection is not None:
            print "You're already connected!"
            return

        port = '/dev/ttyUSB0' # COM port of the Roomba
        if port is not None:
            print "Trying " + str(port) + "... "
            try:
                self.__connection = serial.Serial(port, baudrate=115200, timeout=1)
                print "Connected!"
            except:
                print "Failed."

    # sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send
    def __sendCommandASCII(self, command):
        cmd = ""
        for v in command.split():
            cmd += chr(int(v))

        self.__sendCommandRaw(cmd)

    # sendCommandRaw takes a string interpreted as a byte array
    def __sendCommandRaw(self, command):
        try:
            if self.__connection is not None:
                self.__connection.write(command)
            else:
                print "Not connected."
        except serial.SerialException:
            print "Lost connection"
            self.__connection = None

    # getDecodedBytes returns a n-byte value decoded using a format string.
    # Whether it blocks is based on how the connection was set up.
    def __getDecodedBytes(self, n, fmt):
        try:
            return struct.unpack(fmt, self.__connection.read(n))[0]
        except serial.SerialException:
            print "Lost connection"
            self.__connection = None
            return None
        except struct.error:
            print "Got unexpected data from serial port."
            return None

    # get8Unsigned returns an 8-bit unsigned value.
    def __get8Unsigned(self):
        return self.__getDecodedBytes(1, "B")

    # get8Signed returns an 8-bit signed value.
    def __get8Signed(self):
        return self.__getDecodedBytes(1, "b")

    # get16Unsigned returns a 16-bit unsigned value.
    def __get16Unsigned(self):
        return self.__getDecodedBytes(2, ">H")

    # get16Signed returns a 16-bit signed value.
    def __get16Signed(self):
        return self.__getDecodedBytes(2, ">h")

    def __getEncoder(self):
        """Get encoder values and convert into traveled distances in mm"""
        # Read left encoder
        self.__connection.write((struct.pack(">BB",142,43)))
        left = self.__get16Signed()
        # Read right encoder
        self.__connection.write((struct.pack(">BB",142,44)))
        right = self.__get16Signed()

        # Handle encoders' overflow
        if self.__oldL - left > 32767: # If oldL < 32767 and left > 32767
            diffL = left + pow(2,16) - self.__oldL
        elif left - self.__oldL > 32767: # If oldL > -32768 and left < -32768
            diffL = left - pow(2,16) - self.__oldL
        else:
            diffL = left - self.__oldL

        if self.__oldR - right > 32767: # If oldR < 32767 and right > 32767
            diffR = right + pow(2,16) - self.__oldR
        elif right - self.__oldR > 32767: # If oldR > -32768 and right < -32768
            diffR = right - pow(2,16) - self.__oldR
        else:
            diffR = right - self.__oldR

        # Compute left and right distance
        distL = (10000*diffL*m.pi*72.0)/508.8/10000         #########################
        distR = (10000*diffR*m.pi*72.0)/508.8/10000         #########################
        self.__oldL = left # Save left encoder
        self.__oldR = right # Save right encoder
        return distL, distR

    def __getDistance(self):
        """Return distance in mm since last call, negative value when going forward"""  #########################
        if self.__roomba_sim == None:
            distL, distR = self.__getEncoder()
            distance = (distL + distR)/2.0 # Average left and right traveled distance
            return distance
        else:
            return self.__roomba_sim.getDistance() # Get distance from simulation object

    def __getAngle(self):
        """Return angle in degrees since last call, positive value when rotating left"""
        if self.__roomba_sim == None:
            distL, distR = self.__getEncoder()
            angle = 180*(distR - distL)/(m.pi*235.00) # Compute angle from encoders. Formula from documentation of Roomba
            return angle
        else:
            return self.__roomba_sim.getAngle() # Get angle from simulation object

    def __getBumpers(self):
        """Return left and right bumpers state (True = bumped, False = nothing)"""
        if self.__roomba_sim == None:
            # Read bumpers
            self.__connection.write((struct.pack(">BB",142,7)))
            packet = self.__get8Unsigned()

            right = (packet&1 != 0) # Check if bit 0 is 1
            left = (packet&2 != 0) # Check if bit 1 is 1
            return left,right
        else:
            return self.__roomba_sim.getBumpers() # Get bumpers from simulation object

    def __getLightBumpers(self):
        """Return all light bumpers values [left,front_left,center_left,center_right,front_right,right]"""
        l = []
        # Read light bumpers
        for i in range(6):
            self.__connection.write((struct.pack(">BB",142,46+i)))
            l.append(self.__get16Unsigned())
        return l

    def __sign(self, x):
        """Return the sign of x"""
        if x < 0:
            s = -1
        else:
            s = 1
        return s

    def __move(self,up_down,velo,left_right,rot):
        """Move according to arguments. [up_down] 1 is Up, -1 is Down; [velo] Velocity (absolute value
           in mm/s [-500;500]); [left_right] 1 is Left, -1 is Right; [rot] Rotation velocity
           Velocity deadzone [-11;11], Rotation velocity deadzone [-21;22]"""
        if self.__roomba_sim == None:
            velocity = up_down * velo
            rotation = left_right * rot

            # compute left and right wheel velocities
            vr = velocity + (rotation/2)
            vl = velocity - (rotation/2)

            # create drive command
            cmd = struct.pack(">Bhh", 145, vr, vl)
            self.__sendCommandRaw(cmd)
        else:
            self.__roomba_sim.move_straight(velo)   ###############################

    def _move_straight2(self,val):
        """Move in straight line. [val] = distance to travel in mm (> 0 forward, < 0 backward).
           Stop when an obstacle is encountered."""
        bump = False # Initialize bump state
        # Initialize distance
        distance = abs(self.__getDistance()) # Call once for updating oldR and oldL
        distance = 0 # Reset

        if self.__roomba_sim == None:
            self.__move(self.__sign(val),300,0,0) # Move straight with 300 mm/s
        else:
            self.__roomba_sim.move_straight(val) # Move straight with simulation object

        # Loop until destination reached or obstacle encountered
        # -2 is an offset to take into account that the robot still has some inertia after stopping the motors
        while distance < abs(val) - 2:
            time.sleep(.016) # Wait 16 ms (Roomba updates the sensors' value only every 15 ms)

            left,right = self.__getBumpers()
            if left or right: # If there is an obstacle
                bump = True
                break # Immediately get out of the loop and stop moving

            err = abs(val) - distance
            if err < 70: # Move slower when arriving at desired position for a higher accuracy
                self.__move(self.__sign(val),11+0.5*err,0,0) # velocity below 11 = deadzone
            distance += abs(self.__getDistance())

        # Stop the Roomba
        if self.__roomba_sim == None:
            self.__move(0,0,0,0)
        else:
            self.__roomba_sim.move_straight(0)

        distance += abs(self.__getDistance()) # Last call to have a more accurate value
        distance *= self.__sign(val) # Take into account if it is forward or backward

        # Update absolute position X,Y
        delta_x,delta_y = self.__convert_to_XY(self.__angle_abs,distance)
        self.__X_abs += delta_x
        self.__Y_abs += delta_y

        print 'Distance: ', distance
        return distance

    def _turn2(self,val):
        """Turn on the spot. [val] = angle to turn in degrees (> 0 left, < 0 right)"""
        # Initialize angle
        angle = abs(self.__getAngle()) # Call once for updating oldR and oldL
        angle = 0

        if self.__roomba_sim == None:
            self.__move(0,0,self.__sign(val),200) # Rotate with velocity 200
        else:
            self.__roomba_sim.turn(val) # Rotate with simulation object

        # Loop until angle reached
        while angle < abs(val): # No need for offset when rotating
            time.sleep(.016) # Wait 16 ms (Roomba updates the sensors' value only every 15 ms)

            err = abs(val) - angle
            if err < 15: # Rotate slower when arriving at desired angle for a higher accuracy
                self.__move(0,0,self.__sign(val),22+3*err) # velocity below 22 = deadzone
            angle += abs(self.__getAngle())
            # angle += 10
            # print 'Angle (running): ', angle

        # Stop the Roomba
        if self.__roomba_sim == None:
            self.__move(0,0,0,0)
        else:
            self.__roomba_sim.turn(0)

        angle += abs(self.__getAngle()) # Last call to have a more accurate value
        angle *= self.__sign(val) # Take into account if it is left or right

        # Update absolute angle
        self.__angle_abs += angle

        print 'Angle: ', angle
        return angle

    def __convert_to_XY(self,angle, distance):
        """Convert angle and distance traveled into x and y traveled. Assume angle 0 is going forward
           and rotating left increases this value."""
        x = -distance*m.sin(m.radians(angle))
        y = distance*m.cos(m.radians(angle))
        return x,y

# ONLY RUN THIS FILE TO TEST MOVING AND ROTATING
if __name__ == "__main__":
    sim = input('Simulation? 1/0 (y/n)\n')
    if sim:
        Roomba_sim = Roomba_sim.Roomba_sim()
        Roomba_sim.daemon = True
        Roomba_sim.start()
        explore = Exploration(Roomba_sim)
    else:
        explore = Exploration()

    square = input('Demo square? 1/0 (y/n)\n')
    if square:
        SQUARE_SIDE = 1000
        DIAG = m.sqrt(2*SQUARE_SIDE/2*SQUARE_SIDE/2)
        explore._turn2(135)
        explore._move_straight2(DIAG)
        explore._turn2(-135)
        for i in range(4):
            if i >= 3:
                explore._move_straight2(SQUARE_SIDE/2)
                explore._turn2(-90)
                explore._move_straight2(SQUARE_SIDE/2)
            else:
                explore._move_straight2(SQUARE_SIDE)
                explore._turn2(-90)
        x,y = explore.get_coord()
        print 'X,Y: ', x, y
    else:
        angle = input('Angle (+ value = rotate left, - value = rotate right): ')
        distance = input('Distance (+ value = move forward, - value = move backward): ')
        explore._turn2(angle)
        explore._move_straight2(distance)

