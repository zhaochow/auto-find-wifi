#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: Zhao CHOW
# Date: May 1 2018

from threading import Thread
import time
from random import randint
import numpy as np
import math as m
import csv

import Exploration
import RSSI_Measure

class Localization(Thread):
    """Thread for executing one of the two localization algorithms: random direction
       or RSS gradient."""
    def __init__(self, explore_thread, rssi_obj, algo, max_it, nb_samples, mode, write):
        """Initialize the thread. [explore_thread] is the thread for exploration, [rssi_obj] is the object
           for measuring the RSSI, [algo] is the desired algorithm (1 - random direction, 2 - RSS gradient),
           [max_it] is the max number of iterations, [write] for specifying an output file and [mode] to
           select multithreading or procedural."""
        super(Localization, self).__init__()
        self.__explore_thread = explore_thread
        self.__rssi_obj = rssi_obj
        self.__algo = algo
        self.__max_it = max_it
        self.__nb_samples = nb_samples
        self.__mode = mode
        self.__write = write
        self.__filename = None
        if write:
            self.__init_res2save() # Initialize output file
        
    def run(self):
        """Run the localization algorithm specified at the creation of the thread."""
        if self.__mode == 1:
            if self.__algo == 1:
                self.__random_direction(300,self.__max_it,self.__nb_samples)
            elif self.__algo == 2:
                self.__RSS_gradient(500,self.__max_it,self.__nb_samples)
        elif self.__mode == 2:
            if self.__algo == 1:
                self._random_direction2(300,self.__max_it,self.__nb_samples)
            elif self.__algo == 2:
                self._RSS_gradient2(500,self.__max_it,self.__nb_samples)

    def __random_direction(self,d,max_it,nb_samples):
        """Random direction algorithm. [d] is the distance between 2 sampling locations,
           [max_it] is the max number of iterations and [nb_samples] is the number of samples taken
           for the average RSS."""
        previous_rssi = -1000

        it = 1
        while it <= max_it:
            print 'Iteration: ', it

            new_rssi,rssi_samples = self.__rssi_obj.retrieve_rssi(nb_samples)
            if new_rssi > -30: # If the RSSI is strong enough (robot is close to source) -> stop
                print 'Close to source -> stop'
                break

            if new_rssi < previous_rssi:
                self.__explore_thread.turn(randint(-179,180)) # Turn to random direction if RSS decreased
            self.__explore_thread.move_straight(d)
            results = self.__explore_thread.wait_finish() # Results: angle,distance
            if len(results) == 1: # Add angle 0 degree when not present in results
                results.insert(0,0)

            if self.__write:
                x,y = self.__explore_thread.get_coord()
                self.__write2file([results + [x,y,previous_rssi,new_rssi] + rssi_samples])

            previous_rssi = new_rssi
            it += 1

    def __get_rssi_samples(self,s,nb_samples):
        """Get the RSSI samples used for the RSS gradient estimation. The locations form a square centered
           on the initial position of the robot at the start of this function.[s] is the square and
           [nb_samples] is the number of samples taken for the average RSS"""
        tmp = [] # list used to store all the results of tasks + RSSI samples
        diag = m.sqrt(2*s/2*s/2) # Diagonal of the square
        square_samples = np.zeros((4,1)) # Samples matrix for the gradient computation

        # Go to (-s/2, -s/2)
        self.__explore_thread.turn(135)
        self.__explore_thread.move_straight(diag)
        self.__explore_thread.turn(-135)

        # Get RSSI samples by completing a square and return to the center position
        for i in range(4):
            tmp.extend(self.__explore_thread.wait_finish())
            square_samples[i],samples = self.__rssi_obj.retrieve_rssi(nb_samples)
            tmp.extend(samples)

            if i == 3:
                self.__explore_thread.move_straight(s/2)
                self.__explore_thread.turn(-90)
                self.__explore_thread.move_straight(s/2)
            else:
                self.__explore_thread.move_straight(s)
                self.__explore_thread.turn(-90)

        tmp.extend(self.__explore_thread.wait_finish()) # Make sure all tasks are finished
        return square_samples,tmp

    def __RSS_gradient(self,s,max_it,nb_samples):
        """RSS gradient algorithm. [s] is the square side used for sampling, [max_it] is the number of iterations
           and [nb_samples] is the number of samples taken for the average RSS."""
        sample_coord = np.array([[-s/2, -s/2], # Sample locations, considered fixed (problem with obstacles)
                                 [-s/2,  s/2],
                                 [ s/2,  s/2],
                                 [ s/2, -s/2]])
        pinv_coord = np.linalg.pinv(sample_coord) # Pseudo-inverse of previous matrix

        it = 1
        while it <= max_it:
            print 'Iteration: ', it

            rssi_samples,tmp = self.__get_rssi_samples(s,nb_samples) # Get RSSI samples for gradient computation
            gradient = pinv_coord.dot(rssi_samples)
            print 'Gradient: ', gradient[0], gradient[1]
            if np.linalg.norm(gradient) < 1e-6: # If the gradient is too small, consider direction 0 degree
                gradient = np.array([[0],[1]])

            direction = m.degrees(m.atan2(-gradient[0],gradient[1])) # Compute the direction
            print 'Direction: ', direction

            # Move in the gradient direction
            self.__explore_thread.turn(direction)
            self.__explore_thread.move_straight(2*s)
            results = self.__explore_thread.wait_finish()

            rssi,samples = self.__rssi_obj.retrieve_rssi(nb_samples)
            if rssi > -30: # If the RSSI is strong enough (robot is close to source) -> stop
                it = max_it
                print 'Close to source -> stop'

            if self.__write: # Write data to file if needed
                x,y = self.__explore_thread.get_coord()
                self.__write2file([results + [x,y,gradient[0][0],gradient[1][0]] + rssi_samples.tolist() + tmp + [rssi] + samples])

            it += 1

    ##################################
    ##########  Procedural  ##########
    ##################################

    def _random_direction2(self,d,max_it,nb_samples):
        """Random direction algorithm. [d] is the distance between 2 sampling locations,
           [max_it] is the max number of iterations and [nb_samples] is the number of samples taken
           for the average RSS."""
        previous_rssi = -1000
        
        it = 1
        while it <= max_it:
            print 'Iteration: ', it

            new_rssi,rssi_samples = self.__rssi_obj.retrieve_rssi(nb_samples)
            if new_rssi > -30: # If the RSSI is strong enough (robot is close to source) -> stop
                print 'Close to source -> stop'
                break

            angle = 0
            if new_rssi < previous_rssi:
                angle = self.__explore_thread._turn2(randint(-179,180))
            distance = self.__explore_thread._move_straight2(d)
            
            if self.__write:
                x,y = self.__explore_thread.get_coord()
                self.__write2file([[angle,distance,x,y,previous_rssi,new_rssi] + rssi_samples])

            previous_rssi = new_rssi
            it += 1
    
    def __get_rssi_samples2(self,s,nb_samples):
        """Get the RSSI samples used for the RSS gradient estimation (Procedural). The locations form
           a square centered on the initial position of the robot at the start of this function.[s] is
           the square and [nb_samples] is the number of samples taken for the average RSS"""
        tmp = [] # list used to store all the results of tasks + RSSI samples
        diag = m.sqrt(2*s/2*s/2) # Diagonal of the square
        square_samples = np.zeros((4,1)) # Samples matrix for the gradient computation

        # Go to (-s/2, -s/2)
        tmp.append(self.__explore_thread._turn2(135))
        tmp.append(self.__explore_thread._move_straight2(diag))
        tmp.append(self.__explore_thread._turn2(-135))

        # Get RSSI samples by completing a square and return to the center position
        for i in range(4):
            square_samples[i],samples = self.__rssi_obj.retrieve_rssi(nb_samples)
            tmp.extend(samples)

            if i == 3:
                tmp.append(self.__explore_thread._move_straight2(s/2))
                tmp.append(self.__explore_thread._turn2(-90))
                tmp.append(self.__explore_thread._move_straight2(s/2))
            else:
                tmp.append(self.__explore_thread._move_straight2(s))
                tmp.append(self.__explore_thread._turn2(-90))

        return square_samples,tmp

    def _RSS_gradient2(self,s,max_it,nb_samples):
        """RSS gradient algorithm. [s] is the square side used for sampling, [max_it] is the number of iterations
           and [nb_samples] is the number of samples taken for the average RSS."""
        sample_coord = np.array([[-s/2, -s/2], # Sample locations, considered fixed (problem with obstacles)
                                 [-s/2,  s/2],
                                 [ s/2,  s/2],
                                 [ s/2, -s/2]])
        pinv_coord = np.linalg.pinv(sample_coord) # Pseudo-inverse of previous matrix

        it = 1
        while it <= max_it:
            print 'Iteration: ', it
            results = []
            
            rssi_samples,tmp = self.__get_rssi_samples2(s,nb_samples) # Get RSSI samples for gradient computation

            gradient = pinv_coord.dot(rssi_samples)
            print 'Gradient: ', gradient[0], gradient[1]
            if np.linalg.norm(gradient) < 1e-6: # If the gradient is too small, consider direction 0 degree
                gradient = np.array([[0],[1]])

            direction = m.degrees(m.atan2(-gradient[0],gradient[1])) # Compute the direction
            print 'Direction: ', direction

            # Move in the gradient direction
            results.append(self.__explore_thread._turn2(direction))
            results.append(self.__explore_thread._move_straight2(2*s))

            rssi,samples = self.__rssi_obj.retrieve_rssi(nb_samples)
            if rssi > -30: # If the RSSI is strong enough (robot is close to source) -> stop
                it = max_it
                print 'Close to source -> stop'

            if self.__write: # Write data to file if needed
                x,y = self.__explore_thread.get_coord()
                self.__write2file([results + [x,y,gradient[0][0],gradient[1][0]] + rssi_samples.tolist() + tmp + [rssi] + samples])

            it += 1

    def __init_res2save(self):
        """Initialize output file."""
        timestr = time.strftime("%Y_%m_%d-%H_%M_%S")
        self.__filename = 'roomba' + timestr + '.csv'
        x_start = input('X starting point in mm: ')
        y_start = input('Y starting point in mm: ')

        tmp = [['X start','Y start'],[x_start,y_start]]
        if self.__algo == 1: # If random direction, add corresponding description (not complete though)
            tmp.append(['Angle','Distance','X','Y','Previous RSSI','New RSSI'])
        elif self.__algo == 2: # If RSS gradient, add corresponding description (not complete though)
            tmp.append(['Angle','Distance','X','Y','Gradient X','Gradient Y','Coord 1','Coord 2','Coord 3','Coord 4'])

        with open(self.__filename, 'wb') as f:
            wr = csv.writer(f, quoting=csv.QUOTE_ALL)
            wr.writerows(tmp)

    def __write2file(self,res):
        """Write [res] to output file. [res] must be a list containing lists."""
        with open(self.__filename, 'ab') as f:
            wr = csv.writer(f, quoting=csv.QUOTE_ALL)
            wr.writerows(res)


