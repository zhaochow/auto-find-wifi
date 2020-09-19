Author: Zhao CHOW
Date: May 1 2018


#########################
# main_ProjectRoomba.py #
#########################

This file is the main file which should be run. Various parameters can be specified by the user (input of terminal): which algorithm should be used, the maximum number of iterations, the number of RSS samples to average, should the data be saved in an output file and should we simulate the Roomba.


##################
# Exploration.py #
##################

This file contains all the functions related to the exploration of the environment (control of the Roomba). The main class is Exploration which inherit from Thread to allow multithreading.


###################
# RSSI_Measure.py #
###################

This file contains functions allowing to retrieve the RSS of a specific WiFi network. The main class is RSSI_Measure which inherit from Thread. As of now, threading has not been implemented and this class is only used as a normal class. Future improvements can make use of it.


###################
# Localization.py #
###################

This file contains all functions related to localizing a WiFi source. Currently, 2 algorithms are implemented: random direction and RSS gradient (see function description and report for more details on these algorithms). There are also utility functions for saving the data in a csv output file.


#################
# Roomba_sim.py #
#################

This file contains one class, Roomba_sim, which simulates the Roomba. Only basics functions have been replicated. This class was implemented for testing functions not related to controlling the Roomba. It does not guarantee the good functioning on the real robot.


#######################
# Additional comments #
#######################


Useful Linux commands:

ssh pi@192.168.1.38

(Connect to the Raspberry Pi 3 over SSH from another Linux computer)


scp ~/Documents/MA1/Project/{Exploration,Roomba_sim,RSSI_Measure,main_ProjectRoomba,Localization}.py pi@192.168.1.10:~/Project

(Copy multiple documents from the current computer to the Raspberry Pi 3 over SSH. Adapt if needed)


scp pi@192.168.1.10:~/Project/*.csv ~/Documents/MA1/Project

(Copy all csv files from the Raspberry Pi 3 to the current computer over SSH. Adapt if needed)


