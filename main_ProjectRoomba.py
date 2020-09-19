#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: Zhao CHOW
# Date: May 1 2018
#
# scp ~/Documents/MA1/Project/{Exploration,Roomba_sim,RSSI_Measure,main_ProjectRoomba,Localization}.py pi@192.168.1.10:~/Project
# scp pi@192.168.1.10:~/Project/*.csv ~/Documents/MA1/Project

import Exploration
import RSSI_Measure
import Localization
import Roomba_sim

def main():
    algo = input('Algorithm? 1 - Random direction, 2 - RSS gradient\n')
    max_it = input('Max iterations?\n')
    nb_samples = input('Nb of RSS samples for averaging?\n')
    mode = input('Mode? 1 - Thread, 2 - Procedural\n')
    write = input('Output file? 1/0 (y/n)\n')

    sim = input('Simulation? 1/0 (y/n)\n')
    roomba_sim = None
    if sim:
        roomba_sim = Roomba_sim.Roomba_sim()
        roomba_sim.daemon = True # Thread is killed when main thread killed
        roomba_sim.start() # Start thread

    explore = Exploration.Exploration(roomba_sim)
    explore.daemon = True # Thread is killed when main thread killed
    explore.start() # Start thread (Needed for procedural too, it activate the control of the robot)
    rssi_obj = RSSI_Measure.RSSI_Measure()
    localize = Localization.Localization(explore,rssi_obj,algo,max_it,nb_samples,mode,write)
    localize.daemon = True # Thread is killed when main thread killed
    localize.start() # Start thread
    print 'Algorithm Start'

    localize.join()
    explore.stop()
    explore.join()

if __name__ == '__main__':
    main()
