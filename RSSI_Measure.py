#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Author: Zhao CHOW
# Date: May 1 2018
#
# This code is inspired from the code available on https://www.raspberrypi.org/forums/viewtopic.php?t=85601#p604902
# from Hugo Chargois and elParaguayo.

from threading import Thread
import subprocess
import time
import re

class RSSI_Measure(Thread):
    """Thread for retrieving the RSSI value on the R-Pi 3"""
    def __init__(self):
        super(RSSI_Measure, self).__init__()
        self.__running = 1
        self.__interface = "wlan0"
        self.__essid = "dd-wrt"
        # self.__essid = "WiFi-2.4-05C8"

    def run(self):
        """Currently, this function do nothing"""
        while self.__running:
            # NEED TO SLEEP THREAD HERE, OTHERWISE OTHER THREADS' TIME.SLEEP ARE OFF
            # Using only a pass statement is not good, the ressources seem to be allocated to this thread too often
            time.sleep(2)

    def stop(self):
        """Stop the thread"""
        self.__running = 0

    def retrieve_rssi(self,nb_samples):
        """Return the RSSI of essid (specified in __init__). The returned value is the average
           over [nb_samples] samples taken about every second."""
        samples = []
        average = 0

        for i in range(nb_samples):
            tmp = None
            it = 1
            while not isinstance(tmp,int) and it < 100: # Make sure the RSSI value is read, sometimes the interface is busy
                # The 2 lines below are limiting the RSS refresh rate, they take around 1 second
                # tic = time.time()
                proc = subprocess.Popen(["sudo", "iwlist", self.__interface, "scan", "essid", self.__essid],
                    stdout=subprocess.PIPE, universal_newlines=True)
                out, err = proc.communicate()
                # toc = time.time()
                # print "Elapsed time in sec (proc):",toc-tic

                # print out
                out_lines = out.split("\n") # List containing each line of out
                for i in range(len(out_lines)):
                    match = re.search(self.__essid,out_lines[i]) # Search if essid appears in the line
                    # print out_lines[i],match
                    if match: # If there is a match
                        # print match.group(0)
                        index = out_lines[i-2].index("level")
                        tmp = int((out_lines[i-2][index+6:]).strip('dBm ')) # Get RSS value
                        break
                it += 1

            average += tmp
            samples.append(tmp)

            print 'RSSI: ', tmp, 'dBm'
            print "#################################"

        average /= float(nb_samples)
        print 'Average RSSI: ', average, 'dBm'
        print "#################################"
        return average,samples

# ONLY RUN THIS FILE TO TEST RETRIEVING THE RSSI
if __name__ == "__main__":
    nb_samples = input('Nb of RSS samples for averaging?\n')
    rssi_obj = RSSI_Measure()
    i = 0
    while i < 10:
        tic = time.time()
        rssi_obj.retrieve_rssi(nb_samples)
        toc = time.time()
        print "Elapsed time in sec :",toc-tic
        time.sleep(0.02)
        i += 1
