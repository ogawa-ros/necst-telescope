#!/usr/bin/env python3

import sys
import ephem
import time
import numpy
import math
import os
import pylab
import datetime
import shutil
import matplotlib.pyplot as plt
sys.path.append("/home/exito/necst-telescope/scripts")
import telescope_controller
sys.path.append("/home/exito/necst-core/scripts")
import core_controller
import rospy
from scipy.optimize import curve_fit
import numpy as np
import cv2
import glob

name = "brightest_star"

class brightest_star(object):

    def __init__(self):
        rospy.init_node(name)
        self.catalog_file = "/home/exito/ros/src/necst-1p85m2019/lib/bsc5.dat"
        #self.camera = telescope_controller.camera()
        self.antenna = telescope_controller.antenna()
        self.logger = core_controller.logger()
        pass

    def select_brightest_star(self,reverse=False, obstimedelay=0.0, elmin=20., elmax=90., vmagmin=0, vmagmax=3, azmin=0.,azmax=360., pmramax=1, pmdecmax=1,azint=30., show_graph=True):
        #from operator import itemgetter
        def itemgetter(item):
            return lambda x: x[item]

        """Generate optical pointing targets from BSC catalog. """
        if reverse == True:
            azint=-azint
            (azmin,azmax)=(azmax,azmin)

        nro = ephem.Observer()
        nro.long='138.472153'
        nro.lat='35.940874'
        nro.elevation = 1386.0
        nro.compute_pressure()
        nro.date=ephem.now()+obstimedelay

        data = []
        file = open(self.catalog_file,"r")
        for line in file.readlines():
            try:
                ra2000 = float(line[75:77]) + float(line[77:79]) / 60. + float(line[79:83])/3600.
                dec2000 = float(line[84:86]) + float(line[86:88]) / 60. + float(line[88:90]) / 3600.
                if line[83:84] == '-': dec2000 = -dec2000
                multiple = line[43:44]
                vmag = float(line[103:107])
                #print 'vmag', vmag
                pmra = float(line[149:154])
                pmdec =float( line[154:160])
                #print(a, ra2000,dec2000,pmra,pmdec)
                eline="te|te|te,f|D|G3,%d:%d:%.1f|%.2f,%s%d:%d:%d|%.1f,%f.2,2000" % (float(line[75:77]),float(line[77:79]),float(line[79:83]),pmra*1000.,line[83:84],float(line[84:86]),float(line[86:88]),float(line[88:90]),pmdec*1000.,vmag)
                yh = ephem.readdb(eline)
                yh.compute(nro)
                if multiple == ' ' and math.degrees(yh.alt) >= elmin and math.degrees(yh.alt) <= elmax and vmag >= vmagmin and vmag <= vmagmax and pmra<=pmramax and pmdec<=pmdecmax:
                    if math.degrees(yh.az) < min(azmin,azmax):
                        az = math.degrees(yh.az) + 360.
                    elif math.degrees(yh.az) > max(azmin,azmax):
                        az = math.degrees(yh.az) - 360.
                    else:
                        az = math.degrees(yh.az)
                    data.append([line[7:14],ra2000,dec2000,pmra,pmdec,az,math.degrees(yh.alt),vmag])
            except:pass
            continue
        file.close()

        sdata = numpy.array(sorted(data,key=itemgetter(7)))[0] #sort by vmag
        print("Name = "  +sdata[0])
        print(" Ra  = "  +sdata[1])
        print("Dec  = "  +sdata[2])
        print(" Az  = "  +sdata[5])
        print(" El  = "  +sdata[6])
        print("Vmag = "  +sdata[7])
        x = sdata[5].astype(numpy.float64)
        y = sdata[6].astype(numpy.float64)
        if show_graph==True:
            pylab.figure()
            pylab.plot(x,y,"*")
            pylab.grid()
            pylab.xlabel('Az')
            pylab.ylabel('El')
            pylab.title('Brightest star target')
            pylab.show()

        return sdata


    def move_target(self):
        data = self.select_brightest_star(elmin=15., elmax=85., vmagmin=0, vmagmax=3, azmin=5.,azmax=355., pmramax=1, pmdecmax=1,azint=40., show_graph=True)
        start_timestamp = datetime.datetime.today()
        date = start_timestamp.strftime('%Y%m%d_%H%M%S')
        file_name = name + '/' + date + '.necstdb'
        print(file_name)
        self.logger.start(file_name)

        self.antenna.move_wcs(float(data[1])*15 ,float(data[2]))
        self.antenna.tracking_check()

        time.sleep(5)

        self.logger.stop()

        return


if __name__ == "__main__":
    star = brightest_star()
    filep = star.move_target()
