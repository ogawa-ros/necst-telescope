#!/usr/bin/env python3

import sys
import ephem
import time
import numpy
import math
import os
import pylab
import datetime
import matplotlib.pyplot as plt
sys.path.append("/home/exito/necst-telescope/scripts")
import telescope_controller
import rospy


class optical_pointing(object):

    def __init__(self):
        rospy.init_node("optical_pointing")
        self.catalog_path = "/home/exito/ros/src/necst-telescope/lib/bsc5.dat"
        self.data_path = "/home/m100raspi/data/optical-pointing/"
        #self.data_path = "/home/exito/test/data/optical-pointing/"
        self.camera = telescope_controller.camera()
        self.antenna = telescope_controller.antenna()
        pass

    def select_opt_targets(self,reverse=False, obstimedelay=0.0, elmin=20., elmax=90., vmagmin=4, vmagmax=4.5, azmin=0.,azmax=360., pmramax=1, pmdecmax=1,azint=30., show_graph=False):
        #from operator import itemgetter
        def itemgetter(item):
            return lambda x: x[item]

        CATALOG_PATH = self.catalog_path
        DATA_PATH = self.data_path
        """Generate optical pointing targets from BSC catalog. """
        catalog_file = CATALOG_PATH

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
        file = open(CATALOG_PATH,"r")
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
                    data.append([line[7:14],ra2000,dec2000,pmra,pmdec,az,math.degrees(yh.alt)])
            except:pass
            continue
        file.close()
        #print(data[0])


        sdata = numpy.array(sorted(data,key=itemgetter(5))) #sort by az
        tmp =sdata[:, 5].astype(numpy.float64)

        print('sdata', tmp)
        ddata=numpy.array([]).reshape(0,7)
        elflag=0
        for azaz in numpy.arange(azmin, azmax,azint):
            print('azmin', azmin, azmax, azint)
            ind = numpy.where((tmp >min(azaz,azaz+azint)) & (tmp< max(azaz,azaz+azint)))
            print('ind', ind)
            print('len ind' ,len(ind))
            dum=sdata[ind[0],:]
            ind2=numpy.argsort(dum[:,6])
            if elflag == 0:
                elflag = 1
            else:
                ind2=ind2[::-1]
                elflag = 0
            ddata = numpy.append(ddata, dum[ind2,:],axis=0)
            continue

        x = ddata[:,5].astype(numpy.float64)
        y = ddata[:,6].astype(numpy.float64)
        if show_graph==True:
            pylab.figure()
            pylab.plot(ddata[:,5],ddata[:,6])
            pylab.grid()
            pylab.xlabel('Az')
            pylab.ylabel('El')
            pylab.title('Optical Pointing Target\nstar num ='+str(len(ddata)))
            pylab.show()

        return ddata

    def move_target(self):
        CATALOG_PATH = self.catalog_path
        DATA_PATH = self.data_path
        print('initializing...')
        data = self.select_opt_targets(elmin=20., elmax=90., vmagmin=1, vmagmax=2, azmin=0.,azmax=360., pmramax=1, pmdecmax=1,azint=40., show_graph=True)
        star_num = len(data)
        print('generate target star list: %d stars'%(star_num))
        ans = input("1: START optical pointing \n2: Reselect target star")
        if ans == 2:
            sys.exit()
        else:
            pass
        start_timestamp = datetime.datetime.today()
        data_dir = DATA_PATH + start_timestamp.strftime('%Y%m%d_%H.%M.%S') + '/'
        os.mkdir(data_dir)
        print('create data directory: %s'%(data_dir))
        print('=================== start pointing ======================')
        az, el = [], []
        try:
            for i in range(0, len(data)):
                #print(float(data[i,1]), float(data[i,2]), data[i,3], data[i,4], data[i,5], data[i,6])
                print("star name : " + data[i,0])
                print('No.%d (%d)' % (i+1, star_num))
                print("RA : " + str(data[i,1]), "DEC : " + str(data[i,2]))

                #self.ctrl.move_antenna_opt(px=data[i,3]/3600.*0, py=data[i,4]/3600*0, acc=3, x=data[i,1]*15., y=data[i,2], coord="J2000")
                self.antenna.move_wcs(float(data[i,1])*15 ,float(data[i,2]))
                self.antenna.tracking_check()

                nowtimestamp = datetime.datetime.today()
                timestr = nowtimestamp.strftime('%Y%m%d_%H.%M.%S')
                savename = timestr +".JPG"

                savepath = self.data_path + savename
                pre_az = self.antenna.get_az()
                pre_el = self.antenna.get_el()
                self.camera.capture(savepath)
                time.sleep(1)
                late_az = self.antenna.get_az()
                late_el = self.antenna.get_el()

                angle = [(pre_az+late_az)/2, (pre_el+late_el)/2]

                print("angle " + str(angle))
                print("captured image")
                print("=========================================")
                az.append(angle[0])
                el.append(angle[1])
                time.sleep(0.1)

                continue
        except KeyboardInterrupt:
            self.print('operation INTERRUPTED!')

        filename = start_timestamp.strftime('%Y%m%d_%H.%M.%S.txt')
        filepath = data_dir + filename
        f = open(filepath, "w")
        try:
            for i in range(0, len(data)):
                f.write(str(az[i])+" ")
                f.write(str(el[i])+"\n")
                #f.write(str(d_az[i])+" ")
                #f.write(str(d_el[i])+"\n")
                continue
        except IndexError:
            pass
        f.close()
        print('offset data file is created: %s'%(filepath))
        #self.ctrl.finalize()


        return filepath


if __name__ == "__main__":
    opt = optical_pointing()
    opt.move_target()
