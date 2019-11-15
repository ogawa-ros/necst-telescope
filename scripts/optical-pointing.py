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

name = "optical_pointing"

class optical_pointing(object):

    def __init__(self):
        rospy.init_node("optical_pointing")
        self.catalog_file = "/home/exito/ros/src/necst-telescope/lib/bsc5.dat"
        self.kisa_file = "/home/exito/ros/src/necst-telescope/lib/kisa.dat"

        self.m100_path = "/home/m100raspi/data/optical-pointing/"
        self.data_path = "/home/exito/test/data/optical-pointing/"
        self.pic_path  = "/home/exito/test/data/optical-pointing/picture/"


        self.camera = telescope_controller.camera()
        self.antenna = telescope_controller.antenna()
        self.logger = core_controller.logger()
        pass

    def select_opt_targets(self,reverse=False, obstimedelay=0.0, elmin=20., elmax=90., vmagmin=4, vmagmax=4.5, azmin=0.,azmax=360., pmramax=1, pmdecmax=1,azint=30., show_graph=False):
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
        print('initializing...')
        vmagmin = float(input("vmagmin = " ))
        vmagmax = float(input("vmagmax = " ))
        data = self.select_opt_targets(elmin=20., elmax=90., vmagmin=vmagmin, vmagmax=vmagmax, azmin=270.,azmax=360., pmramax=1, pmdecmax=1,azint=40., show_graph=True)
        star_num = len(data)
        print('generate target star list: %d stars'%(star_num))
        ans = input("1: START optical pointing \n2: Reselect target star\n Select number = ")
        if ans == str(2):
            sys.exit()
        else:
            pass
        start_timestamp = datetime.datetime.today()
        self.data_dir = self.data_path + start_timestamp.strftime('%Y%m%d_%H:%M:%S') + '/'
        os.mkdir(self.data_dir)
        print('create data directory: %s'%(self.data_dir))
        self.pic_dir = self.pic_path + start_timestamp.strftime('%Y%m%d_%H:%M:%S') + '/'
        print('=================== start pointing ======================')
        az = []
        el = []
        pic = []

        date = start_timestamp.strftime('%Y%m%d_%H%M%S')
        file_name = name + '/' + date + '.necstdb'
        print(file_name)
        self.logger.start(file_name)

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

                savefile = self.m100_path + start_timestamp.strftime('%Y%m%d_%H:%M:%S') + "/" +savename
                pre_az = self.antenna.get_az()
                pre_el = self.antenna.get_el()
                self.camera.capture(savefile)
                late_az = self.antenna.get_az()
                late_el = self.antenna.get_el()

                angle = [(pre_az+late_az)/2, (pre_el+late_el)/2]

                print("angle " + str(angle))
                print("captured image")
                print("=========================================")
                az.append(angle[0])
                el.append(angle[1])
                pic.append(savename)
                time.sleep(5)

                continue
        except KeyboardInterrupt:
            self.print('operation INTERRUPTED!')

        self.logger.stop()

        filename = start_timestamp.strftime('%Y%m%d_%H.%M.%S.dat')
        filepath = self.data_dir + filename
        f = open(filepath, "w")
        try:
            for i in range(0, len(data)):
                f.write(str(az[i])+" ")
                f.write(str(el[i])+" ")
                f.write(str(pic[i])+"\n")
                continue
        except IndexError:
            pass
        f.close()
        print('offset data file is created: %s'%(filepath))

        return filepath


    def calc_daz_del(self,filepath):
        npix_x = 6000   #number of pixcels
        npix_y = 4000
        sensor_x = 22.3   #sensor size[mm]
        f = 500.   #shoten kyori[mm]
        #fl = sorted(glob.glob(self.pic_dir+'*.JPG'))
        fl = np.loadtxt(filepath,dtype="unicode").T[2].tolist()
        _Az = np.loadtxt(filepath,dtype="unicode").T[0].tolist()
        Az = [float(i) for i in _Az]
        _El = np.loadtxt(filepath,dtype="unicode").T[1].tolist()
        El = [float(i) for i in _El]
        pix_x = []
        pix_y = []

        for fl1 in fl:
            fl1_fname = os.path.basename(fl1)

            img = cv2.imread(self.data_dir+fl1, cv2.IMREAD_GRAYSCALE)
            print(self.pic_dir+fl1)
            print(img)
            img = np.flipud(img)
            ret, nimg = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY)
            det_img, contours, hierarchy = cv2.findContours(nimg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            stars = []
            areas = []
            for cnt in contours:
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    stars.append(np.array([[cx,cy]], dtype='int32'))
                else:
                    stars.append(np.array([cnt[0][0]], dtype='int32'))
                areas.append(cv2.contourArea(cnt))
            areasarr = np.array(areas)
            idx = areasarr.argmax()
            plt.imshow(np.flipud(cv2.imread(fl1)), vmin=0, vmax=256)
            plt.xlim(0, npix_x)
            plt.ylim(0, npix_y)
            plt.plot(stars[idx][0][0], stars[idx][0][1], marker='+')
            plt.savefig(self.data_dir+os.path.splitext(os.path.basename(fl1))[0]+'.mark.png')
            plt.close()
            pix_x.append(stars[idx][0][0])
            pix_y.append(stars[idx][0][1])
        pix = np.array([pix_x, pix_y]).T

        dpix_x = (pix[:,0] - npix_x//2)
        dpix_y = (pix[:,1] - npix_y//2)

        theta_x = 2 * np.degrees(np.arctan(sensor_x / (2*f)))   #[degree]

        theta_x_pix = (theta_x / npix_x) * 3600.   #[arcsec]

        #---pixcel --> pix_x_to_arcsec
        d_x = dpix_x * theta_x_pix   #[arcsec]
        d_y = dpix_y * theta_x_pix   #[arcsec]

        d_x_sigma = np.std(d_x)
        d_y_sigma = np.std(d_y)

        d_x_rms = np.sqrt(np.sum(d_x**2)/len(d_x))
        d_y_rms = np.sqrt(np.sum(d_y**2)/len(d_y))

        d_rms = np.sqrt(d_x_rms**2 + d_y_rms**2)
        d_sigma = np.sqrt(d_x_sigma**2 + d_y_sigma**2)

        print('d_rms = %0.2f [arcsec]'%d_rms)
        print('d_sigma = %0.2f [arcsec]'%d_sigma)

        p_array = np.array([Az, El, d_x, d_y]).T
        np.savetxt(self.data_dir + 'Az_El_dAz_dEl.txt', p_array, fmt='%i', delimiter=', ')
        self.scatter_plot(Az, El, ('Az', 'degree'), ('El', 'degree'))
        self.scatter_plot(Az, d_x, ('Az', 'degree'), ('dAz', 'arcsec'))
        self.scatter_plot(Az, d_y, ('Az', 'degree'), ('dEl', 'arcsec'))
        self.scatter_plot(El, d_x, ('El', 'degree'), ('dAz', 'arcsec'))
        self.scatter_plot(El, d_y, ('El', 'degree'), ('dEl', 'arcsec'))
        self.scatter_plot(d_x, d_y, ('dAz', 'arcsec'), ('dEl', 'arcsec'))

    def scatter_plot(self,x, y, xlabel, ylabel):
        plt.figure()

        plt.scatter(x, y, s=5)
        if xlabel[0] == 'dAz' and ylabel[0] == 'dEl':
            plt.title('%s_vs_%s\nrms = %0.2f[arcsec]'%(xlabel[0], ylabel[0], d_rms))
            plt.axes().set_aspect('equal', 'datalim')
            X, Y = [], []
            for num in np.linspace(-180,180,360):
                X.append(5. * math.sin(math.radians(num)))
                Y.append(5. * math.cos(math.radians(num)))
            plt.plot(X, Y)
        elif xlabel[0] == 'Az' and ylabel[0] == 'El':
            plt.title('%s_vs_%s'%(xlabel[0], ylabel[0]))

        elif xlabel[0] == 'Az':
            plt.title('%s_vs_%s'%(xlabel[0], ylabel[0]))

        elif xlabel[0] == 'El':
            plt.title('%s_vs_%s'%(xlabel[0], ylabel[0]))

        else:
            print('use correct label name')

        plt.xlabel('%s [%s]'%xlabel)
        plt.ylabel('%s [%s]'%ylabel)
        plt.grid()
        plt.savefig(self.data_dir + '%s_vs_%s.png'%(xlabel[0], ylabel[0]))

    def f_az(self, X, b1, b2, b3, g1):
        Az, El = X
        return (b1 * np.cos(Az*(np.pi/180.))) + (b2 * np.sin(Az*(np.pi/180.))) + b3 + (g1 * El)

    def f_el(self, X, a1, A2, a3):
        Az, El = X
        return (a1 * np.tan(El*(np.pi/180.))) + (A2 / np.cos(El*(np.pi/180.))) + a3 + ((b1 * np.sin(Az*(np.pi/180.)) * np.sin(El*(np.pi/180.)) - b2 * np.cos(Az*(np.pi/180.)) * np.sin(El*(np.pi/180.))) / np.cos(El*(np.pi/180.)))


    def fitting(self):
        txt = np.loadtxt(fname=self.data_dir+'Az_El_dAz_dEl.txt', dtype='int', delimiter=',').T
        Az = txt[0]
        El = txt[1]
        dAz = txt[2]
        dEl = txt[3]

        fit_dEl = curve_fit(self.f_az, (Az, El), dEl)
        b1 = fit_dEl[0][0]
        b2 = fit_dEl[0][1]
        b3 = fit_dEl[0][2]
        g1 = fit_dEl[0][3]

        fit_dAz = curve_fit(self.f_el, (Az, El), dAz)
        a1 = fit_dAz[0][0]
        a2 = fit_dAz[0][1]
        a3 = fit_dAz[0][2]

        a1_deg = ' a1 = ' + str(a1/3600.) + ' [degree]'
        a2_deg = ' a2 = ' + str(a2/3600.) + ' [degree]'
        a3_deg = ' a3 = ' + str(a3/3600.) + ' [degree]'
        b1_deg = ' b1 = ' + str(b1/3600.) + ' [degree]'
        b2_deg = ' b2 = ' + str(b2/3600.) + ' [degree]'
        b3_deg = ' b3 = ' + str(b3/3600.) + ' [degree]'
        g1_deg = ' g1 = ' + str(g1/3600.) + ' [no dimension]'


        a1 = a1/3600. #[degree]
        a2 = a2/3600. #[degree]
        a3 = a3/3600. #[degree]
        b1 = b1/3600. #[degree]
        b2 = b2/3600. #[degree]
        b3 = b3/3600. #[degree]
        g1 = g1/3600. #[no dimension]

        dkisa_list = []
        dkisa_list.append(a1)
        dkisa_list.append(a2)
        dkisa_list.append(a3)
        dkisa_list.append(b1)
        dkisa_list.append(b2)
        dkisa_list.append(a3)
        dkisa_list.append(g1)

        dkisa_array = np.array([a1_sec, a2_sec, a3_sec, b1_sec, b2_sec, b3_sec, g1_sec, ' ', a1_min, a2_min, a3_min, b1_min, b2_min, b3_min, g1_min, ' ', a1_deg, A2_deg, a3_deg, B1_deg, B2_deg, B3_deg, G1_deg]).T
        np.savetxt(self.data_dir +'dkisa.txt', dkisa_array, fmt='%s')
        return dkisa_list

    def apply_kisa(self,dkisa):
        fkisa = open(self.kisa_file,"r")
        kisa = fkisa.readlines()
        a1 = float(kisa[0])+dkisa[0]
        a2 = float(kisa[1])+dkisa[1]
        a3 = float(kisa[2])+dkisa[2]
        b1 = float(kisa[3])+dkisa[3]
        b2 = float(kisa[4])+dkisa[4]
        b3 = float(kisa[5])+dkisa[5]
        g1 = float(kisa[6])+dkisa[6]
        fkisa.close()

        old_kisa_file = self.data_dir + "old_kisa.dat"
        shutil.copy(self.kisa_file,old_kisa_file)

        nkisa = open(self.kisa_file,"w")
        fkisa.write(str(a1)+"\n")
        fkisa.write(str(a2)+"\n")
        fkisa.write(str(a3)+"\n")
        fkisa.write(str(b1)+"\n")
        fkisa.write(str(b2)+"\n")
        fkisa.write(str(b3)+"\n")
        fkisa.write(str(g1)+"\n")
        nkisa.close()

        new_kisa_file = self.data_dir + "new_kisa.dat"
        shutil.copy(self.kisa_file,new_kisa_file)
        print("create new kisa.dat")


if __name__ == "__main__":
    opt = optical_pointing()
    filep = opt.move_target()
    dkisa = opt.calc_daz_del(filep)
    apply_kisa(dkisa)
