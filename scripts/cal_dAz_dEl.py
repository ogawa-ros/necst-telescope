import cv2
import numpy as np
import matplotlib.pyplot as plt
#import matplotlib.patches as patches
import subprocess
import sys
import os
import pylab
import math
import glob

#---parameters---
#npix_x = 2400   #number of pixcels
#npix_y = 1600
npix_x = 6000   #number of pixcels
npix_y = 4000
#npix_x = 1600   #number of pixcels
#npix_y = 2400

sensor_x = 22.3   #sensor size[mm]
#sensor_y = 14.9

f = 500.   #shoten kyori[mm]
#------------------------------------

#fl = sorted(subprocess.getoutput('find raw/ -name "*.JPG"').split('\n'))
#fl = sorted(subprocess.getoutput("test_picture/*.JPG").split('\n'))
#fl = glob.glob('test_picture/*.JPG')
fl = glob.glob('test_picture/*.JPG')

pix_x = []
pix_y = []
for fl1 in fl:
    img = cv2.imread(fl1, cv2.IMREAD_GRAYSCALE)
    #ret, nimg = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY)
    ret, nimg = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
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
    plt.imshow(cv2.imread(fl1), vmin=0, vmax=256)
    plt.xlim(0, npix_x)
    plt.ylim(0, npix_y)
    plt.plot(stars[idx][0][0], stars[idx][0][1], marker='+')
    #plt.savefig('fig/'+os.path.splitext(os.path.basename(fl1))[0]+'.mark.JPG')
    plt.savefig('fig/'+os.path.splitext(os.path.basename(fl1))[0]+'.mark.png')
    plt.close()
    pix_x.append(stars[idx][0][0])
    pix_y.append(stars[idx][0][1])
pix = np.array([pix_x, pix_y]).T

#np.savetxt('pix.txt', pix, fmt='%i', delimiter=',', header='pix_x, pix_y')
#pix = np.loadtxt('pix.txt', delimiter=',', dtype='int32')
dpix_x = (pix[:,0] - npix_x//2)
dpix_y = (pix[:,1] - npix_y//2)
#dpix_y = (pix[:,1] - npix_y//2)*(-1)

theta_x = 2 * np.degrees(np.arctan(sensor_x / (2*f)))   #[degree]
#theta_y = 2 * np.degrees(numpy.arctan(sensor_y / (2*f)))

theta_x_pix = (theta_x / npix_x) * 3600.   #[arcsec]
#pix_y_to_arcsec = (theta_y / npix_y) * 3600.

#---pixcel --> pix_x_to_arcsec
d_x = dpix_x * theta_x_pix   #[arcsec]
d_y = dpix_y * theta_x_pix   #[arcsec]

d_x_sigma = np.std(d_x)
d_y_sigma = np.std(d_y)

d_x_rms = np.sqrt(np.sum(d_x**2)/len(d_x))
d_y_rms = np.sqrt(np.sum(d_y**2)/len(d_y))

d_rms = np.sqrt(d_x_rms**2 + d_y_rms**2)
d_sigma = np.sqrt(d_x_sigma**2 + d_y_sigma**2)

print('d_rms = %0.2f'%d_rms)
print('d_sigma = %0.2f'%d_sigma)

#---load Az, El---
#AzEl = np.loadtxt('AzEl_list.txt')

###setting parameter manually
Az = [125., 57., 218., 23., 87., 290.]
El = [73., 43., 65., 55., 87., 61.]
#Az = [60., 70.]
#El = [45., 50.]
#d_x = [13., -24., -12., 32., 4., -18.]
#d_y = [-16., -14., 25., -22., 15., 34.]
###

p_array = np.array([Az, El, d_x, d_y]).T
np.savetxt('Az_El_dAz_dEl.dat', p_array, fmt='%i', delimiter=', ')
np.savetxt('Az_El_dAz_dEl.txt', p_array, fmt='%i', delimiter=', ')
#np.savetxt('Az_El_dAz_dEl.txt', txt_for_fitting, fmt='%i', delimiter=',', header='pix_x, pix_y')

def scatter_plot(x, y, xlabel, ylabel):
    plt.figure()
    #plt.axes().set_aspect('equal', 'datalim')
    #plt.xlim(-5000, 5000)
    #plt.ylim(-5000, 5000)
    plt.scatter(x, y, s=5)
    if xlabel[0] == 'd_x' and ylabel[0] == 'd_y':
        plt.title('%s_vs_%s\nrms = %0.2f[arcsec]'%(xlabel[0], ylabel[0], d_rms))
        plt.axes().set_aspect('equal', 'datalim')
        #plt.xlim(-5000, 5000)
        #plt.ylim(-5000, 5000)
        X, Y=[], []
        for num in np.linspace(-180,180,360):
            X.append(5. * math.sin(math.radians(num)))
            Y.append(5. * math.cos(math.radians(num)))
        plt.plot(X, Y)
    elif xlabel[0] == 'Az' and ylabel[0] == 'El':
        plt.title('%s_vs_%s'%(xlabel[0], ylabel[0]))
        #plt.xlim(0, 360)
        #plt.ylim(0, 90)
    elif xlabel[0] == 'Az':
        plt.title('%s_vs_%s'%(xlabel[0], ylabel[0]))
        #plt.xlim(0, 360)
        #plt.ylim(-5000, 5000)
    elif xlabel[0] == 'El':
        plt.title('%s_vs_%s'%(xlabel[0], ylabel[0]))
        #plt.xlim(0, 90)
        #plt.ylim(-5000, 5000)
    else:
        print('use correct label name')

    plt.xlabel('%s [%s]'%xlabel)
    plt.ylabel('%s [%s]'%ylabel)
    plt.grid()
    plt.savefig('%s_vs_%s.png'%(xlabel[0], ylabel[0]))

scatter_plot(Az, El, ('Az', 'degree'), ('El', 'degree'))
scatter_plot(Az, d_x, ('Az', 'degree'), ('d_x', 'arcsec'))
scatter_plot(Az, d_y, ('Az', 'degree'), ('d_y', 'arcsec'))
scatter_plot(El, d_x, ('El', 'degree'), ('d_x', 'arcsec'))
scatter_plot(El, d_y, ('El', 'degree'), ('d_y', 'arcsec'))
scatter_plot(d_x, d_y, ('d_x', 'arcsec'), ('d_y', 'arcsec'))
