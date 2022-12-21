from matplotlib import pyplot as plt
from matplotlib import path
import numpy as np
import sys
import csv
import math as m
import utm


def read_rangeinfo(file):
    values = []
    with open(file, newline='\n') as csvfile:
        rps_reader = csv.reader(csvfile, delimiter=",")
        next(rps_reader)
        for row in rps_reader:
            if float(row[5])>0:
                values.append((float(row[2]), float(row[5]),float(row[6])))
    return values

def read_nav(file):
    values = []
    lats = []
    lons = []
    with open(file, newline='\n') as csvfile:
        rps_reader = csv.reader(csvfile, delimiter=",")
        next(rps_reader)
        h = 0
        for row in rps_reader:
            if h != float(row[2]):
                x = utm.from_latlon(float(row[5]),float(row[6]))
                values.append((float(row[2]), x[0],x[1]))
                lats.append(x[0])
                lons.append(x[1])
                h = float(row[2])
    return values,lats,lons

def read_traj(file):
    values = []
    trjx = []
    trjy = []
    with open(file, newline='\n') as csvfile:
        rps_reader = csv.reader(csvfile, delimiter=",")
        next(rps_reader)
        h = 0
        for row in rps_reader:
            values.append((float(row[0]),float(row[1])))
            trjx.append(float(row[1]))
            trjy.append(float(row[0]))
    return values,trjx,trjy

def ranges(rinfo,nav):
    pings1 = []
    pings2 = []
    for r in rinfo:
        for n in nav:
            if n[0]==r[0]:
                if r[2]==1:
                    pings1.append((n[1],n[2],r[1]))
                elif r[2]==2:
                    pings2.append((n[1],n[2],r[1]))
    return pings1,pings2
    


if __name__ == '__main__':
    rinfo = read_rangeinfo('/home/marta/catkin_ws/stalker_2022-10-18-12-21-58-stalker-range_info.csv')
    for i in rinfo:
        print(i[1])
    
    mx= utm.from_latlon(41.778662,3.032922)
    ux= utm.from_latlon(41.779042, 3.032657)
    ux= utm.from_latlon(41.779085, 3.032583)
    print(mx)
    modem = (mx[0],mx[1])
    usbl = (ux[0],ux[1])
    nav,nlats,nlons = read_nav('/home/marta/catkin_ws/stalker_2022-10-18-12-21-58-sparus2-navigator-navigation.csv')
    #print(nav[:][1])
    trj,trjx,trjy = read_traj('/home/marta/catkin_ws/trajectory_opti.csv')
    p1,p2 = ranges(rinfo,nav)
    fig,((ax1,ax4),(ax2,ax3) )= plt.subplots(2,2)
    #for n in nav:
     #   plt.scatter(n[1],n[2], cmap = 'black')
    
    ax1.set_aspect(1)
    #ax1.plot(nlats,nlons,color='k',label='recorregut')
    ax2.plot(nlats,nlons,color='k')
    ax3.plot(nlats,nlons,color='k')
    ax4.plot(nlats,nlons,color='k')
    ax1.plot(trjx,trjy,color = 'k')
    ax1.set_xlabel('meters (m)')
    ax1.set_ylabel('meters (m)')
    ax2.set_xlabel('Latitude (m)')
    ax2.set_ylabel('Longitude (m)')
    ax3.set_xlabel('Latitude (m)')
    ax3.set_ylabel('Longitude (m)')
    ax4.set_xlabel('Latitude (m)')
    ax4.set_ylabel('Longitude (m)')
    ax1.scatter(trjx,trjy,color = 'limegreen',label='est_auv_poses')
    ax1.scatter(trjx[0],trjy[0],color = 'yellow',label='initial pose')
    ax1.scatter(trjx[-1],trjy[-1],color = 'orange',label='final pose')
    ax1.set_aspect(1)
    for o in p1:
        c1 = plt.Circle((o[0],o[1]),o[2],color = 'b',fill = False)
        c2 = plt.Circle((o[0],o[1]),o[2],color = 'b',fill = False)
        ax2.scatter(o[0],o[1], c = 'c')
        ax4.scatter(o[0],o[1], c = 'c')
        ax2.set_aspect(1)
        ax2.add_artist(c1)
        
        #ax4.add_artist(c2)
    for q in p2:
        c3 =plt.Circle((q[0],q[1]),q[2],color = 'r',fill = False)
        c4 =plt.Circle((q[0],q[1]),q[2],color = 'r',fill = False)
        ax3.scatter(q[0],q[1], c = 'm')
        ax4.scatter(q[0],q[1], c = 'm')
        ax3.set_aspect(1)
        ax3.add_artist(c3)
        ax4.set_aspect(1)
        #ax4.add_artist(c4)


    #ax1.scatter(modem[0],modem[1],c = 'red',label='modem')
    #ax1.scatter(usbl[0],usbl[1],c = 'blue',label='usbl')
    ax3.scatter(modem[0],modem[1],c = 'red',label='real position mod2')
    ax2.scatter(usbl[0],usbl[1],c = 'blue',label='real position mod1')

    ax1.scatter(-80.06313304160307,91.42421416743446,c = 'dodgerblue',label='est_mod1')
    ax1.scatter(-57.953351902786004,52.49456121279947,c = 'hotpink',label='est_mod2')

    ax4.scatter(modem[0],modem[1],c = 'red',label='real position mod2')
    ax4.scatter(usbl[0],usbl[1],c = 'blue',label='real position mod1')
    ax1.legend(loc='upper right')
    ax4.legend(loc='upper right')
    ax2.legend(loc='upper right')
    ax3.legend(loc='upper right')
    ax1.set_title('Factor Graph Estimations')
    ax2.set_title('Modem 1 ranges')
    ax3.set_title('Modem 2 ranges')
    ax4.set_title('Real positions')
    fig.suptitle('Results inital test')
    
    plt.show()