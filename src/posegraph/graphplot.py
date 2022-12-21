from matplotlib import pyplot as plt
from matplotlib import path
import numpy as np
import sys
import csv
import math as m
import utm
from matplotlib.animation import FuncAnimation
import imageio
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import rospy

class GraphPloter():
    def __init__(self):
        rospy.init_node('depth_controller')
        rospy.Subscriber("/landmarks",
                           	Marker,
                         	self.plotlandmark,
                         	queue_size=1)
        rospy.Subscriber("/robot_optimized_path",
                           	Path,
                         	self.plotpath,
                         	queue_size=1)

        #self.lmvector = []
        #self.pvector = []
        self.newping = False
        #self.ax = ax
        self.n = 0
    def plotlandmark(self,landmark):
        self.lmvector = landmark.points
        print(self.lmvector)
        self.lmx =[]
        self.lmy = []
        for i in range(len(self.lmvector)):
            self.lmx.append(self.lmvector[i].x)
            self.lmy.append(self.lmvector[i].y)
        self.newping = True
        
    def plotpath(self,path):
        self.pvector = path.poses
        print(self.pvector)
        self.px = []
        self.py = []
        for h in range(len(self.pvector)):
            self.px.append(self.pvector[h].pose.position.x)
            self.py.append(self.pvector[h].pose.position.y)
        
        if self.newping:

            #
            

            fig,self.ax = plt.subplots()
            self.ax.scatter(self.lmy,self.lmx,c = 'red')
            self.ax.scatter(self.py,self.px,c = 'blue')
            self.ax.plot(self.py,self.px,c = 'cyan')
            #plt.title(f'Position estimation at step {self.n}')
            #self.ax.set_xlim([-120, 0])
            #self.ax.set_ylim([0, 140])
            
            
            plt.savefig(f'/home/marta/Pictures/plot_{self.n}.png')
            #plt.show()
            #self.ax.cla()
            
            self.storegif()
            self.n +=1
            self.newping = False
    
    def storegif(self,):
        frames = []
            #with imageio.get_writer('mybars.gif', mode='I') as writer:
        for n in range(self.n):
            print(n)
            image = imageio.v2.imread(f'/home/marta/Pictures/plot_{n}.png')
            #writer.append(image)
            frames.append(image)

        imageio.mimsave('/home/marta/Pictures/example4.gif', # output gif
                frames,          # array of input frames
                fps = 5,loop = 1)         # optional: frames per second

if __name__ == '__main__':
    try:
        #fig,ax = plt.subplots()
        graphplot = GraphPloter()
        #plt.show()
        rospy.spin()
    except rospy.ROSInterruptException:
        
        pass