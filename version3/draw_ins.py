# -*- coding: utf-8 -*-

import math
import matplotlib.pyplot as plt

IMG_LENGTH = 600
IMG_WIDTH = 300
OUTPUT_PATH = 'ins_output/'

POT_NUM = 120
class Point():
    def __init__(self, x=0.0, y=0.0, frame=0):
        self.x = x
        self.y = y
        self.frame = frame
        
points = []

def read_file(file_name):
    global points
    with open(file_name, 'r') as file:
        lines = file.readlines()
        for line in lines:
            line_sp = line.split()
            point = Point(x = float(line_sp[1]),
                          y = float(line_sp[2]),
                          frame = int(line_sp[0])
                          )
            
            points.append(point)
    
def get_instruction(waypoints):
    x = []
    y = []
    theta = math.atan2((waypoints[5].y - waypoints[0].y),
                       (waypoints[5].x - waypoints[0].x))
    for i in range(min(len(waypoints)-1, POT_NUM)):
        _x = waypoints[i].x - waypoints[0].x
        _y = waypoints[i].y - waypoints[0].y
        
        new_theta = math.pi/2-theta

        x_ = _x*math.cos(new_theta) - _y*math.sin(new_theta)
        y_ = _y*math.cos(new_theta) + _x*math.sin(new_theta)
        
        x.append(x_)
        y.append(y_)
    
    scale = 20
    fig = plt.figure(figsize=(IMG_LENGTH/100,IMG_WIDTH/100))
    plt.xlim(-scale, scale)
    plt.ylim(0, scale)
    plt.axis('off')
    plt.plot(x,y,"r-",linewidth=50)
    plt.tight_layout()
    fig.savefig(OUTPUT_PATH + '%06d' % waypoints[0].frame + '_ins.png', bbox_inches='tight', dpi=113.4, pad_inches=-0.2)
    plt.close(fig)
    
if __name__ == '__main__':
    read_file('location.txt')
    for index in range(len(points) - POT_NUM):
        get_instruction(points[index:index+POT_NUM])