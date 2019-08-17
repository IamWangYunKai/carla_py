# -*- coding: utf-8 -*-
import math
import matplotlib.pyplot as plt

points = []

class Point():
    def __init__(self, x=0.0, y=0.0, frame=0):
        self.x = x
        self.y = y
        self.frame = frame
        

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

def calc_length(points):
    dists = []
    for i in range(len(points) - 1):
        dist =math.sqrt(pow(points[i].x - points[i+1].x, 2) + pow(points[i].y - points[i+1].y, 2))
        dists.append(dist)
        
    return sum(dists)
    
    
        
def draw_trajectory(points):
    x = []
    y = []
    for point in points:
        x.append(point.x)
        y.append(point.y)
        
    fig = plt.figure(figsize=(5,5))
    ax = plt.gca()
    ax.set_title('Trajectory of Dataset')
    ax.set_xlabel('x/m')
    ax.set_ylabel('y/m')
    plt.xlim(-185, 285)
    plt.ylim(-235, 235)
    plt.plot(points[0].x, points[0].y, 'ro', alpha=0.8, label="start")
    plt.plot(points[-1].x, points[-1].y, 'go', alpha=0.8, label="end")
    plt.plot(x,y,"b-",linewidth=1, alpha=0.8, label="trajectory")
    plt.legend(loc='best')
    plt.show()
    fig.savefig('trajectory.pdf', bbox_inches='tight')
    plt.close(fig)
    
if __name__ == '__main__':
    read_file('location.txt')
    draw_trajectory(points)
    print('Total lenght of trajectory: %.1f km' % (calc_length(points)/1000))