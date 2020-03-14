import cv2 
import math
import numpy as np
from matplotlib.collections import PatchCollection
from matplotlib.patches import Ellipse, Circle, Wedge, Polygon
import matplotlib.pyplot as plt

class Maze:
    def __init__(self,filename):
        # instatiates an object of class maze
        self.filename = filename

        self.fig,self.ax = plt.subplots(figsize=(12,8))

        major_ticks_x = np.arange(0, 301, 20)
        minor_ticks_x = np.arange(0, 301, 5)
        major_ticks_y = np.arange(0, 201, 20)
        minor_ticks_y = np.arange(0, 201, 5)

        self.ax.set_xticks(major_ticks_x)
        self.ax.set_xticks(minor_ticks_x, minor=True)
        self.ax.set_yticks(major_ticks_y)
        self.ax.set_yticks(minor_ticks_y, minor=True)

        self.ax.grid(which='minor', alpha=0.2)
        self.ax.grid(which='major', alpha=0.5)

        self.ax.set_xlim(0, 300)
        self.ax.set_ylim(0, 200)

        self.ax.set_aspect('equal')

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title("Maze")

        self.patches = []
        self.read_obstacles()        

        for obs in self.obstacles:
            if obs['type'] == 'c': # circle
                self.draw_circle(obs,0,obs['color'])
                
            elif obs['type'] == 'p': # polygon
                self.draw_polygon(obs,0,obs['color'])

            elif obs['type'] == 'e': # ellipse
                self.draw_ellipse(obs,0,obs['color'])
                
            elif obs['type'] == 'rr': # rotate rect
                self.get_rr_points(obs)
                self.draw_polygon(obs,0,obs['color'])
                

        p = PatchCollection(self.patches, alpha=1)
        self.ax.add_collection(p)


    def in_bounds(self,point):
        x = point[0]
        y = point[1]
        if 0<=x<=self.width-1 and 0<=y<=self.height-1:
            return True
        else:
            return False


    def in_obstacle(self,node,offset):
        x = node[0]
        y = node[1]
       
        for obs in self.obstacles:
            if obs['type'] == 'c':
                center = obs['center']
                radius = obs['radius']+ offset
                if ((x-center[0])**2 + (y-center[1])**2 <= radius**2):
                    return True
            elif obs['type'] == 'e':
                center = obs['center']
                a1 = obs['axis'][0]+ offset
                a2 = obs['axis'][1]+ offset
                if ((((x-center[0])**2)/a1**2) + (((y-center[1])**2)/a2**2) <= 1):
                    return True
            elif obs['type'] == 'p' or obs['type'] == 'rr':
                points=obs['points'].copy()
                points.append(points[0])
                contour = np.array(obs['points'], dtype=np.int32)    
                topx,topy,w,h = cv2.boundingRect(contour)
                topx -= offset
                topy -= offset
                w += offset*2
                h += offset*2
                test = (topx+w//2, topy+h//2)

                point_sets = []
                for i in range(len(points)-1):
                    p = np.array([points[i][0],points[i][1],0])
                    q = np.array([points[i+1][0],points[i+1][1],0])
                    v1 = q-p
                    v1_hat = v1/ np.linalg.norm(v1)
                    z = np.array([0,0,1])
                    off_hat = np.cross(z,v1_hat)
                    off_vect = off_hat*offset
                    new_p = p+off_vect
                    new_q = q+off_vect
                    point_sets.append([(new_p[0],new_p[1]),(new_q[0],new_q[1])])

                a,b,c = [],[],[]
                for i in range(len(point_sets)):
                    p1 = point_sets[i][0]
                    p2 = point_sets[i][1]
                    ai=(p1[1]-p2[1])
                    bi=(p2[0]-p1[0])
                    ci=((p1[0]*p2[1])-(p2[0]*p1[1]))
                    if (ai*test[0]+bi*test[1]+ci<0):
                        ai*=-1
                        bi*=-1
                        ci*=-1
                    a.append(ai)
                    b.append(bi)
                    c.append(ci)
                count=0
  
                for i in range(len(a)):
                    if ((a[i]*x + b[i]*y + c[i]) >= 0):
                        count+=1
                    if count==len(a):
                        return True
        return False


    def draw_circle(self,obs,offset,color):
        # Draws a circle on the maze image
        x,y = obs['center']
        radius = obs['radius']
        circle = Circle((x, y), radius)
        self.patches.append(circle)


    def draw_polygon(self,obs,offset,color):
        # Draws a polygon on the maze image
        points = obs['points']
        polygon = Polygon(points, True)
        self.patches.append(polygon)
                

    def draw_ellipse(self,obs,offset,color):
        # Draws an ellipse on the maze image
        x,y = obs['center']
        axis = obs['axis']
        ellipse = Ellipse((x,y),2*axis[0],2*axis[1])
        self.patches.append(ellipse)


    def get_rr_points(self,obs):
        # Write code that modifies that attribute maze to have 1s everywhere inside
        # of the obstacle obs. Should expand the obstacle by the offset, may be easier
        # to just define the points and pass them into define_polygon
        w = obs['width']
        h = obs['height']
        ang1 = math.radians(obs['angle'])
        ang2 = math.radians(90-obs['angle'])
        p1 = obs['start_point']
        p2 = (p1[0]-(w*math.cos(ang1)),p1[1]+(w*math.sin(ang1)))
        p3 = (p1[0]+(h*math.cos(ang2)),p1[1]+(h*math.sin(ang2)))
        p4 = (p3[0]-(w*math.cos(ang1)),p3[1]+(w*math.sin(ang1)))
        points = [p1,p2,p4,p3]
        obs['points'] = points           


    def read_obstacles(self):
        # reads in obstacles from a text file. Supports four types of obstacles,
        # circles,polygons,ellipses,and rotatedrects. Returns a list where each
        # obstacle is a dictionary 
        maze_file = open(self.filename,"r")
        lines = maze_file.readlines()
        default_color = (255,0,0)
        obstacles = []

        for i,line in enumerate(lines):
            if line.split(':')[0].strip() == 'height':
                h = line.split(':')[-1]
                self.height = int(h)
            if line.split(':')[0].strip() == 'width':
                w = line.split(':')[-1]
                self.width = int(w)
            if line == 'circle\n':
                j = i+1
                next_line = lines[j]
                center = radius = None
                color = default_color
                while next_line != '\n':
                    if next_line.split(':')[0].strip() == 'center':
                        p = next_line.split(':')[-1].split(',')
                        center = (int(p[0]),int(p[1]))
                    if next_line.split(':')[0].strip() == 'radius':
                        radius = int(next_line.split(':')[-1])
                    if next_line.split(':')[0].strip() == 'color':
                        bgr = next_line.split(':')[-1].split(',')
                        color = (int(bgr[0]),int(bgr[1]),int(bgr[2]))
                    j += 1
                    if j<len(lines):
                        next_line = lines[j]
                    else:
                        break

                if radius!=None and center!=None:
                    obs = {'type':'c','center':center,'radius':radius,'color':color}
                    obstacles.append(obs)
            
            if line == 'polygon\n':
                points = []
                color = default_color
                j = i+1
                next_line = lines[j]
                while next_line != '\n':
                    if next_line.split(':')[0].strip() == 'point':
                        p = next_line.split(':')[-1].split(',')
                        points.append((int(p[0]),int(p[1])))
                    if next_line.split(':')[0].strip() == 'color':
                        bgr = next_line.split(':')[-1].split(',')
                        color = (int(bgr[0]),int(bgr[1]),int(bgr[2]))
                    j += 1
                    if j<len(lines):
                        next_line = lines[j]
                    else:
                        break

                if len(points)>0:
                    obs = {'type':'p','points':points,'color':color}
                    obstacles.append(obs)
            
            if line == 'ellipse\n': 
                center = axis = None
                angle = start = 0
                end = 360
                color = default_color

                j = i+1
                next_line = lines[j]

                while next_line != '\n':
                    if next_line.split(':')[0].strip() == 'center':
                        p = next_line.split(':')[-1].split(',')
                        center = (int(p[0]),int(p[1]))
                    if next_line.split(':')[0].strip() == 'axis':
                        p = next_line.split(':')[-1].split(',')
                        axis = (int(p[0]),int(p[1]))
                    if next_line.split(':')[0].strip() == 'angle':
                        angle = int(next_line.split(':')[-1])
                    if next_line.split(':')[0].strip() == 'start':
                        radius = int(next_line.split(':')[-1])
                    if next_line.split(':')[0].strip() == 'end':
                        radius = int(next_line.split(':')[-1])
                    if next_line.split(':')[0].strip() == 'color':
                        bgr = next_line.split(':')[-1].split(',')
                        color = (int(bgr[0]),int(bgr[1]),int(bgr[2]))
                    j += 1
                    if j<len(lines):
                        next_line = lines[j]
                    else:
                        break
                
                if radius!=None and axis!=None:
                    obs = {'type':'e','center':center,'axis':axis,'angle':angle,
                           'start':start,'end':end,'color':color}
                    obstacles.append(obs)

            if line == 'rotatedrect\n': 
                start_point = height = width = angle = None
                color = default_color

                j = i+1
                next_line = lines[j]

                while next_line != '\n':
                    if next_line.split(':')[0].strip() == 'start_point':
                        p = next_line.split(':')[-1].split(',')
                        start_point = (int(p[0]),int(p[1]))
                    if next_line.split(':')[0].strip() == 'l1':
                        height = int(next_line.split(':')[-1])
                    if next_line.split(':')[0].strip() == 'l2':
                        width = int(next_line.split(':')[-1])
                    if next_line.split(':')[0].strip() == 'angle':
                        angle = int(next_line.split(':')[-1])
                    if next_line.split(':')[0].strip() == 'color':
                        bgr = next_line.split(':')[-1].split(',')
                        color = (int(bgr[0]),int(bgr[1]),int(bgr[2]))
                    j += 1
                    if j<len(lines):
                        next_line = lines[j]
                    else:
                        break
                
                if start_point!=None and height!=None and width!=None and angle!=None:
                    obs = {'type':'rr','start_point':start_point,'height':height,
                           'width':width,'angle':angle,'color':color}
                    obstacles.append(obs)

        maze_file.close()
        self.obstacles = obstacles

    

if __name__ == '__main__':
    maze = 'maze2'
    mymaze = Maze(maze+'.txt',1)
    cv2.imshow('Maze2',mymaze.image)
    cv2.waitKey(0)
    print(mymaze.in_obstacle((251.5,150),2))