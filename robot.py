import cv2
import numpy as np
import math
import time
import os
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from matplotlib.collections import PatchCollection
from matplotlib.patches import Ellipse, Circle, Wedge, Polygon, Arrow
import matplotlib.pyplot as plt
import bisect

class Robot:
    def __init__(self,maze,userInput):
        # Maze/Solver Params
        self.maze = maze
        self.pos_thresh = 15
        self.ang_thresh = 10
        self.goal_radius = 200

        #Robot params
        self.clearance = 15
        self.radius = 177 # Robot radius
        self.wheel_radius=76
        self.L=230 # Wheel distance #http://robotics.caltech.edu/wiki/images/9/9a/CSME133a_Lab2_Instructions.pdf
        self.offset=self.clearance+self.radius
        self.min_speed=1
        self.max_speed=10
        
        self.min_time=.1
        self.max_time=10

        self.maze.generate_constraints(self.offset)

        if userInput:
            self.get_user_nodes()
        else:
            #Easy
            # self.start = (1100,1000,90)
            # self.goal = (2000,1000)
            #Hard
            self.start = (1100,1000,90)
            self.goal = (5000,8160)
            self.fast = 8
            self.slow = 1
            self.move_time=1
            # self.d = 10

        
        

        s_circle = Circle((self.start[0],self.start[1]), self.goal_radius, color='green',alpha=.4)
        self.maze.ax.add_patch(s_circle)
        g_circle = Circle((self.goal[0],self.goal[1]), self.goal_radius, color='red',alpha=.4)
        self.maze.ax.add_patch(g_circle)

        
    def move(self,point,direction):
        x = point[0]
        y = point[1]
        theta = np.deg2rad(point[2])

        if direction == "FastFast":
            ul=self.fast
            ur=self.fast
        elif direction == "FastSlow":
            ul=self.fast
            ur=self.slow
        elif direction == "Fast0":
            ul=self.fast
            ur=0
        elif direction == "SlowFast":
            ul=self.slow
            ur=self.fast
        elif direction == "SlowSlow":
            ul=self.slow
            ur=self.slow
        elif direction == "Slow0":
            ul=self.slow
            ur=0
        elif direction == "0Fast":
            ul=0
            ur=self.fast
        elif direction == "0Slow":
            ul=0
            ur=self.slow


        vx=self.wheel_radius/2*(ul+ur)*math.cos(theta)
        vy=self.wheel_radius/2*(ul+ur)*math.sin(theta)
        vth=self.wheel_radius/self.L*(ur-ul)

        dx=vx*self.move_time
        dy=vy*self.move_time
        phi=vth*self.move_time

        d=math.sqrt(dx**2+dy**2)

        new_x=x+dx*math.cos(theta+phi)
        new_y=y+dy*math.sin(theta+phi)

        new_theta = np.rad2deg(theta+phi)
        if new_theta >= 360 or new_theta<0:
            new_theta = new_theta%360

        new_point = (new_x,new_y,new_theta)

        return new_point, d


    def check_neighbors(self,cur_node):
        directions = ['FastFast','FastSlow','Fast0','SlowFast','SlowSlow','Slow0','0Fast','0Slow']

        neighbors = []
        d_list=[]
        for direction in directions:
            new_point, d = self.move(cur_node,direction)
            if self.maze.in_bounds(new_point):
                if not self.maze.in_obstacle(new_point,self.offset):
                    neighbors.append(new_point)
                    d_list.append(d)

        return neighbors, d_list


    def trunc(self,a,thresh):
        dec_a = a % 1
        int_a = a//1

        if dec_a % thresh < thresh/100:
            trunc_a = int_a + dec_a
        else: 
            for val in np.arange(0,1,thresh):
                if(dec_a-val)<=thresh:
                    if abs(dec_a-(val)) < abs(dec_a-(val+thresh)):
                        trunc_a = int_a+val
                    else:
                        trunc_a = int_a+(val+thresh)
                    break

        return trunc_a

    def arb_round(self,a,thresh):
        remainder=a%thresh
        if remainder<thresh/2:
            # round down
            b=math.floor(a/thresh)
            
        else:
            #round up
            b=math.ceil(a/thresh)
        
        arb_round_a=thresh*b

        return arb_round_a


    def discretize(self,point):
        x=point[0]
        y=point[1]
        theta=point[2]

        if self.pos_thresh<1:
            x = int(self.trunc(x,self.pos_thresh)*(1/self.pos_thresh))
            y = int(self.trunc(y,self.pos_thresh)*(1/self.pos_thresh))
            # if theta < 0:
            #     theta = 360+round(theta)
            theta = int(theta*(1/self.ang_thresh))
        else:
            x = int(self.arb_round(x,self.pos_thresh)*(1/self.pos_thresh))
            y = int(self.arb_round(y,self.pos_thresh)*(1/self.pos_thresh))
            # if theta < 0:
            #     theta = 360+round(theta)
            theta = int(theta*(1/self.ang_thresh))

        new_point=(x,y,theta)
        return new_point


    def A_star(self):

        # each node = (x,y,theta) <- floats
        # nodes = [node1,node2,..,node_n]
        self.nodes = []
        self.nodes.append(self.start)

        # visited_nodes = binary 3D matrix 1 for have visited 0 for haven't
        size_x = int(self.maze.width/self.pos_thresh)
        size_y = int(self.maze.height/self.pos_thresh)
        size_th = int(360/self.ang_thresh)
        visited_nodes = np.zeros((size_x,size_y,size_th))
        start_disc = self.discretize(self.start)
        visited_nodes[start_disc[0],start_disc[1],start_disc[2]] = 1 #set start node as checked

        print(visited_nodes.shape)
        
        # costs = 3D matrix where at discretized point is tuple (cost2come,cost2goal)
        self.costs2come = np.full((visited_nodes.shape[0],visited_nodes.shape[1],visited_nodes.shape[2]),1)
        self.costs2goal = np.full((visited_nodes.shape[0],visited_nodes.shape[1],visited_nodes.shape[2]),1)
        cost2come = 0
        self.costs2come[start_disc[0],start_disc[1],start_disc[2]] = cost2come
        cost2goal = math.sqrt((self.goal[0] - self.start[0])**2 + (self.goal[1] - self.start[1])**2)
        
        # parents = 3D matrix of size h/thresh,w/thresh,360/th_thresh index is ind of parent in nodes
        self.parents = np.full((visited_nodes.shape[0],visited_nodes.shape[1],visited_nodes.shape[2]),0)
        self.parents[start_disc[0],start_disc[1],start_disc[2]] = -1 #set parent of start node to -1
        
        # queue needs to be a list of tuples (node_ind,cost2come+cost2goal)
        queue_inds = [0]
        queue_costs = [cost2come+cost2goal]
    
        self.foundGoal = False 

        while queue_inds:
            # Set the current node as the top of the queue and remove it
            parent = queue_inds.pop(0)
            cost = queue_costs.pop(0)

            cur_node = self.nodes[parent]
            cur_disc = self.discretize(cur_node)

            cost2come = self.costs2come[cur_disc[0],cur_disc[1],cur_disc[2]]

            neighbors, d = self.check_neighbors(cur_node)

            for i, p in enumerate(neighbors):
                cost2goal = math.sqrt((self.goal[0] - p[0])**2 + (self.goal[1] - p[1])**2)
                disc_p = self.discretize(p)
                if visited_nodes[disc_p[0],disc_p[1],disc_p[2]] == 0: 
                    visited_nodes[disc_p[0],disc_p[1],disc_p[2]] = 1
                    self.costs2come[disc_p[0],disc_p[1],disc_p[2]] = cost2come+d[i]
                    self.parents[disc_p[0],disc_p[1],disc_p[2]] = parent
                    self.nodes.append(p)

                    cost_fun = cost2come+d[i]+cost2goal
                    sorted_ind = bisect.bisect_right(queue_costs,cost_fun)
                    queue_inds.insert(sorted_ind,(len(self.nodes)-1))
                    queue_costs.insert(sorted_ind,cost_fun)

                elif cost2come + d[i] < self.costs2come[disc_p[0],disc_p[1],disc_p[2]]:
                    self.costs2come[disc_p[0],disc_p[1],disc_p[2]] = cost2come+d[i]
                    self.parents[disc_p[0],disc_p[1],disc_p[2]] = parent

                if cost2goal<self.goal_radius:
                    self.foundGoal = True
                    queue_inds.clear()
                    break


    def generate_path(self):
        #Assume the last item in nodes is the goal node
        goal = self.nodes[-1]
        disc_goal = self.discretize(goal)
        parent = int(self.parents[disc_goal[0],disc_goal[1],disc_goal[2]])
        path_nodes = [parent]
        while parent != -1:
            node = self.nodes[path_nodes[-1]]
            disc_node = self.discretize(node)
            parent = int(self.parents[disc_node[0],disc_node[1],disc_node[2]])
            path_nodes.append(parent)
        self.path = [goal]
        for ind in path_nodes:
            if ind == -1:
                break
            else:
                self.path.insert(0,self.nodes[ind])


    def switchCoords2Cartesian(self,coordSystem, point):
        x=point[0]
        y=point[1]
        if len(point)==3:
            theta=point[2]

        if coordSystem == "Image":
            new_x=x
            new_y=self.maze.height-y
            

        elif coordSystem == "Cartesian":
            new_x=x
            new_y=y

        elif coordSystem == "Gazebo":
            new_x=self.maze.width/2-x
            new_y=self.maze.height/2-y
            new_point=(new_x,new_y,theta)
        
        if len(point)==3:
            new_point=(new_x,new_y,theta)
        else:
            new_point=(new_x,new_y)

        return new_point


    def get_user_nodes(self):
        needCoordinates=True
        while needCoordinates:
            print('You have 3 options for coordinate systems. Image coordinates have the origin at the top left, with positive y in the downward direction, and positive x in the rightward direction. Cartesian coordinates have the origin in the bottom left, with positive y in the upward direction, and positive x in the rightward direction. Gazebo coordinates have the origin at the center, with positive y in the upward direction, and positive x in the rightward direction. Regardless of your coordinate system choice, the output visualizaion will be shown in Cartesian coordinates.')
            coords=input('Please enter either "image", "cartesian", or "gazebo": ')
            if coords.lower() == 'image' or coords.lower() == 'i':
                needCoordinates=False
                coordSystem="Image"
            elif coords.lower() == 'cartesian' or coords.lower() == 'cart' or coords.lower() == 'c':
                needCoordinates=False
                coordSystem="Cartesian"
            elif coords.lower() == 'gazebo' or coords.lower() == 'g':
                needCoordinates=False
                coordSystem="Gazebo"
            else:
                print("I don't understand what you entered. Please try again.")

        valid_input = False
        while not valid_input:
            valid_pt = False
            while not valid_pt:
                print('Please enter a start point (x,y,theta)')
                start_str_x = input('start x (mm): ')
                start_str_y = input('start y (mm): ')
                start_str_th = input('start theta (deg): ')
                try:
                    start_point = (float(start_str_x),float(start_str_y),int(start_str_th))
                except ValueError:
                    print('The start position coordinates must be numbers. Please try again')
                else:
                    start_point=self.switchCoords2Cartesian(coordSystem,start_point)
                    # Check if start point is valid in maze 
                    if self.maze.in_bounds(start_point):
                        if self.maze.in_obstacle(start_point,0):
                            print("The start point is in an obstacle")
                        else:
                            valid_pt = True
                    else:
                        print("The start point is not valid")

            valid_pt = False
            while not valid_pt:
                print('Please enter a goal point (x,y)')
                goal_str_x = input('goal x (mm): ')
                goal_str_y = input('goal y (mm): ')
                try:
                    goal_point = (float(goal_str_x),float(goal_str_y))
                except ValueError:
                    print('The goal position coordinates must be numbers. Please try again')
                else:
                    goal_point=self.switchCoords2Cartesian(coordSystem,goal_point)
                    # Check if goal point is valid in maze 
                    if self.maze.in_bounds(goal_point):
                        if self.maze.in_obstacle(goal_point,0):
                            print("The goal point is in an obstacle")
                        else:
                            valid_pt = True
                    else:
                        print("The goal point is not valid")

            # Check that start is not goal
            distance = math.sqrt((goal_point[0] - start_point[0])**2 + (goal_point[1] - start_point[1])**2)
            if distance <= self.goal_radius:
                print('The start cannot be within the goal')
            else:
                valid_input = True

        validSpeed1 = False
        while not validSpeed1:
            print('Please enter the first speed that your robot wheels can move at')
            speed1_str = input('Speed 1 (rad/s): ')
            try:
                speed1 = float(speed1_str)
                if self.min_speed <= speed1 <= self.max_speed:
                    validSpeed1 = True
                else:
                    print('The value must be between '+str(self.min_speed)' and '+str(self.max_speed))
            except ValueError:
                print('Wheel speed must be a number. Please try again')

        validSpeed2 = False
        while not validSpeed2:
            print('Please enter the second speed that your robot wheels can move at')
            speed2_str = input('Speed 2 (rad/s): ')
            try:
                speed2 = float(speed2_str)
                if (self.min_speed <= speed2 <= self.max_speed) and speed2 is not speed1:
                    validSpeed2 = True
                elif (self.min_speed <= speed2 <= self.max_speed) and speed2 is speed1:
                    print('Speed 2 cannot be the same as speed 1.')
                else:
                    print('The value must be between '+str(self.min_speed)' and '+str(self.max_speed))
            except ValueError:
                print('Wheel speed must be a number. Please try again')

        validTime = False
        while not validTime:
            print('Please enter the run time - this is how long the wheels will spin for each movement.')
            time_str = input('Run time (s): ')
            try:
                self.move_time = float(time_str)
                if (self.min_time <= self.move_time <= self.max_time):
                    validTime = True
                else:
                    print('The value must be between '+str(self.min_time)' and '+str(self.max_time))
            except ValueError:
                print('Run time must be a number. Please try again')

        self.fast=max(speed1,speed2)
        self.slow=min(speed1,speed2)

        print("Fast speed will be: "+str(self.fast))
        print("Slow speed will be: "+str(self.slow))

        self.start = start_point
        self.goal = goal_point
        # self.d = d


    def plotter(self,start_pos, end_pos,color="black"):
        x_s=start_pos[0]
        y_s=start_pos[1]
        theta=start_pos[2]
        # print("Start")
        # print(x_s, y_s, theta)

        x_f=end_pos[0]
        y_f=end_pos[1]
        # print("end")
        # print(end_pos)

        dx=x_f-x_s
        dy=y_f-y_s
        # head_width=0.5,length_includes_head=True, head_length=0.5,
        arrow = plt.Arrow(x_s, y_s, dx, dy, color=color)

        return arrow

        


    def visualize(self,output,show):
        frame_interval=50
        if output:
            fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
            filename = 'rigid_robot_plot.mp4'
            fps_out = 100

            print('Writing to video. Please Wait.')
            
            if os.path.exists(filename):
                os.remove(filename)
            
            out_plt = cv2.VideoWriter(filename, fourcc, fps_out, (800,800))
        
        canvas = FigureCanvas(self.maze.fig)
        
        count=0
        # Show the searched nodes
        for point in self.nodes:
            # neighborhood,d_list=self.check_neighbors(point)
            # for neighbor in neighborhood:
            disc_node = self.discretize(point)
            parent_node = int(self.parents[disc_node[0],disc_node[1],disc_node[2]])

            if parent_node == -1:
                parent = self.start
            else:
                parent = self.nodes[parent_node]

            arrow = self.plotter(parent,point,color='cyan')
            self.maze.ax.add_artist(arrow)


            self.maze.fig.canvas.draw()
            maze_img = np.frombuffer(self.maze.fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(self.maze.fig.canvas.get_width_height()[::-1] + (3,))
            maze_img = cv2.cvtColor(maze_img,cv2.COLOR_RGB2BGR)

            if output:
                out_plt.write(maze_img)             

            if show:
                if cv2.waitKey(1) == ord('q'):
                    exit()
                cv2.imshow('Visualization',maze_img)

            arrow.remove()
            arrow = self.plotter(parent,point,color='gray')
            self.maze.ax.add_artist(arrow)

        robot_circle=plt.Circle((self.path[0][0],self.path[0][1]), self.offset, color='orange')
        self.maze.ax.add_artist(robot_circle)
        self.maze.fig.canvas.draw()
        maze_img = np.frombuffer(self.maze.fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(self.maze.fig.canvas.get_width_height()[::-1] + (3,))
        maze_img = cv2.cvtColor(maze_img,cv2.COLOR_RGB2BGR)

        if output:          
            out_plt.write(maze_img)

        if show:
            cv2.imshow('Visualization',maze_img)
            if cv2.waitKey(1) == ord('q'):
                exit()

        # Draw the path
        for i in range(len(self.path)-1):
            #Remove the previous circle
            robot_circle.remove()

            # Plot the path arrows
            arrow = self.plotter(self.path[i],self.path[i+1],color='red')
            self.maze.ax.add_artist(arrow)

            # Plot the robot
            robot_circle=plt.Circle((self.path[i+1][0],self.path[i+1][1]), self.offset, color='orange')
            self.maze.ax.add_artist(robot_circle)

            self.maze.fig.canvas.draw()
            maze_img = np.frombuffer(self.maze.fig.canvas.tostring_rgb(), dtype=np.uint8).reshape(self.maze.fig.canvas.get_width_height()[::-1] + (3,))
            maze_img = cv2.cvtColor(maze_img,cv2.COLOR_RGB2BGR)

            if output:          
                out_plt.write(maze_img)

            if show:
                if count>=frame_interval: #Only show every frame_intrval'th frame
                    cv2.imshow('Visualization',maze_img)
                    if cv2.waitKey(1) == ord('q'):
                        exit()
                    if i == len(self.path)-2:
                        cv2.waitKey(0)
                    frame_interval=0

        if output:
            out_plt.release()

        count+=1

            

if __name__ == '__main__':
    
    print("You should't run this program by itself. Please run main.py instead.")
