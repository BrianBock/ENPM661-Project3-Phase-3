# Import Python functions
import cv2 
from datetime import datetime as dtime
import numpy as np

# Import our classes
from maze import Maze
from robot import Robot

# Visualization and Video
# visualization and video are mutually exclusive
write_to_video = False
show_visualization = True
show_solve=True #show every step of the solution to the video/visualization (if True)
solve_frame_interval=100 # Update the visualization/video every this many nodes (higher numbers run faster)

# Allow for user input start/goal coordinates, wheel speeds, and move time
userInput = True

# Precomputed Saves
trySolve=True #Toggle False if you want to use a precomputed save


if write_to_video and show_visualization:
    print("Visualization and video are mutually exclusive. Please change one and try again. Exiting.")
    exit()


# Construct maze object
maze = Maze('maze.txt')
print("Maze created")

# Contstruct the robot
robot = Robot(maze,userInput)
print("Robot created")

# Record the start time so we can compute run time at the end
starttime = dtime.now()

if trySolve:
    # Run Search
    print("Attempting to solve. Please be patient, this may take several minutes")
    robot.A_star()

    if robot.foundGoal:
        searchtime=dtime.now()
        searchtime=searchtime-starttime
        print('Found solution in '+str(searchtime)+' (hours:min:sec)')
        print('Generating path')
        robot.generate_path()
        print('Path generated')

        print('\nPath: ')
        print('The path has ' + str(len(robot.path)) + ' nodes')
        for point in robot.path:
            print(point)

        print('\nPoints from movements:')

        point = robot.path[0]
        for direction in robot.actions:
            # print(direction)
            new_point,d = robot.move(point,direction)
            print(new_point)
            point = new_point

        print('\nMoves: ')
        print('There are ' + str(len(robot.actions)) + ' moves')
        print(robot.actions)


    else:
        print('Unable to find path between start and goal.')
        exit()


# Visualize the path
robot.visualize(write_to_video,show_visualization,show_solve,solve_frame_interval)
endtime = dtime.now()
runtime=endtime-starttime
print("Finished in "+str(runtime)+" (hours:min:sec)")

# new_pint,d=robot.move((4105,-3400,90),"Fast0")
# print(new_pint)
# print(np.deg2rad(new_pint[2]))


