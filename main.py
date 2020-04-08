# Import Python functions
import cv2 
from datetime import datetime as dtime

# Import our classes
from maze import Maze
from robot import Robot

# Visualization and Video
# visualization and video are mutually exclusive
write_to_video = True
show_visualization = False
show_solve=True #show every step of the solution to the video/visualization (if True)
solve_frame_interval=100 # Update the visualization/video every this many nodes (higher numbers run faster)

# Allow for user input start/goal coordinates, wheel speeds, and move time
userInput = False

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
        print(robot.path)


        print('\nMoves: ')
        print('There are ' + str(len(robot.path_moves)) + ' moves')
        print(robot.path_moves)


    else:
        print('Unable to find path between start and goal.')
        exit()


# Visualize the path
robot.visualize(write_to_video,show_visualization,show_solve,solve_frame_interval)
endtime = dtime.now()
runtime=endtime-starttime
print("Finished in "+str(runtime)+" (hours:min:sec)")


