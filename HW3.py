#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for RBE 595/CS 525 Motion Planning
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
draw=[]
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

def split_path(path):
    path = path.split('\n')
    for i in xrange(len(path)):
        path[i] = path[i].split(',')
        for j in xrange(len(path[i])):
            path[i][j] = float(path[i][j])
    return path

def draw_path(path, color):
    for i in path:
        robot.SetActiveDOFValues(i)
        draw.append(env.plot3(points=robot.GetLinks()[49].GetTransform()[0:3,3],pointsize=0.02,colors=color,drawstyle=1))

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('hw3.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]
    ### INITIALIZE YOUR PLUGIN HERE ###
    RaveInitialize()
    RaveLoadPlugin('build/rrtplugin')
    rrtmodule = RaveCreateModule(env,'rrtmodule')
    ### END INITIALIZING YOUR PLUGIN ###


    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    #set start config
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
    startconfig = [-0.15,0.075,-1.008,-0.11,0,-0.11,0]
    robot.SetActiveDOFValues(startconfig);
    robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

    with env:
        goalconfig = [0.449,-0.201,-0.151,-0.11,0,-0.11,0]

        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig
        # biDirectional decides whether to implement RRTConnect or biDirectional RRTConnect
        # Pass 0 for RRTConnect
        # Pass 1 for biDirectional RRTConnect
        #file = open("Testfile.txt", "w")
        #for i in range(1,101,5):
        #file.write("Goal Bias: " + str(i/100.0) + "\n")
        #file.flush()
        biDirectional = 0
        command = "RRT " + str(biDirectional) + " "

        startconfig = repr(startconfig).strip('[').strip(']')
        command = command + startconfig + " "

        goalconfig = repr(goalconfig).strip('[').strip(']')
        command = command + goalconfig + " "

        #goal_bias = i/100.0
        goal_bias = 0.15
        goal_bias = repr(goal_bias)
        command = command + goal_bias + " "

        step_size = 0.3
        step_size = repr(step_size)
        command = command + step_size + " "

        lmodel = databases.linkstatistics.LinkStatisticsModel(robot)
        if not lmodel.load():
            lmodel.autogenerate()
        lmodel.setRobotWeights()
        lmodel.setRobotResolutions(xyzdelta=0.01)
        I = [robot.GetJoint(name).GetDOFIndex() for name in jointnames]
        weights = repr(list(robot.GetDOFWeights(I))).strip('[').strip(']')
        command = command + weights + " "

        lower, upper = robot.GetActiveDOFLimits()
        lower[4], lower[6] = 0,0
        upper[4], upper[6] = 0,0
        lower = repr(list(lower)).strip('[').strip(']')
        upper = repr(list(upper)).strip('[').strip(']')
        command = command + lower + " " + upper + " "

        print rrtmodule.SendCommand(command)

        #for j in range(10):
        start_time = time.time()
        path = rrtmodule.SendCommand("Start")
        print("Time taken to compute path: %s seconds" % (time.time() - start_time))
        #file.write("Run " + str(j) + ": " + str(time.time() - start_time) + "\n")
        #file.flush()
        #file.close()
        path = split_path(path)

        draw_path(path, [1, 0, 0])

        start_time = time.time()
        smooth_path = rrtmodule.SendCommand("Smooth")
        print("Time taken to smoothen path: %s seconds" % (time.time() - start_time))

        smooth_path = split_path(smooth_path)
        draw_path(smooth_path, [0, 0, 1])

        a = raw_input("Enter 1")

        trajectory = RaveCreateTrajectory(env,'')
        trajectory.Init(robot.GetActiveConfigurationSpecification())
        for i in xrange(len(smooth_path)):
    		trajectory.Insert(i,smooth_path[i])
        planningutils.RetimeActiveDOFTrajectory(trajectory,robot,hastimestamps=False,maxvelmult=1,maxaccelmult=1)
        print 'Duration of trajectory =',trajectory.GetDuration()

        robot.GetController().SetPath(trajectory)
        ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")
