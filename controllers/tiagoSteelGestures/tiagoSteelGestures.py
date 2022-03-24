###########################################################################
# Social Referencing Disambiguation Gesture Design for Domestic Service Robots
# tiagoSteelGestures.py
# tiagoSteelGestures controller for social referencing gesture 
#
# Kevin Fan
# Electrical and Computer Engineering
# University of Waterloo
# Waterloo, Canada
# k36fan@uwaterloo.ca
############################################################################

from controller import Robot
from controller import Keyboard
# from controller import Speaker
import math
import numpy as np

armJointsNum=7;

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# initialize devices
armJointNames=[]
for i in range(1,8):
    armJointNames.append("arm_%s_joint"%str(i))
headJointNames=['head_1_joint','head_2_joint']
gripperNames=['gripper_left_finger_joint','gripper_right_finger_joint']
torsoName='torso_lift_joint'
wheelNames=['wheel_left_joint','wheel_right_joint']

armParts=[]
headParts=[]
gripperParts=[]
wheels=[]


keyboard=Keyboard()
keyboard.enable(timestep)
# speaker=Speaker("s1")


for i in range(armJointsNum):
    armParts.append(robot.getDevice(armJointNames[i]))
    if not(i>1):
        headParts.append(robot.getDevice(headJointNames[i]))
        gripperParts.append(robot.getDevice(gripperNames[i]))
        wheels.append(robot.getDevice(wheelNames[i]))
torsoPart=robot.getDevice(torsoName)

# speaker.speak("Hello World",1)
def confusionGesture(time):    
    armTarget=[0.07,0.5,-2.85,1.4,2,-1.39,0]
    gripperTarget=[0.025,0.025]
    torsoPart.setPosition(0)
    for i in range(armJointsNum):
        armParts[i].setPosition(armTarget[i])
        if not(i>1):
            gripperParts[i].setPosition(gripperTarget[i])
    armParts[4].setPosition(0.2*math.sin(4*time)+1.87)
            
def lookDownAndScan(time):
     headParts[1].setPosition(-0.25)#look down
     headParts[0].setPosition(0.6*math.sin(2*time))
     
def pointGesture(key):
    if(key==ord('1')):    
        armTarget=[0.5,0.8,-1.1,1.1,1,-0.2,0]#point at the can
        horizontalHead=-0.6#look at the can
    elif(key==ord('2')):
        armTarget=[1,0.75,-1.2,1.1,1,-0.2,0]#point at the beer
        horizontalHead=0#look at the can
    elif(key==ord('3')):
        armTarget=[1.5,0.75,-1.2,1.1,1,-0.2,0]#point at the waterbottle 
        horizontalHead=0.6#look at the can
    else:
        return# do nothing
    gripperTarget=[0,0]
    headParts[1].setPosition(-0.25)#look down
    headParts[0].setPosition(horizontalHead)
    torsoPart.setPosition(0)
    
    for i in range(armJointsNum):
        armParts[i].setPosition(armTarget[i])
        if not(i>1):
            gripperParts[i].setPosition(gripperTarget[i])

def getAttention(time):
    armTarget=[0.07,0.5,-3,1,2,0,1.41]
    #armTarget=[0.07, 0.26, -3.16, 1.27, 1.32, 0.0, 1.41]
    gripperTarget=[0.045,0.045]
    for i in range(armJointsNum):
        armParts[i].setPosition(armTarget[i])
        if not(i>1):
            gripperParts[i].setPosition(gripperTarget[i])
    armParts[1].setPosition(0.2*math.sin(4*time)+0.8)
    armParts[3].setPosition(0.1*math.sin(3*time)+1)
    armParts[5].setPosition(0.35*math.sin(3*time)+0)
    torsoPart.setPosition(0.1*math.sin(3*time)+0.1)
    #headParts[1].setPosition(0.2*math.sin(3*time)+0.1)#look down

def idle():
    armTarget=[0.07,-1.5,0,2,0.5,-1.39,2]
    gripperTarget=[0,0]
    completeFlag=1
    for i in range(armJointsNum):
        armParts[i].setPosition(armTarget[i])
        armParts[i].setVelocity(armParts[i].getMaxVelocity()/2)
        if not(i>1):
            gripperParts[i].setPosition(gripperTarget[i])
            
def objectGrasping():#pick up can
    # delay=1000
    armTarget=[0.53,0.75,-1.1,1.1,1,-0.2,0]
    horizontalHead=-0.6#look at the can
    gripperTarget=[0.045,0.045]
    headParts[1].setPosition(-0.25)#look down
    headParts[0].setPosition(horizontalHead)
    torsoPart.setPosition(0)
    for i in range(armJointsNum):
        armParts[i].setPosition(armTarget[i])
        if not(i>1):
            gripperParts[i].setPosition(gripperTarget[i])
    


    
initialTime=robot.getTime()
# idle()
    
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    time=robot.getTime()-initialTime
    key=keyboard.getKey()
    if (time>0 and time <=8):
        getAttention(time)
    if (time>8 and time <=11.5):
        idle()
    if (time>11.5 and time <=20):
        lookDownAndScan(time)
    if(time>13.5 and time <=20):
        confusionGesture(time)
    if(time>10+10 and time<=12.5+10):
        pointGesture(ord('3'))
    if(time>12.5+10 and time<=13.5+10):
        horizontalHead=np.linspace(0.6,0,10)
        for i in horizontalHead:
            headParts[0].setPosition(i)
        verticalHead=np.linspace(-0.2,0,10)
        for i in verticalHead:
            headParts[1].setPosition(i)#look up
    if(time>13.5+10 and time<=16.5+10):
        headParts[0].setPosition(0.3*math.sin(3*time)+0.3)
        headParts[1].setPosition(-0.125*math.sin(3*time)-0.125)
    if(time>16.5+10 and time<=18.5+10):
        pointGesture(ord('2'))
    if(time>18.5+10 and time<=19.5+10):
        verticalHead=np.linspace(-0.2,0,10)
        for i in verticalHead:
            headParts[1].setPosition(i)#look up
    if(time>19.5+10 and time<=22.5+10):
        headParts[1].setPosition(-0.125*math.sin(3*time)-0.125)
    if(time>22.5+10 and time<=24.5+10):
        pointGesture(ord('1'))
    if(time>24.5+10 and time<=25.5+10):
        horizontalHead=np.linspace(-0.6,0,10)
        for i in horizontalHead:
            headParts[0].setPosition(i)
        verticalHead=np.linspace(-0.2,0,10)
        for i in verticalHead:
            headParts[1].setPosition(i)#look up
    if(time>25.5+10 and time<=28.5+10):
        headParts[0].setPosition(-0.3*math.sin(3*time)-0.3)
        headParts[1].setPosition(-0.125*math.sin(3*time)-0.125)
    
    if(time>38.5 and time <=41.5):
        objectGrasping()
    if(time>41.5 and time <=44.5):
        wheelTarget=[math.pi/1.5,math.pi/1.5]
        wheels[0].setPosition(wheelTarget[0])
        wheels[0].setVelocity(1.5)
        
        wheels[1].setPosition(wheelTarget[1])
        wheels[1].setVelocity(1.5)
    if(time>44.5 and time<=45.5):
        gripperTarget=[0.01,0.01]
        gripperParts[0].setPosition(gripperTarget[0])
        gripperParts[1].setPosition(gripperTarget[1])
    if(time>45.5 and time<=46.5):
        armParts[1].setPosition(0.8)
    if(time>46.5 and time<=48.5):
        wheelTarget=[math.pi/-1.5,math.pi/-1.5]
        wheels[0].setPosition(wheelTarget[0])
        wheels[0].setVelocity(1.5)
        wheels[1].setPosition(wheelTarget[1])
        wheels[1].setVelocity(1.5)
         
        
         
        
    # #getAttention(time)     
    
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.