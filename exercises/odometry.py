import sys
sys.path.insert(0, '../lib')
import vrep
import math
import time
import matplotlib.pyplot as plt
import numpy as np

vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5)
if clientID != -1:
    print("Connected to remoteApi server.")
    ret1, motorLeft=vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", vrep.simx_opmode_oneshot_wait)
    ret2, motorRight=vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", vrep.simx_opmode_oneshot_wait)

    ret3, robot_handle = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx", vrep.simx_opmode_oneshot_wait)

    ret4, previousJointPosition=vrep.simxGetJointPosition(clientID,motorLeft,vrep.simx_opmode_streaming)
    ret5, previousJointPositionR=vrep.simxGetJointPosition(clientID,motorRight,vrep.simx_opmode_streaming)
    totalJointPosition=0
    odometry=[0,0]
    ang=0
    ret8, previousPosition=vrep.simxGetObjectPosition(clientID, robot_handle,-1,vrep.simx_opmode_oneshot_wait)
    # function to show the plot 
    plt.ion()
    fig, ax = plt.subplots()
    x, y = [],[]
    # naming the x axis 
    plt.xlabel('x - axis') 
    # naming the y axis 
    plt.ylabel('y - axis') 
    x=[previousPosition[0],previousPosition[0]]
    y=[previousPosition[1],previousPosition[1]]
    # giving a title to my graph 
    plt.title('Odometry vs Ground Truth')   

    plt.xlim(-10,10)
    plt.ylim(-10,10)
    plt.draw()
    color=['blue','red']
    l1=['Truth','Odometry']
    sc =ax.scatter(x[0],y[0],s=5, edgecolors='none', c=color[0], label=l1[0])
    sc =ax.scatter(x[1],y[1],s=5, edgecolors='none', c=color[1], label=l1[1])
    fig.canvas.draw()
    ax.legend()
    plt.show()

##    v0=2
##    vLeft=v0
##    vRight=v0
    
    #diametro da roda = 195mm; comprimento do eixo = 381mm
    radius=0.195/2
    L=0.381/2
    #vrep.simxSetJointTargetVelocity(clientID,motorLeft,0,vrep.simx_opmode_streaming)
    #vrep.simxSetJointTargetVelocity(clientID,motorRight,0,vrep.simx_opmode_streaming)
    while(v0>0):
        jl, actualJointPosition=vrep.simxGetJointPosition(clientID,motorLeft,vrep.simx_opmode_buffer)
        jr, actualJointPositionR=vrep.simxGetJointPosition(clientID,motorRight,vrep.simx_opmode_buffer)
        dxL=actualJointPosition-previousJointPosition
        dxR=actualJointPositionR-previousJointPositionR
        
        if (dxL>=0):
            dxL=math.fmod(dxL+math.pi,2*math.pi)-math.pi
        else:
            dxL=math.fmod(dxL-math.pi,2*math.pi)+math.pi

        
        if (dxR>=0):
            dxR=math.fmod(dxR+math.pi,2*math.pi)-math.pi
        else:
            dxR=math.fmod(dxR-math.pi,2*math.pi)+math.pi
            
        ret4, previousJointPosition=vrep.simxGetJointPosition(clientID,motorLeft,vrep.simx_opmode_oneshot)
        ret5, previousJointPositionR=vrep.simxGetJointPosition(clientID,motorRight,vrep.simx_opmode_oneshot)
        #vrep.simxSetFloatSignal(clientID,"leftEncoder",totalJointPosition,vrep.simx_opmode_oneshot_wait)
        ret9, position=vrep.simxGetObjectPosition(clientID, robot_handle,-1,vrep.simx_opmode_oneshot_wait)
        newposition=math.sqrt(math.pow(previousPosition[0]-position[0],2)+math.pow(previousPosition[1]-position[1],2))
        
        j = [round(dxL,3),round(dxR,3)]
        #print(j)
        if (round(dxR,3) != round(dxL,3)):
          ang=ang-(radius*dxL/(2*L))+(radius*dxR/(2*L))
        else:
          ang=ang
        dist0=(radius*dxL/2)+(radius*dxR/2)

        rotXY=np.dot(np.array([[math.cos(ang),-math.sin(ang)],[math.sin(ang),math.cos(ang)]]),np.array([[dist0],[0]]))
        print(dist0)
        transXY=np.array([[rotXY[0][0]+previousPosition[0]],[rotXY[1][0]+previousPosition[1]]])
        previousPosition=[transXY[0][0],transXY[1][0]]
        x=[position[0],transXY[0][0]]
        y=[position[1],transXY[1][0]]
        color=['blue','red']
        l1=['Truth','Odometry']
        sc =ax.scatter(x,y,s=5, edgecolors='none', c=color, label=l1)
        fig.canvas.draw()
        time.sleep(0.1)
else:
    vrep.simxFinish(clientID)
    sys.exit("\033[91m ERROR: Unable to connect to remoteApi server. Consider running scene before executing script.")
