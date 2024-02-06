import sim
import sys
import time
import math
import numpy as np

PI=math.pi

sim.simxFinish(-1)
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected CoppeliaSim to remote API server')
else:
    print ('Connecting to CoppeliaSim remote API server: Failed')
    sys.exit('Could not connect to CoppeliaSim remote API')


res,left_motor = sim.simxGetObjectHandle(clientID, "./leftMotor", sim.simx_opmode_blocking)
res,right_motor = sim.simxGetObjectHandle(clientID, "./rightMotor", sim.simx_opmode_blocking)

sensor_loc=np.array([-PI/2, -50/180.0*PI,-30/180.0*PI,-10/180.0*PI,10/180.0*PI,30/180.0*PI,50/180.0*PI,PI/2,PI/2,130/180.0*PI,150/180.0*PI,170/180.0*PI,-170/180.0*PI,-150/180.0*PI,-130/180.0*PI,-PI/2]) 


res, ultra_sonic_sensor_Object = sim.simxGetObjectHandle(clientID, "./visible", sim.simx_opmode_blocking)

ultra_sonic_sensors = list(range(16))

sensor_value = np.array([])

for i in range(0, 16, 1):
    res, ultra_sonic_sensors[i] = sim.simxGetObjectChild(clientID, ultra_sonic_sensor_Object, i, sim.simx_opmode_blocking)
    

# set proximitysensor detector

for sensor in ultra_sonic_sensors:
    returnCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sensor, sim.simx_opmode_streaming)
    sensor_value=np.append(sensor_value,np.linalg.norm(detectedPoint))

t = time.time()


while (time.time() - t)<120:
    sensor_value = np.array([]) 
    for sensor in ultra_sonic_sensors:
        retunCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, sensor, sim.simx_opmode_buffer)
        sensor_value = np.append(sensor_value,np.linalg.norm(detectedPoint))
       
    #controller specific
    sensor_square = sensor_value[0:8]*sensor_value[0:8] #square the values of front-facing sensors 1-8
    min_ind = np.where(sensor_square == np.min(sensor_square))
    min_ind=min_ind[0][0]
    #print(sensor_square[min_ind])
    if sensor_square[min_ind] < 0.2:
        steer = -1/sensor_loc[min_ind]
        print(sensor_loc[min_ind])
    else:
        steer= 0
        
  
    forw_velocity = 1	#forward velocity
    kp= 0.5	#steering gain
    left_velocity = forw_velocity + (kp*steer)
    right_velocity = forw_velocity - (kp*steer)
  
    
    res = sim.simxSetJointTargetVelocity(clientID, left_motor, left_velocity, sim.simx_opmode_streaming)
    res = sim.simxSetJointTargetVelocity(clientID, right_motor, right_velocity, sim.simx_opmode_streaming)
    
    time.sleep(0.2)
    
res = sim.simxSetJointTargetVelocity(clientID, left_motor, 0, sim.simx_opmode_streaming)
res = sim.simxSetJointTargetVelocity(clientID, right_motor, 0, sim.simx_opmode_streaming)
 
