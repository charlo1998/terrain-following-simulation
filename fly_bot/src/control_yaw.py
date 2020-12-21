#!/usr/bin/env python
#---------------------------------------------------
#from pid import PID
import rospy
import message_filters
import math
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
#---------------------------------------------------
#-------------
import time
#--------------------------------------------------------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------------------------------------------------------

def PID_pos(yaw,x,y,u,v):
	global x_setpoint, yaw_setpoint, y_setpoint, roll_setpoint, pitch_setpoint, u_set, v_set
	global kp_x, ki_x, kd_x, kp_y, ki_y, kd_y, kp_u, ki_u, kd_u, kp_v, ki_v, kd_v
	global prevErr_x, prevTime_x, prevErr_y, prevErr_u, prevErr_v
	global pMem_x, iMem_x, dMem_x, pMem_y, iMem_y, dMem_y, pMem_u, iMem_u, dMem_u, pMem_v, iMem_v, dMem_v
	global flag_pos, deltaT, point_a, point_b, point_c
	
	#check time
	currTime = time.time()
	
	#Reset the following variables during the first run only.
	if flag_pos == 0:
		prevTime_x = currTime
		prevErr_x = 0
		prevErr_y = 0
		prevErr_u = 0
		prevErr_v = 0
		pMem_x = 0
		iMem_x = 0
		dMem_x = 0
		pMem_y = 0
		iMem_y = 0
		dMem_y = 0
		pMem_u = 0
		iMem_u = 0
		dMem_u = 0
		pMem_v = 0
		iMem_v = 0
		dMem_v = 0
		deltaT = 0
		point_a = False
		point_b = False
		point_c = False
		x_setpoint = 0
		y_setpoint = 0
		flag_pos += 1


	#Define dt, dy(t) here for kd calculations.
	dTime = currTime - prevTime_x
	deltaT += dTime
	print('Temps ecoule: ' + str(deltaT))

	#calculate errors
	err_x = float(x) - x_setpoint
	err_y = float(y) - y_setpoint	

	#define mission 
	if(deltaT <= 4):
		x_setpoint = 0
		y_setpoint = 0
		point_a = True
	elif(point_a == True):
		x_setpoint = 5.5*math.cos(yaw-math.pi/2)
		y_setpoint = 5.5*math.sin(yaw-math.pi/2)+5
		point_b = True
		if (yaw_setpoint < 358):
			yaw_setpoint += 0.5
			#yaw_setpoint = 270
		else:
			yaw_setpoint = 0



	print('vitesse : ' + str((u**2 + v**2)**(0.5)))
	print('condition de position : ' + str((err_x**2 + err_y**2)**(0.5)))

	#check if we are close, if yes control on position
	if((err_x**2 + err_y**2)**(0.5) <= 1.5):
		#dErr_pitch = err_pitch - prevErr_pitch
		dErr_x = err_x - prevErr_x
		dErr_y = err_y - prevErr_y

		if(dTime >= sampleTime):
			#Kp*e(t)
			pMem_x = kp_x * err_x
			pMem_y = kp_y * err_y
			
			#integral(e(t))
			iMem_x += err_x * dTime
			iMem_y += err_y * dTime
			
			if(iMem_y > 400): iMem_y = 400
			if(iMem_y < -400): iMem_y = 400
			if(iMem_x > 400): iMem_x = 400
			if(iMem_x < -400): iMem_x = 400
			
			#derivative(e(t))
			dMem_x = dErr_x / dTime
			dMem_y = dErr_y / dTime


		#Store the current variables into previous variables for the next iteration.
		prevTime_x = currTime
		prevErr_x = err_x
		prevErr_y = err_y

		#output = Kp*e(t) + Ki*integral(e(t)) + Kd*derivative(e(t))
		output_x = pMem_x + ki_x * iMem_x + kd_x * dMem_x
		output_y = pMem_y + ki_y * iMem_y + kd_y * dMem_y

		pitch_setpoint = -output_x*math.cos(yaw) - output_y*math.sin(yaw)
		roll_setpoint = output_y*math.cos(yaw)  - output_x*math.sin(yaw)
		if(pitch_setpoint > 1): pitch_setpoint = 1
		if(pitch_setpoint < -1): pitch_setpoint = -1
		if(roll_setpoint > 1): roll_setpoint = 1
		if(roll_setpoint < -1): roll_setpoint = -1
	#if too far control on speed
	else:
		u_set = -err_x
		v_set = -err_y
		u_set = u_set / (u_set**2 + v_set**2)**(0.5)
		v_set = v_set / (u_set**2 + v_set**2)**(0.5)
		#reduce speed upon arrival
		if((err_x**2 + err_y**2)**(0.5) <= 3.5):
			u_set = u_set/2
			v_set = v_set/2
		else:
			u_set = u_set/1.2
			v_set = v_set/1.2

		print("u_set: " + str(u_set))
		print("v_set: " + str(v_set))

		err_u = float(u) - u_set
		err_v = float(v) - v_set

		print("err_u: " + str(err_u))
		print("err_v: " + str(err_v))

		#dErr_pitch = err_pitch - prevErr_pitch
		dErr_u= err_u - prevErr_u
		dErr_v = err_v - prevErr_v

		if(dTime >= sampleTime):
			#Kp*e(t)
			pMem_u = kp_u * err_u
			pMem_v = kp_v * err_v
			
			#integral(e(t))
			iMem_u += err_u * dTime
			iMem_v += err_v * dTime
			
			if(iMem_v > 400): iMem_v = 400
			if(iMem_v < -400): iMem_v = 400
			if(iMem_u > 400): iMem_u = 400
			if(iMem_u < -400): iMem_u = 400
			
			#derivative(e(t))
			dMem_u = dErr_u / dTime
			dMem_v = dErr_v / dTime

		#Store the current variables into previous variables for the next iteration.
		prevTime_x = currTime
		prevErr_u = err_u
		prevErr_v = err_v

		#output = Kp*e(t) + Ki*integral(e(t)) + Kd*derivative(e(t))
		output_u = pMem_u + ki_u * iMem_u + kd_u * dMem_u
		output_v = pMem_v + ki_v * iMem_v + kd_v * dMem_v

		pitch_setpoint = -output_u*math.cos(yaw) - output_v*math.sin(yaw)
		roll_setpoint = output_v*math.cos(yaw) - output_u*math.sin(yaw)

		if(pitch_setpoint > 1): pitch_setpoint = 1
		if(pitch_setpoint < -1): pitch_setpoint = -1
		if(roll_setpoint > 1): roll_setpoint = 1
		if(roll_setpoint < -1): roll_setpoint = -1


		print('kp '+ str(pMem_u))
		print('ki '+str(iMem_u*ki_u))
		print('kd '+ str(dMem_u*kd_u))
		print('output u '+str(output_u))
		print('output v '+str(output_v))

	print('Objective :')
	print('x :' + str(x_setpoint))
	print('y :' + str(y_setpoint))

		


def PID(roll, pitch, yaw, altitude, f):
	#Define the global variables to prevent them from dying and resetting to zero, each time a function call occurs. Some of these variables may be redundant.
	global kp_roll, ki_roll, kd_roll, kp_alt, ki_alt, kp_yaw, ki_yaw, kd_yaw
	global prevErr_roll, prevErr_yaw, prevErr_alt, prevErr_pitch, prevTime
	global pMem_roll, pMem_yaw, pMem_alt, pMem_pitch, iMem_pitch, iMem_roll, iMem_yaw, iMem_alt, dMem_roll,  dMem_yaw, dMem_pitch
	global flag_angle
	global yaw_setpoint, roll_setpoint, pitch_setpoint, alt_setpoint, sampleTime
	#-----------------------

	#Define other variables here, and calculate the errors.
	print('ref pitch: ' + str(pitch_setpoint))
	print('ref roll: ' + str(roll_setpoint))
	alt_setpoint = 2.1
	err_pitch = float(pitch)*(180 / 3.141592654) - pitch_setpoint 
 	err_roll = float(roll)*(180 / 3.141592654) - roll_setpoint
	if (yaw >= 0):
		err_yaw = float(yaw)*(180/3.141592654) - yaw_setpoint
	elif (yaw_setpoint >= 180):
		err_yaw = float(yaw)*(180/3.141592654) + 360 - yaw_setpoint
	elif (yaw >= -3.141592654/2):
		err_yaw = float(yaw)*180/3.141592654 - yaw_setpoint
	else:
		err_yaw = float(yaw)*180/3.141592654 + 360 - yaw_setpoint
	err_alt = -float(altitude) + alt_setpoint
	currTime = time.time()
	#-----------------------
	#Reset the following variables during the first run only.
	if flag_angle == 0:
		prevTime = 0
		prevErr_roll = 0
		prevErr_pitch = 0
		prevErr_yaw = 0
		prevErr_alt = 0
		pMem_roll = 0
		pMem_pitch = 0
		pMem_yaw = 0
		pMem_alt = 0
		iMem_roll = 0
		iMem_pitch = 0
		iMem_yaw = 0
		iMem_alt = 0
		dMem_roll = 0
		dMem_pitch = 0
		dMem_yaw = 0
		flag_angle += 1
		
	#------------------------
	#Define dt, dy(t) here for kd calculations.
	dTime = currTime - prevTime
	dErr_pitch = err_pitch - prevErr_pitch
	dErr_roll = err_roll - prevErr_roll
	dErr_yaw = err_yaw - prevErr_yaw
	dErr_alt = err_alt - prevErr_alt

	#-------------------------------------------------------------------------------------------------------------------------------
	#This is the Heart of the PID algorithm. PID behaves more accurately, if it is sampled at regular intervals. You can change the sampleTime to whatever value is suitable for your plant.
	if(dTime >= sampleTime):
		#Kp*e(t)
		pMem_roll = kp_roll * err_roll
		pMem_pitch = kp_pitch * err_pitch
		pMem_yaw = kp_yaw * err_yaw
		pMem_alt = kp_alt * err_alt
		
		#integral(e(t))
		iMem_roll += err_pitch * dTime
		iMem_pitch += err_roll * dTime
		iMem_yaw += err_yaw * dTime
		iMem_alt += err_alt * dTime

		
		if(iMem_roll > 400): iMem_roll = 400
		if(iMem_roll < -400): iMem_roll = -400
		if(iMem_pitch > 400): iMem_pitch = 400
		if(iMem_pitch < -400): iMem_pitch = -400
		if(iMem_yaw > 400): iMem_yaw = 400
		if(iMem_yaw < -400): iMem_yaw = 400
		if(iMem_alt > 400): iMem_alt = 400
		if(iMem_alt < -400): iMem_alt = 400
		
		#derivative(e(t))
		dMem_roll = dErr_roll / dTime
		dMem_pitch = dErr_pitch / dTime
		dMem_yaw = dErr_yaw / dTime
		if(dMem_roll > 1): dMem_roll = 1
		if(dMem_roll < -1): dMem_roll = -1
		if(dMem_pitch > 1): dMem_pitch = 1
		if(dMem_pitch < -1): dMem_pitch = -1
		
	
	#Store the current variables into previous variables for the next iteration.
	prevTime = currTime
	prevErr_roll = err_roll
	prevErr_pitch = err_pitch
	prevErr_yaw = err_yaw
	prevErr_alt = err_alt

	
	
	#output = Kp*e(t) + Ki*integral(e(t)) + Kd*derivative(e(t))
	output_roll = pMem_roll + ki_roll * iMem_roll + kd_roll * dMem_roll
	output_pitch = pMem_pitch + ki_pitch * iMem_pitch + kd_pitch * dMem_pitch
	output_yaw = pMem_yaw + ki_yaw * iMem_yaw + kd_yaw * dMem_yaw
	output_alt = pMem_alt + ki_alt * iMem_alt

	#print('pitch '+ str(pitch))
	#print('output pitch ' + str(output_pitch))
	#print('kp '+ str(pMem_pitch))
	#print('ki '+str(iMem_pitch*ki_pitch))
	#print('kd '+ str(dMem_pitch*kd_pitch))
	#print('err pitch ' + str(err_pitch))
	#print('d_err '+str(dErr_pitch))
	#print('roll '+ str(roll))
	#print('err roll ' + str(err_roll))
	#print('d_err '+str(dErr_roll))
	#print('output roll '+str(output_roll))

	#print('output alt '+str(output_alt))
	#-------------------------------------------------------------------------------------------------------------------------------
	#Some Gazebo information for your reference.
	
	#Positive roll is right wing down
	#Positive pitch is front nose down
	#Positive yaw is rotate CCW about z-axis
	
	#Red is x-axis
	#Green is y-axis
	#Blue is z-axis
	
	#-------------------------------------------------------------------------------------------------------------------------------
	#br: Back Right
	#bl: Back Left
	#fl: Front Left
	#fr: Front Right
	#Calculate the ESC pulses (1000us - 2000us PWM signal) for each of the motor.
	#default 1500
	
	#br in my code is fr in gazebo's world
	esc_br = 1500 + output_roll - output_yaw + output_alt + output_pitch
	#bl in my code is br in gazebo's world
	esc_bl = 1500 + output_roll + output_yaw + output_alt - output_pitch
	#fl in my code is bl in gazebo's world
	esc_fl = 1500 - output_roll - output_yaw + output_alt - output_pitch
	#fr in my code is fl in gazebo's world
	esc_fr = 1500 - output_roll + output_yaw + output_alt + output_pitch

	
	
	#Limit the ESC pulses to upper limit and lower limit, in case the PID algorithm goes crazy and high af.
	if(esc_br > 2000): esc_br = 2000
	if(esc_bl > 2000): esc_bl = 2000
	if(esc_fr > 2000): esc_fr = 2000
	if(esc_fl > 2000): esc_fl = 2000
	
	if(esc_br < 1100): esc_br = 1100
	if(esc_bl < 1100): esc_bl = 1100
	if(esc_fr < 1100): esc_fr = 1100
	if(esc_fl < 1100): esc_fl = 1100

	
	print('ref yaw: ' + str(yaw_setpoint))
	print('yaw ' + str(yaw))
	print('kp '+ str(pMem_yaw))
	print('ki '+str(iMem_roll*ki_yaw))
	print('kd '+ str(dMem_roll*kd_yaw))
	print('output yaw '+str(output_yaw))	
	print('motors:  ')
	print(esc_br)
	print(esc_bl)
	print(esc_fl)
	print(esc_fr)


	#Map the esc values to motor values
	br_motor_vel = ((esc_br - 1500)/25) + 65
	bl_motor_vel = ((esc_bl - 1500)/25) + 65
	fr_motor_vel = ((esc_fr - 1500)/25) + 65
	fl_motor_vel = ((esc_fl - 1500)/25) + 65
	
	#---------------------------------------------------------------------------------------------------------------------------------
	#Provide the motor velocities to the object 'f' that will now exit out of this function, and gets published to gazebo, providing velocities to each motor. Note that the sign here is +,-,+,- i.e CW, CCW, CW, CCW in gazebo model. Change view of gazebo model (by scrolling) such that the green line comes to your left, red line goes forward, and blue line goes upward. This is the convention that i refer to as "Gazebo model" incase you get confused.
	f.data = [fr_motor_vel,-fl_motor_vel,bl_motor_vel, -br_motor_vel]
	
	#Return these variables back to the control file.
	return f, err_roll, err_pitch, err_yaw

#--------------------------------------------------------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------------------------------------------------------


#Assign your PID values here. From symmetry, control for roll and pitch is the same.
kp_roll = 70
ki_roll = 0.0002
kd_roll = 89
kp_pitch = kp_roll
ki_pitch = ki_roll
kd_pitch = kd_roll
kp_yaw = 6
ki_yaw = 0 #was 0
kd_yaw = 0	#was 0
kp_alt = 200
ki_alt = 0.001
kp_x = 2
ki_x = kp_x/700
kd_x = 2.5
kp_y = kp_x
ki_y = ki_x
kd_y = kd_x
kp_u = 10
ki_u = kp_x/1000
kd_u = 0
kp_v = kp_u
ki_v = ki_u
kd_v = kd_u


yaw_setpoint = 0
roll_setpoint = 0
pitch_setpoint = 0
flag_angle = 0
flag_pos = 0
sampleTime = 0.00001


def control_kwad(pose, sensor, args):
	#Declare global variables as you dont want these to die, reset to zero and then re-initiate when the function is called again.
	global roll, pitch, yaw, altitude, err_roll, err_pitch, err_yaw
	
	#Assign the Float64MultiArray object to 'f' as we will have to send data of motor velocities to gazebo in this format
	f = Float64MultiArray()
	
	#Convert the quaternion data to roll, pitch, yaw data
	#The model_states contains the position, orientation, velocities of all objects in gazebo. In the simulation, there are objects like: ground, Contruction_cone, quadcopter (named as 'Kwad') etc. So 'msg.pose[ind]' will access the 'Kwad' object's pose information i.e the quadcopter's pose.
	ind = pose.name.index('Kwad')
	print('ind: ' + str(ind))
	orientationObj = pose.pose[ind].orientation
	orientationList = [orientationObj.x, orientationObj.y, orientationObj.z, orientationObj.w]
	(roll, pitch, yaw) = (euler_from_quaternion(orientationList))

	speedObj = pose.twist[ind].linear
	u = speedObj.x
	v = speedObj.y

	positionObj = pose.pose[ind].position
	#altitude = positionObj.z
	x = positionObj.x
	y = positionObj.y
	print('x: ' + str(x) + 'y: ' + str(y))
	sensorObj = sensor.ranges
	altitude = min(sensorObj)
	if (altitude > 300):
		altitude = 0
	
	

	#send roll, pitch, yaw data to PID() for attitude-stabilisation, along with 'f', to obtain 'fUpdated'
	#Alternatively, you can add your 'control-file' with other algorithms such as Reinforcement learning, and import the main function here instead of PID().
	PID_pos(yaw,x,y,u,v)
	(fUpdated, err_roll, err_pitch, err_yaw) = PID(roll, pitch, yaw, altitude, f)
	
	#The object args contains the tuple of objects (velPub, err_rollPub, err_pitchPub, err_yawPub. publish the information to namespace.
	args[0].publish(fUpdated)
	args[1].publish(err_roll)
	args[2].publish(err_pitch)
	args[3].publish(err_yaw)
	#print("Roll: ",roll*(180/3.141592653),"Pitch: ", pitch*(180/3.141592653),"Yaw: ", yaw*(180/3.141592653))
	#print(orientationObj)
#----------------------------------------------------

#Initiate the node that will control the gazebo model
rospy.init_node("Control")

#initiate publishers that publish errors (roll, pitch,yaw - setpoint) so that it can be plotted via rqt_plot /err_<name>  
err_rollPub = rospy.Publisher('err_roll', Float32, queue_size=1)
err_pitchPub = rospy.Publisher('err_pitch', Float32, queue_size=1)
err_yawPub = rospy.Publisher('err_yaw', Float32, queue_size=1)

#initialte publisher velPub that will publish the velocities of individual BLDC motors
velPub = rospy.Publisher('/Kwad/joint_motor_controller/command', Float64MultiArray, queue_size=4)

#Subscribe to /gazebo/model_states to obtain the pose in quaternion form
#Upon receiveing the messages, the objects msg, velPub, err_rollPub, err_pitchPub and err_yawPub are sent to "control_kwad" function.
PoseSub = message_filters.Subscriber('/gazebo/model_states', ModelStates)
SensorSub = message_filters.Subscriber('/Kwad/laser/scan', LaserScan)

ts = message_filters.ApproximateTimeSynchronizer([PoseSub, SensorSub], 10, 0.001, allow_headerless=True)
ts.registerCallback(control_kwad, (velPub, err_rollPub, err_pitchPub, err_yawPub))


rospy.spin()
