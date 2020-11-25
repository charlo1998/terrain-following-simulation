#-------------
import time

#--------------------------------------------------------------------------------------------------------------------------------------
def PID(roll, pitch, yaw, altitude, x, f):
	#Define the global variables to prevent them from dying and resetting to zero, each time a function call occurs. Some of these variables may be redundant.
	global kp_x, ki_x, kd_x, kp_roll, ki_roll, kd_roll, kp_alt, ki_alt, kp_yaw, ki_yaw, kd_yaw
	global prevErr_roll, prevErr_yaw, prevErr_alt, prevErr_x
	global pMem_roll, pMem_yaw, pMem_alt, pMem_x, iMem_roll, iMem_yaw, iMem_alt, iMem_x, dMem_roll,  dMem_yaw, dMem_x
	global flag
	global setpoint, alt_setpoint, x_setpoint, sampleTime
	#-----------------------
	print(flag)
	#Define other variables here, and calculate the errors.
	sampleTime = 0.00001
	setpoint = 0
	alt_setpoint = 1.5
	x_setpoint = 4.0
	err_pitch = float(pitch)*(180 / 3.141592653) - setpoint 
 	err_roll = float(roll)*(180 / 3.141592653) - setpoint
	err_yaw = float(yaw)*(180/3.14159263) - setpoint
	err_alt = -float(altitude) + alt_setpoint
	err_x = float(x) - x_setpoint
	currTime = time.time()
	#-----------------------
	#Reset the following variables during the first run only.
	if flag == 0:
		prevTime = 0
		prevErr_roll = 0
		prevErr_pitch = 0
		prevErr_yaw = 0
		prevErr_alt = 0
		prevErr_x = 0
		pMem_roll = 0
		#pMem_pitch = 0
		pMem_yaw = 0
		pMem_alt = 0
		pMem_x = 0
		iMem_roll = 0
		#iMem_pitch = 0
		iMem_yaw = 0
		iMem_alt = 0
		iMem_x = 0
		dMem_roll = 0
		dMem_x = 0
		dMem_pitch = 0
		dMem_yaw = 0
		flag += 1
		
	#------------------------
	#Define dt, dy(t) here for kd calculations.
	dTime = currTime - prevTime
	#dErr_pitch = err_pitch - prevErr_pitch
	dErr_roll = err_roll - prevErr_roll
	dErr_yaw = err_yaw - prevErr_yaw
	dErr_alt = err_alt - prevErr_alt
	dErr_x = err_x - prevErr_x

	print(err_x)
	print(dErr_x)
	#-------------------------------------------------------------------------------------------------------------------------------
	#This is the Heart of the PID algorithm. PID behaves more accurately, if it is sampled at regular intervals. You can change the sampleTime to whatever value is suitable for your plant.
	if(dTime >= sampleTime):
		#Kp*e(t)
		pMem_roll = kp_roll * err_roll
		#pMem_pitch = kp_pitch * err_pitch
		pMem_yaw = kp_yaw * err_yaw
		pMem_alt = kp_alt * err_alt
		pMem_x = kp_x * err_x
		
		#integral(e(t))
		iMem_roll += err_pitch * dTime
		#iMem_pitch += err_roll * dTime
		iMem_yaw += err_yaw * dTime
		iMem_alt += err_alt * dTime
		iMem_x += err_x * dTime
		
		if(iMem_roll > 400): iMem_roll = 400
		if(iMem_roll < -400): iMem_roll = -400
		#if(iMem_pitch > 400): iMem_pitch = 400
		#if(iMem_pitch < -400): iMem_pitch = -400
		if(iMem_yaw > 400): iMem_yaw = 400
		if(iMem_yaw < -400): iMem_yaw = 400
		if(iMem_alt > 400): iMem_alt = 400
		if(iMem_alt < -400): iMem_alt = 400
		if(iMem_x > 4000): iMem_x = 4000
		if(iMem_x < -4000): iMem_x = 4000
		
		#derivative(e(t))
		dMem_roll = dErr_roll / dTime
		#dMem_pitch = dErr_pitch / dTime
		dMem_yaw = dErr_yaw / dTime
		dMem_x = dErr_x / dTime
	
	#Store the current variables into previous variables for the next iteration.
	prevTime = currTime
	prevErr_roll = err_roll
	#prevErr_pitch = err_pitch
	prevErr_yaw = err_yaw
	prevErr_alt = err_alt
	prevErr_x = err_x
	
	#output = Kp*e(t) + Ki*integral(e(t)) + Kd*derivative(e(t))
	output_roll = pMem_roll + ki_roll * iMem_roll + kd_roll * dMem_roll
	#output_pitch = pMem_pitch + ki_pitch * iMem_pitch + kd_pitch * dMem_pitch
	output_yaw = pMem_yaw + ki_yaw * iMem_yaw + kd_yaw * dMem_yaw
	output_alt = pMem_alt + ki_alt * iMem_alt
	output_x = pMem_x + ki_x * iMem_x + kd_x * dMem_x
	
	
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
	esc_br = 1500 + output_roll - output_yaw + output_alt + output_x
	#bl in my code is br in gazebo's world
	esc_bl = 1500 + output_roll + output_yaw + output_alt - output_x
	#fl in my code is bl in gazebo's world
	esc_fl = 1500 - output_roll - output_yaw + output_alt - output_x
	#fr in my code is fl in gazebo's world
	esc_fr = 1500 - output_roll + output_yaw + output_alt + output_x

	print('motors: ')
	print(esc_br)
	print(esc_bl)
	print(esc_fl)
	print(esc_fr)
	
	#Limit the ESC pulses to upper limit and lower limit, in case the PID algorithm goes crazy and high af.
	if(esc_br > 2000): esc_br = 2000
	if(esc_bl > 2000): esc_bl = 2000
	if(esc_fr > 2000): esc_fr = 2000
	if(esc_fl > 2000): esc_fl = 2000
	
	if(esc_br < 1100): esc_br = 1100
	if(esc_bl < 1100): esc_bl = 1100
	if(esc_fr < 1100): esc_fr = 1100
	if(esc_fl < 1100): esc_fl = 1100
	
	#Map the esc values to motor values
	br_motor_vel = ((esc_br - 1500)/25) + 65
	bl_motor_vel = ((esc_bl - 1500)/25) + 65
	fr_motor_vel = ((esc_fr - 1500)/25) + 65
	fl_motor_vel = ((esc_fl - 1500)/25) + 65
	#----------------------------------------------------------------------------------------------------------------------------------
	#Ignore this shit here.
	'''
	if(fl_motor_vel > 70): fl_motor_vel = 70
	if(fr_motor_vel > 70): fr_motor_vel = 70
	if(bl_motor_vel > 70): bl_motor_vel = 70
	if(br_motor_vel > 70): br_motor_vel = 70
	
	
	if(err_roll > 0 && err_pitch > 0):
		fl_motor_vel = 51
		fr_motor_vel = 45
		bl_motor_vel = 51
		br_motor_vel = 51
	elif(err_roll > 0 && err_pitch < 0):
		fl_motor_vel = 45
		fr_motor_vel = 51
		bl_motor_vel = 51
		br_motor_vel = 51
	elif(err_roll < 0 && err_pitch > 0):
	'''	
	#---------------------------------------------------------------------------------------------------------------------------------
	#Provide the motor velocities to the object 'f' that will now exit out of this function, and gets published to gazebo, providing velocities to each motor. Note that the sign here is +,-,+,- i.e CW, CCW, CW, CCW in gazebo model. Change view of gazebo model (by scrolling) such that the green line comes to your left, red line goes forward, and blue line goes upward. This is the convention that i refer to as "Gazebo model" incase you get confused.
	f.data = [fr_motor_vel,-fl_motor_vel,bl_motor_vel, -br_motor_vel]
	
	#Return these variables back to the control file.
	return f, err_roll, err_pitch, err_yaw
#--------------------------------------------------------------------------------------------------------------------------------------
