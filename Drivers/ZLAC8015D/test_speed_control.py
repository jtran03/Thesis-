
from ZLAC8015D import *
import time

motors = ZLAC8015D()

motors.disable_motor()

motors.set_accel_time(1000,1000)
motors.set_decel_time(1000,1000)
#hi
motors.set_mode(3)
motors.enable_motor()

# cmds = [140, 170]
#cmds = [100, 50]
#cmds = [150, -100]
cmds = [5, -5]
start_time = time.time()
f = open("Matlab/demofile2.txt", "w")

# Begin Recording
time_elapsed = time.time() - start_time
while time_elapsed < 5: # Wait 5 seconds
	dL, dR, enL, enR = motors.get_wheels_travelled()
	f.write(str(time_elapsed) + ", " + str(enL) + ", " + str(enR) + "\n")
	print(time_elapsed)

# Set the motor speed
motors.set_rpm(cmds[0],cmds[1])

# Record 5 seconds
while time_elapsed < 10: # Wait 5 seconds
	dL, dR, enL, enR = motors.get_wheels_travelled()
	f.write(str(time_elapsed) + ", " + str(enL) + ", " + str(enR) + "\n")
	time_elapsed = time.time() - start_time
	print(time_elapsed)

# Set the motor speed
motors.set_rpm(0,0)

# Record 5 seconds
while time_elapsed < 15: # Wait 5 seconds
	dL, dR, enL, enR = motors.get_wheels_travelled()
	f.write(str(time_elapsed) + ", " + str(enL) + ", " + str(enR) + "\n")
	time_elapsed = time.time() - start_time
	print(time_elapsed)

# Set motor speed
motors.set_rpm(cmds[1],cmds[0])

# Record 5 seconds
while time_elapsed < 20: # Wait 5 seconds
	dL, dR, enL, enR = motors.get_wheels_travelled()
	f.write(str(time_elapsed) + ", " + str(enL) + ", " + str(enR) + "\n")
	time_elapsed = time.time() - start_time
	print(time_elapsed)

# Close File
f.close()
motors.disable_motor()

# while True:
# 	try:
# 		period = time.time() - start_time
# 		rpmL, rpmR = motors.get_rpm()
#
# 		print("rpmL: {:.1f} | rpmR: {:.1f}".format(rpmL,rpmR))
# 		time.sleep(1)
#
# 		# if (i % 10) == 0:
#
#
# 	except KeyboardInterrupt:
# 		motors.disable_motor()
# 		break
