
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
cmds = [1, -1]

motors.set_rpm(cmds[0],cmds[1])
f = open("demofile2.txt", "w")

# First loop #######################
start_time = time.time()
while time.time() - start_time < 10:
	try:
		dL, dR, enL, enR = motors.get_wheels_travelled()

		f.write(str(time.time()) + ", " + str(enL) + ", " + str(enR) + "\n")
		#time.sleep(1)
	except KeyboardInterrupt:
		motors.disable_motor()
		break
# Wait 5 seconds#######################
time.sleep(5)
# Second loop #######################
start_time = time.time()
while time.time() - start_time < 10:
	try:
		dL, dR, enL, enR = motors.get_wheels_travelled()

		f.write(str(time.time()) + ", " + str(enL) + ", " + str(enR) + "\n")
		#time.sleep(1)
	except KeyboardInterrupt:
		motors.disable_motor()
		break
###############################
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
