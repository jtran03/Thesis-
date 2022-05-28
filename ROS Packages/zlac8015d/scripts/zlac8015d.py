#!/usr/bin/env python
import rospy
import zlac8015d_driver.py
import time

def tester():
	motors = ZLAC8015D.Controller()

	motors.disable_motor()

	motors.set_accel_time(1000,1000)
	motors.set_decel_time(1000,1000)

	motors.set_mode(3)
	motors.enable_motor()

	# cmds = [140, 170]
	#cmds = [100, 50]
	#cmds = [150, -100]
	cmds = [-50, 30]

	motors.set_rpm(cmds[0],cmds[1])

	start_time = time.time()
	i = 0
	while True:
		try:
			period = time.time() - start_time
			rpmL, rpmR = motors.get_rpm()

			print("rpmL: {:.1f} | rpmR: {:.1f}".format(rpmL,rpmR))
			time.sleep(1)

		except KeyboardInterrupt:
			motors.disable_motor()
			break

if __name__ == '__main__':
	try: 
		tester()
	except rospy.ROSInterruptException:
        pass