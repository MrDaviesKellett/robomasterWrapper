from random import randint
from time import sleep

from robowrap import RoboMaster


robot = RoboMaster()

r = randint(0, 255)
g = randint(0, 255)
b = randint(0, 255)
robot.set_leds(r, g, b)

robot.cam.set_vision_debug(True)
robot.cam.debug_mode = "verbose"

robot.arm.move_to(x=0, z=50)
robot.arm.gripper.open()
sleep(1.5)
robot.arm.move(x=175, z=-250)
robot.arm.move_to(x=175, z=-20)

robot.cam.start()
sleep(0.1)
robot.cam.view()

robot.cam.move_to_marker("1", target_y=-0.1)
while not robot.cam.at_marker:
    robot.cam.view()
    sleep(0.25)

robot.arm.gripper.close()
sleep(1.5)
robot.close()
