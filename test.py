from robomasterpy import RoboMaster
from random import randint
from time import sleep

rm = RoboMaster()
r = randint(0, 255)
g = randint(0, 255)
b = randint(0, 255)
rm.setLEDs(r, g, b)
rm.turnLeft(90)
rm.turnRight(90)
rm.forward(0.5)
rm.left(0.5)
rm.back(0.5)
rm.right(0.5)
rm.circle(0.25, 1)
# TODO: test how this actually moves, arc or straight line with directional change?
rm.move(1, angle=90)
rm.rotate(45)
rm.stopAfter(3)  # stop after 3 seconds
rm.resetGimbal()
rm.moveGimbal(20, 20)
rm.moveGimbal(-20, -20)
rm.moveGimbalto(180, 90)
rm.fireBlaster(times=3)
for i in range(256):
    rm.setBlasterLED(i)
    sleep(0.01)  # TODO: test how often commands can be sent.
sleep(1)
rm.setBlasterLED(0)
rm.suspendGimbal()
sleep(3)
rm.resumeGimbal()
