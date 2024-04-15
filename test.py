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
input("continue? ")
rm.rotate(45)
rm.stopAfter(3)  # stop after 3 seconds
rm.gun.reset()
rm.gun.move(20, 20)
rm.gun.move(-20, -20)
rm.gun.moveto(180, 90)
rm.gun.fire(times=3)
for i in range(256):
    rm.gun.setLED(i)
    sleep(0.01)  # TODO: test how often commands can be sent.
sleep(1)
rm.gun.setLED(0)
rm.gun.suspend()
sleep(3)
rm.gun.resume()
