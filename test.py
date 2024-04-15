from robomasterpy import RoboMaster
from random import randint

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
rm.move(1, zRotation=90)
