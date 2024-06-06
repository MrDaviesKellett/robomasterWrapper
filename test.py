from robomasterwrapper.robomasterpy import RoboMaster
from random import randint
from time import sleep

# Create a new robomaster object
rm = RoboMaster()
# assign a random colour using r,g,b variables
r = randint(0, 255)
g = randint(0, 255)
b = randint(0, 255)
# set the colour of all LEDs
rm.setLEDs(r, g, b)


rm.cam.setVisionDebug(True)
rm.cam.followSpeed = 2
rm.cam.setPID(330,1,20)
rm.cam.debugMode = True
for i in range(100):
    rm.cam.detectMarker()

rm.close()

exit()

# Turn left by 90 degrees
rm.turnLeft(90)
# turn right by 90 degrees
rm.turnRight(90)
# move forward, left, back and right by half a metre each
rm.forward(0.5)
rm.left(0.5)
rm.back(0.5)
rm.right(0.5)
# move in a circle with a radius of 1/4 of a metre and at 1m/s
rm.circle(0.25, 1)
# continually rotate
rm.rotate(45)
# stop rotating after 3 seconds
rm.stopAfter(3)
# display what's seen by the camera
rm.cam.display()
# move the gun by a given amount from the current position
rm.gun.move(20, 20)
rm.gun.move(-40, -60)
sleep(2) # wait 2 seconds
# move the gun back to it's centre
rm.gun.recenter()
# move the gun to a specific location (always from the origin)
rm.gun.moveto(180, 90)
# fire the gun 3 times
rm.gun.fire(times=3)
# gradually increase the brightness of the gun's light
for i in range(256):
    rm.gun.setLED(i)
    sleep(0.02)  # TODO: test how often commands can be sent.
sleep(1) # wait for a second
# turn off the gun's light
rm.gun.setLED(0)
# recentre the gun
rm.gun.recenter()
# turn off power to the gun
rm.gun.suspend()
sleep(3) # wait 3 seconds
# turn on power to the gun
rm.gun.resume()


