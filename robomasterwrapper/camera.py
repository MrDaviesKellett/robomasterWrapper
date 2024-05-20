import cv2

class Camera:
    def __init__(self, RoboMaster) -> None:
        self.robomaster = RoboMaster
        self.robot = self.robomaster.robot
        self.camera = self.robot.camera
        self.vision = self.robot.vision
        self.streaming = False
        self.detectMode = "line"
        self.visionDebug = False
        self.debugColor = (0,255,0)
        self.resolution = "360p"
        self.width = 640
        self.height = 360
        self.debugMode = False
        self.__debugList = []

    # Camera Display
    def setResolution(self, resolution="360P"):
        if not resolution in ['360p','540p','720p']:
            print("Invalid resolution")
            print("Please choose from the following options:")
            print("360p, 520p, 720p")
        self.resolution = resolution.lower()
        if self.resolution == '360p':
            self.width = 640
            self.height = 360
        elif self.resolution == '520p':
            self.width = 960
            self.height = 540
        elif self.resolution == '720p':
            self.width = 1280
            self.height = 720

    def stop(self):
        self.camera.stop_video_stream()
        self.streaming = False
        cv2.destroyAllWindows()

    def start(self):
        self.camera.start_video_stream(display=False, resolution=self.resolution)
        self.streaming = True
    
    def view(self):
        if not self.streaming:
            self.start()
        img = self.camera.read_cv2_image(strategy='newest') 
        while self.__debugList:
            item = self.__debugList.pop()
            if item['type'] == "box":
                cv2.rectangle(img, item['start'], item['end'], item['color'], 2)
            if item['type'] == "point":
                cv2.circle(img, item['point'], 2, item['color'], -1)
        cv2.imshow("Robot", img)
        cv2.waitKey(1)


    # AI Vision

    def setDetectMode(self, mode):
        if not mode.lower() in ['person','gesture','line','marker','robot']:
            print("Invalid detect mode")
            print("Please choose from the following options:")
            print("Person, Gesture, Line, Marker, Robot")
        self.detectMode = mode.lower()

    def setVisionDebug(self, value = True):
        self.visionDebug = value

    def setDebugColor(self, color = (255,0,0)):
        self.debugColor = color

    def __detectCallback(self, info):
        if not info:
            return False
        if self.detectMode == "person":
            peopleList = []
            for person in info:
                if self.debugMode:
                    print(f"person detected at x: {person[0]} y: {person[1]} w: {person[2]} h: {person[3]}")
                x = int(person[0] * self.width)
                y = int(person[1] * self.height)
                w = int(person[2] * self.width)
                h = int(person[3] * self.height)
                peopleList += [{'type':'box',
                        'start':(x-w//2,y-h//2),
                        'end':(x+w//2,y+h//2),
                        'color':self.debugColor}]
            if self.visionDebug:
                self.__debugList = peopleList
        if self.detectMode == "gesture":
            if self.debugMode:
                gestureList = []
                for gesture in info:
                    if self.debugMode:
                        print(f"gesture detected at x: {gesture[0]} y: {gesture[1]} w: {gesture[2]} h: {gesture[3]}")
                    x = int(gesture[0] * self.width)
                    y = int(gesture[1] * self.height)
                    w = int(gesture[2] * self.width)
                    h = int(gesture[3] * self.height)
                    gestureList += [{'type':'box',
                            'start':(x-w//2,y-h//2),
                            'end':(x+w//2,y+h//2),
                            'color':self.debugColor}]
            if self.visionDebug:
                self.__debugList = gestureList
        if self.detectMode == "line":
            linetype = "none"
            if info[0] == 1:
                linetype = "straight"
            if info[0] == 2:
                linetype = "forked"
            if info[0] == 3:
                linetype = "crossing"
            if self.debugMode:                    
                print(f"\n{linetype} line detected")
            pointList = []
            for i, point in enumerate(info[1:]):
                if self.debugMode:
                    print(f"Point {i+1:>2} - x: {point[0]:.2f} y: {point[1]:.2f} t: {point[2]:>6.2f} c: {point[3]:>5.2f}")
                x = int(point[0] * self.width)
                y = int(point[1] * self.height)
                t = int(point[2] * self.width)
                c = int(point[3] * self.height)
                pointList += [{'type':'point',
                        'point':(x,y),
                        'color':self.debugColor}]
            if self.visionDebug:
                self.__debugList = pointList              
        if self.detectMode == "marker":
            markerList = []
            for marker in info:
                markerType = "none"
                if marker[4] == 0:
                    markerType = "heart"
                if marker[4] == 0:
                    markerType = "A"
                if marker[4] == 0:
                    markerType = "B"
                if marker[4] == 0:
                    markerType = "C"
                if marker[4] == 10:
                    markerType = "zero"
                if marker[4] == 11:
                    markerType = "one"
                if marker[4] == 12:
                    markerType = "two"
                if marker[4] == 13:
                    markerType = "three"
                if marker[4] == 14:
                    markerType = "four"
                if marker[4] == 15:
                    markerType = "five"
                if marker[4] == 16:
                    markerType = "six"
                if marker[4] == 17:
                    markerType = "seven"
                if marker[4] == 18:
                    markerType = "eight"
                if marker[4] == 19:
                    markerType = "nine"
                if self.debugMode:
                    print(f"marker {markerType} detected at x: {marker[0]} y: {marker[1]} w: {marker[2]} h: {marker[3]}")
                x = int(marker[0] * self.width)
                y = int(marker[1] * self.height)
                w = int(marker[2] * self.width)
                h = int(marker[3] * self.height)
                markerList += [{'type':'box',
                        'start':(x-w//2,y-h//2),
                        'end':(x+w//2,y+h//2),
                        'color':self.debugColor}]
            if self.visionDebug:
                self.__debugList = markerList
        if self.detectMode == "robot":
            robotList = []
            for robot in info:
                if self.debugMode:
                    print(f"robot detected at x: {robot[0]} y: {robot[1]} w: {robot[2]} h: {robot[3]}")
                x = int(robot[0] * self.width)
                y = int(robot[1] * self.height)
                w = int(robot[2] * self.width)
                h = int(robot[3] * self.height)
                robotList += [{'type':'box',
                        'start':(x-w//2,y-h//2),
                        'end':(x+w//2,y+h//2),
                        'color':self.debugColor}]
            if self.visionDebug:
                self.__debugList = robotList
        

    def detect(self, name=None, color=None):
        if name is None:
            name = self.detectMode
        if not self.streaming:
            self.start()
        self.detectMode = name
        self.vision.sub_detect_info(name,color,self.__detectCallback)

    def detectPerson(self):
        self.detect("person")

    def detectGesture(self):
        self.detect("gesture")

    def detectLine(self, color="red"):
        self.detect("line",color)

    def detectMarker(self, color="red"):
        self.detect("marker", color)

    def detectRobot(self):
        self.detect("robot")