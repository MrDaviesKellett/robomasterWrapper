import cv2
from simple_pid import PID


class Camera:
    def __init__(self, robomaster) -> None:
        self.robomaster = robomaster
        self.robot = self.robomaster.robot
        self.camera = self.robot.camera
        self.vision = self.robot.vision
        self.streaming = False
        self.detecting = False
        self.detectMode = "line"
        self.visionDebug = False
        self.debugColor = (0, 255, 0)
        self.resolution = "360p"
        self.width = 640
        self.height = 360
        self.debugMode = False
        self.__debugList = []
        self.frequency = 30
        self.followSpeed = 0.5
        self.followDistance = 0.5
        self.atMarker = False
        self.pid = PID(-330, -0, -28, setpoint=0.0, sample_time=1.0 / self.frequency)
        self.pid.output_limits = (
            -self.followSpeed / 3.5 * 600,
            self.followSpeed / 3.5 * 600,
        )

    # Camera Display
    def setResolution(self, resolution: str = "360P") -> None:
        """
        Set the resolution of the camera stream
        Args:
        resolution(str, optional): The resolution to set. Defaults to "360P".
        """
        if not resolution in ["360p", "540p", "720p"]:
            print("Invalid resolution")
            print("Please choose from the following options:")
            print("360p, 520p, 720p")
        self.resolution = resolution.lower()
        if self.resolution == "360p":
            self.width = 640
            self.height = 360
        elif self.resolution == "520p":
            self.width = 960
            self.height = 540
        elif self.resolution == "720p":
            self.width = 1280
            self.height = 720

    def stopDetect(self) -> None:
        """
        Stop the video stream
        """
        self.vision.unsub_detect_info(self.detectMode)
        self.detecting = False

    def stop(self) -> None:
        """
        Stop the video stream
        """
        self.stopDetect()
        self.camera.stop_video_stream()
        self.streaming = False
        cv2.destroyAllWindows()

    def start(self) -> None:
        """
        Start the video stream
        """
        self.camera.start_video_stream(display=False, resolution=self.resolution)
        self.streaming = True

    def view(self) -> None:
        """
        View the video stream
        """
        if not self.streaming:
            print("Camera is not streaming, please use the cam.start() command first")
        try:
            img = self.camera.read_cv2_image(strategy="newest")
            while self.__debugList:
                item = self.__debugList.pop()
                if item["type"] == "box":
                    cv2.rectangle(img, item["start"], item["end"], item["color"], 2)
                if item["type"] == "point":
                    cv2.circle(img, item["point"], 2, item["color"], -1)
            cv2.imshow("Robot", img)
            cv2.waitKey(1)
        except:
            return False

    # AI Vision

    def setPID(self, P=330, I=0, D=28):
        """
        set the PID values
        Args:
        P(Int: optional): P = Proportional
        I(Int: optional): I = Integral
        D(Int: optional): D = Derivative
        """
        self.pid.Kp = -P
        self.pid.Ki = -I
        self.pid.Kd = -D

    def setDetectMode(self, mode: str = "line") -> None:
        """
        Set the detection mode of the AI image recognition
        Args:
        mode(str, optional) choose from "person", "gesture", "line", "marker", "robot" Default to "line".
        """
        if not mode.lower() in ["person", "gesture", "line", "marker", "robot"]:
            print("Invalid detect mode")
            print("Please choose from the following options:")
            print("Person, Gesture, Line, Marker, Robot")
        self.detectMode = mode.lower()

    def setVisionDebug(self, value=True) -> None:
        """
        Set vision debugging on
        """
        self.visionDebug = value

    def setDebugColor(self, color=(255, 0, 0)) -> None:
        """
        Set the vision debugging color
        """
        self.debugColor = color

    def __detectCallback(self, info) -> bool:
        """
        detect things in the video stream on device
        Args:
        info(dict): The information of the detected object
        """
        if not info:
            return False
        if self.detectMode == "person":
            peopleList = []
            for person in info:
                if self.debugMode:
                    print(
                        f"person detected at x: {person[0]} y: {person[1]} w: {person[2]} h: {person[3]}"
                    )
                x = int(person[0] * self.width)
                y = int(person[1] * self.height)
                w = int(person[2] * self.width)
                h = int(person[3] * self.height)
                peopleList += [
                    {
                        "type": "box",
                        "start": (x - w // 2, y - h // 2),
                        "end": (x + w // 2, y + h // 2),
                        "color": self.debugColor,
                    }
                ]
            if self.visionDebug:
                self.__debugList = peopleList
        if self.detectMode == "gesture":
            if self.debugMode:
                gestureList = []
                for gesture in info:
                    if self.debugMode:
                        print(
                            f"gesture detected at x: {gesture[0]} y: {gesture[1]} w: {gesture[2]} h: {gesture[3]}"
                        )
                    x = int(gesture[0] * self.width)
                    y = int(gesture[1] * self.height)
                    w = int(gesture[2] * self.width)
                    h = int(gesture[3] * self.height)
                    gestureList += [
                        {
                            "type": "box",
                            "start": (x - w // 2, y - h // 2),
                            "end": (x + w // 2, y + h // 2),
                            "color": self.debugColor,
                        }
                    ]
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
                if self.debugMode == "verbose":
                    print(
                        f"Point {i+1:>2} - x: {point[0]:.2f} y: {point[1]:.2f} t: {point[2]:>6.2f} c: {point[3]:>5.2f}"
                    )
                x = int(point[0] * self.width)
                y = int(point[1] * self.height)
                t = int(point[2] * self.width)
                c = int(point[3] * self.height)
                pointList += [
                    {"type": "point", "point": (x, y), "color": self.debugColor}
                ]
            if self.visionDebug:
                self.__debugList = pointList
        if self.detectMode == "marker":
            markerList = []
            for marker in info:
                if self.debugMode:
                    print(
                        f"marker {marker[4]} detected at x: {marker[0]} y: {marker[1]} w: {marker[2]} h: {marker[3]}"
                    )
                x = int(marker[0] * self.width)
                y = int(marker[1] * self.height)
                w = int(marker[2] * self.width)
                h = int(marker[3] * self.height)
                markerList += [
                    {
                        "type": "box",
                        "start": (x - w // 2, y - h // 2),
                        "end": (x + w // 2, y + h // 2),
                        "color": self.debugColor,
                    }
                ]
            if self.visionDebug:
                self.__debugList = markerList
        if self.detectMode == "robot":
            robotList = []
            for robot in info:
                if self.debugMode:
                    print(
                        f"robot detected at x: {robot[0]} y: {robot[1]} w: {robot[2]} h: {robot[3]}"
                    )
                x = int(robot[0] * self.width)
                y = int(robot[1] * self.height)
                w = int(robot[2] * self.width)
                h = int(robot[3] * self.height)
                robotList += [
                    {
                        "type": "box",
                        "start": (x - w // 2, y - h // 2),
                        "end": (x + w // 2, y + h // 2),
                        "color": self.debugColor,
                    }
                ]
            if self.visionDebug:
                self.__debugList = robotList

    def detect(self, name=None, color=None) -> None:
        """
        Detect an object of type name and of a particular color
        Args:
        name(str or None, optional) the type of detection from "person", "gesture", "line", "marker", "robot" Default to "line".
        color(str or None, optional) the color of detected objects, can be "red", "green", "blue"
        """
        if name is None:
            name = self.detectMode
        if not self.streaming:
            self.start()
        self.detectMode = name
        if not self.detecting:
            self.vision.sub_detect_info(name, color, self.__detectCallback)
            self.detecting = True

    def detectPerson(self) -> None:
        """
        Detect a person
        """
        self.detect("person")

    def detectGesture(self) -> None:
        """
        Detect Gestures
        """
        self.detect("gesture")

    def detectLine(self, color="red") -> None:
        """
        Detect a line (red, green or blue)
        Args:
        color(str, optional): "red", "green" or "blue", Default to "red"
        """
        self.detect("line", color)

    def detectMarker(self, color="red") -> None:
        """
        Detect a marker (red, green or blue)
        Args:
        color(str, optional): "red", "green" or "blue", Default to "red"
        """
        self.detect("marker", color)

    def detectRobot(self) -> None:
        """
        Detect another robomaster robot
        """
        self.detect("robot")

    def setFollowSpeed(self, speed):
        """
        Set the follwowing speed
        """
        self.followSpeed = speed

    def setFollowDistance(self, distance):
        """
        Set the following distance
        """
        self.followDistance = distance

    def __lookatCallback(self, info):
        """
        Detect and look at things in the video stream on device
        Args:
        info(dict): The information of the detected object
        """
        self.__detectCallback(info)

    def __followCallback(self, info):
        """
        Detect and follow things in the video stream on device
        Args:
        info(dict): The information of the detected object
        """
        self.__detectCallback(info)
        if self.detectMode == "line":
            if info == [0]:
                self.robomaster.setSpeed(0, 0, 0)
                return False
            followPoint = 5
            angle = info[followPoint + 1][2]
            val = self.pid(angle)
            if self.debugMode:
                print(
                    f"following point {followPoint} tangent angle is {angle}, pid is {val}"
                )
            self.robomaster.setSpeed(x=self.followSpeed, z=val)
        if self.detectMode == "marker":
            if info == [0]:
                self.robomaster.setSpeed(0, 0, 0)
                return False
            # TODO: follow markers maintaining a specific distance

    def follow(self, name=None, color="red"):
        """
        Detect and follow an object of type name and of a particular color
        Args:
        name(str or None, optional) the type of detection from "person", "gesture", "line", "marker", "robot" Default to "line".
        color(str or None, optional) the color of detected objects, can be "red", "green", "blue"
        """
        if name is None:
            name = self.detectMode
        if not self.streaming:
            self.start()
        self.detectMode = name
        if not self.detecting:
            self.vision.sub_detect_info(name, color, self.__followCallback)
            self.detecting = True

    def followLine(self, color="red"):
        """
        Detect and follow a line (red, green or blue)
        Args:
        color(str, optional): "red", "green" or "blue", Default to "red"
        """
        self.follow("line", color)

    def moveToMarker(self, markerType="1", color="red", error=0.06, speed=1, minSpeed = 0.02, targetX = 0, targetY = 0.5):
        """
        Detect and move in front of a marker (red, green or blue)
        Args:
        markerType(str, optional): the type of marker to follow, Default to "0"
        color(str, optional): "red", "green" or "blue", Default to "red"
        error(float, optional): the error margin for the detection, Default to 0.03
        speed(float, optional): the speed of the robot in m/s, Default to 0.5 m/s
        minSpeed(float, optional): the minimum speed in m/s the robot should move, Default 0.02 m/s
        targetX(float, optional): the target position of the marker on the camera view in x -1 is the left of the view 1 is the right of the view
        targetY(float, optional): the garget position of the marker on the camera view in y -1 is the top of the view 1 is the bottom of the view
        """

        if type(markerType) is not str:
            try:
                markerType = str(markerType)
            except ValueError:
                raise TypeError("markerType must be a string")
            
        self.atMarker = False

        def _moveToMarkerCallback(info):
            """
            Detect and move to a marker (red, green or blue)
            Args:
            info(dict): The information of the detected object
            """
            if self.atMarker:
                self.robomaster.stop()
                return
            
            if not info == []:
                for marker in info:
                    x = 0
                    y = 0
                    if marker[4] == markerType:

                        x = marker[0]  # range of 0 to 1
                        y = marker[1]  # range of 0 to 1

                        # map x into -1 to 1 (for left and right movement of the robot)
                        x = x * 2 - 1
                        y = y * 2 - 1

                        if self.debugMode == "verbose":
                            print(
                                f"marker {marker[4]} detected at x: {x:.3f} y: {y:.3f} w: {marker[2]:.3f} h: {marker[3]:.3f}"
                            )
                        
                        if x < targetX - error or x > targetX + error:
                            xMove = (x - targetX) * speed
                            if abs(xMove) < minSpeed:
                                if x > 1:
                                    xMove = minSpeed
                                else:
                                    xMove = -minSpeed
                            self.robomaster.setSpeed(0, xMove, 0)  # move left/right
                        elif y < targetY - error or y > targetY + error:
                            yMove = (y - targetY) * speed
                            if abs(yMove) < minSpeed:
                                if y > 1:
                                    yMove = minSpeed
                                else:
                                    yMove = -minSpeed
                            self.robomaster.setSpeed(-yMove, 0, 0)  # just move forward..
                        if targetY - error < y < targetY + error and targetX - error < x < targetX + error:
                            if self.debugMode == "verbose":
                                print("arrived at Marker")
                            self.robomaster.stop()
                            self.stopDetect()
                            self.atMarker = True
                            return True
            else:
                if self.debugMode == "verbose":
                    print("no marker found!")
                #self.robomaster.stop()           
            return False

        if not self.streaming:
            self.start()
        self.detectMode = "marker"
        if not self.detecting:
            self.detecting = True
            self.vision.sub_detect_info(
                self.detectMode, color, _moveToMarkerCallback
            )
