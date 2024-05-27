import cv2
from simple_pid import PID


class Camera:
    def __init__(self, RoboMaster) -> None:
        self.robomaster = RoboMaster
        self.robot = self.robomaster.robot
        self.camera = self.robot.camera
        self.vision = self.robot.vision
        self.streaming = False
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

    def stop(self) -> None:
        """
        Stop the video stream
        """
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
            self.start()
        img = self.camera.read_cv2_image(strategy="newest")
        while self.__debugList:
            item = self.__debugList.pop()
            if item["type"] == "box":
                cv2.rectangle(img, item["start"], item["end"], item["color"], 2)
            if item["type"] == "point":
                cv2.circle(img, item["point"], 2, item["color"], -1)
        cv2.imshow("Robot", img)
        cv2.waitKey(1)

    # AI Vision

    def setPID(self, P=330, I=0, D=28):
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
        self.vision.sub_detect_info(name, color, self.__detectCallback)

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

    def __followCallback(self, info):
        """
        detect and follow things in the video stream on device
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
        self.vision.sub_detect_info(name, color, self.__followCallback)

    def followLine(self, color="red"):
        """
        Detect and follow a line (red, green or blue)
        Args:
        color(str, optional): "red", "green" or "blue", Default to "red"
        """
        self.follow("line", color)
