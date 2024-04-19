from robomaster import camera
import cv2

class Camera:
    def __init__(self, RoboMaster) -> None:
        self.robomaster = RoboMaster
        self.robot = self.robomaster.robot
        self.camera = self.robot.camera
        self.displayOn = False

    # Camera Display

    def displayThread(self):
        self.camera.start_video_stream(display=False)
        self.displayOn = True
        while self.displayOn:
            img = self.camera.read_cv2_image()
            cv2.imshow("Robomaster camera", img)
            cv2.waitKey(1)
        cv2.destroyAllWindows()
        self.displayOn = False
        self.camera.stop_video_stream()

    def display(self):
        pass

    # AI Vision