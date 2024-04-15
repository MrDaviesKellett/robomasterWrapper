from robomaster import led
from robomaster.action import Action


class Gun:
    def __init__(self, RoboMaster) -> None:
        self.robomaster = RoboMaster
        self.robot = self.robomaster.robot

    # LEDs

    def setLEDs(
        self,
        r: int = 0,
        g: int = 0,
        b: int = 0,
        leds: str = "gun",
        effect: str = "on",
    ):
        """
        Set the robot LEDs to a specific colour.
        Args:
        r(int): Red value.
        g(int): Green value.
        b(int): Blue value.
        leds (str): LED component. front, back, left, right, gun, gunLeft, gunRight, all. Defaults to "all".
        effect (str): LED effect. on, off, pulse, flash, breath, scrolling. Defaults to "on".
        """

        if leds == "gun":
            comp = led.COMP_TOP_ALL
        elif leds == "gunLeft":
            comp = led.COMP_TOP_LEFT
        elif leds == "gunRight":
            comp = led.COMP_TOP_RIGHT
        else:
            comp = led.COMP_TOP_ALL

        if effect == "on":
            effect = led.EFFECT_ON
        elif effect == "off":
            effect = led.EFFECT_OFF
        elif effect == "pulse":
            effect = led.EFFECT_PULSE
        elif effect == "flash":
            effect = led.EFFECT_FLASH
        elif effect == "breath":
            effect = led.EFFECT_BREATH
        elif effect == "scrolling":
            effect = led.EFFECT_SCROLLING
        else:
            effect = led.EFFECT_ON

        self.led.set_led(comp=comp, r=r, g=g, b=b, effect=effect)

    # gun

    def rotate(self, pitchSpeed: float = 0.0, yawSpeed: float = 0.0) -> None:
        """
        Rotate the gun at a set pitch and yaw speed.
        Minimum Pitch speed is -360°/s, maximum pitch speed is +360°/s.
        Minimum Yaw speed is -360°/s, maximum yaw speed is +360°/s.
        Args:
        pitchSpeed (float): Speed of the gun pitch in degrees per second. Defaults to 0.0.
        yawSpeed (float): Speed of the gun yaw in degrees per second. Defaults to 0.0.
        blocking (bool): Block until action is complete. Defaults to False.
        """
        self.robot.gimbal.drive_speed(pitch_speed=pitchSpeed, yaw_speed=yawSpeed)

    def move(
        self,
        pitch: float = 0.0,
        yaw: float = 0.0,
        pitchSpeed: float = 30.0,
        yawSpeed: float = 30.0,
        blocking: bool = True,
    ) -> Action:
        """
        Move the gun to a set pitch and yaw position.
        The Origin (starting point) is at the current position of the gun.
        Minimum Pitch is -55° and Maximum Pitch is +55°.
        Minimum Yaw is -55° and Maximum Yaw is +55°.
        Minimum Pitch speed is 0°/s, maximum pitch speed is 540°/s.
        Minimum Yaw speed is 0°/s, maximum yaw speed is 540°/s.
        Args:
        pitch (float): Pitch position of the gun in degrees. Defaults to 0.0.
        yaw (float): Yaw position of the gun in degrees. Defaults to 0.0.
        pitchSpeed (float): Speed of the gun pitch in degrees per second. Defaults to 30.0.
        yawSpeed (float): Speed of the gun yaw in degrees per second. Defaults to 30.0.
        blocking (bool): Block until action is complete. Defaults to False.
        """
        if not blocking:
            return self.robot.gimbal.move(
                pitch=pitch, yaw=yaw, pitch_speed=pitchSpeed, yaw_speed=yawSpeed
            )
        else:
            return self.robot.gimbal.move(
                pitch=pitch, yaw=yaw, pitch_speed=pitchSpeed, yaw_speed=yawSpeed
            ).wait_for_completed()

    def moveGunto(
        self,
        pitch: float = 0.0,
        yaw: float = 0.0,
        pitchSpeed: float = 30.0,
        yawSpeed: float = 30.0,
        blocking: bool = True,
    ) -> Action:
        """
        Move the gun to a set pitch and yaw position.
        The Origin (starting point) is the coordinate at initialisation (start up).
        Minimum Pitch is -25° and Maximum Pitch is +30°.
        Minimum Yaw is -250° and Maximum Yaw is +250°.
        Minimum Pitch speed is 0°/s, maximum pitch speed is 540°/s.
        Minimum Yaw speed is 0°/s, maximum yaw speed is 540°/s.
        Args:
        pitch (float): Pitch position of the gun in degrees. Defaults to 0.0.
        yaw (float): Yaw position of the gun in degrees. Defaults to 0.0.
        pitchSpeed (float): Speed of the gun pitch in degrees per second. Defaults to 30.0.
        yawSpeed (float): Speed of the gun yaw in degrees per second. Defaults to 30.0.
        blocking (bool): Block until action is complete. Defaults to False.
        """
        if not blocking:
            return self.robot.gimbal.moveto(
                pitch=pitch, yaw=yaw, pitch_speed=pitchSpeed, yaw_speed=yawSpeed
            )
        else:
            return self.robot.gimbal.moveto(
                pitch=pitch, yaw=yaw, pitch_speed=pitchSpeed, yaw_speed=yawSpeed
            ).wait_for_completed()

    def recenter(
        self, pitchSpeed: float = 30.0, yawSpeed: float = 30.0, blocking: bool = True
    ) -> Action:
        # TODO: test that min and max values are correct
        """
        Recenters the gun to its starting position.
        Minimum Pitch speed is 0°/s, maximum pitch speed is 540°/s.
        Minimum Yaw speed is 0°/s, maximum yaw speed is 540°/s.
        """
        if not blocking:
            return self.robot.gimbal.recenter(
                pitch_speed=pitchSpeed, yaw_speed=yawSpeed
            )
        else:
            return self.robot.gimbal.recenter(
                pitch_speed=pitchSpeed, yaw_speed=yawSpeed
            ).wait_for_completed()

    def resume(self) -> bool:
        """
        Resumes the gun after it has been paused.
        """
        return self.robot.gimbal.resume()

    def suspend(self) -> bool:
        """
        Puts the gun into a paused state, where it will be loose and unpowered until resumed.
        """
        return self.robot.gimbal.suspend()

    # Blaster

    def fireBlaster(self, fireType: str = "ir", times: int = 1) -> bool:
        """
        Fires the blaster.
        Fire type can be either "ir" or "water". Defaults to "ir".
        "water" refers to water based pellets.
        args:
        fireType (str): Type of blaster fire. Defaults to "ir".
        times (int): Number of times the blaster should be fired. Defaults to 1.
        """
        return self.robot.blaster.fire(fire_type=fireType, times=times)

    def setBlasterLED(self, brightness: int = 100, effect: str = "on") -> bool:
        # TODO: test effect
        """
        Sets the blaster LED brightness and effect.
        Minimum brightness is 0, maximum brightness is 255.
        Effect can be either "on" or "off". Defaults to "on".
        args:
        brightness (int): Brightness of the blaster LED. Defaults to 100.
        effect (str): Effect of the blaster LED. Defaults to "on".
        """
        return self.robot.blaster.set_led(brightness=brightness, effect=effect)
