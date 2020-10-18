import sys
import logging
import time
from ev3dev2.sensor import Sensor
from ev3dev2 import *

if sys.version_info < (3, 4):
    raise SystemError('Must be using Python 3.4 or higher')

#SensorTouch
class TouchSensor(Sensor):
    """
    Touch Sensor
    """

    SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    #: Button state
    MODE_TOUCH = 'TOUCH'
    MODES = (MODE_TOUCH, )

    def __init__(self, **kwargs):
        self.kwargs = kwargs

        if "clientID" in self.kwargs:  # local preference
            self.clientID = self.kwargs['clientID']
        elif "clientID" in globals():  # global question
            if VERBOSE:
                print("global ID")
            self.clientID = clientID
            self.kwargs['clientID'] = clientID
        else:
            self.kwargs['clientID'] = connection(dummy=not True)

        self.connected = True
        self.mode = TouchSensor.MODE_TOUCH

    @property
    def is_pressed(self):
        """
        A boolean indicating whether the current touch sensor is being
        pressed.
        """
        self._ensure_mode(self.MODE_TOUCH)

        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'SensorTouch',
            [], [], [], bytearray(),
            vrep.simx_opmode_oneshot_wait
        )
        returnCode, outInts, outFloats, outStrings, outBuffer = out

        pressed = outInts[0]
        print(pressed)
        return bool(pressed)

    @property
    def is_released(self):
        return not self.is_pressed

    def _wait(self, wait_for_press, timeout_ms, sleep_ms):
        tic = time.time()

        if sleep_ms:
            sleep_ms = float(sleep_ms / 1000)

        # The kernel does not supoort POLLPRI or POLLIN for sensors so we have
        # to drop into a loop and check often
        while True:

            if self.is_pressed == wait_for_press:
                return True

            if timeout_ms is not None and time.time() >= tic + timeout_ms / 1000:
                return False

            if sleep_ms:
                time.sleep(sleep_ms)

    def wait_for_pressed(self, timeout_ms=None, sleep_ms=10):
        """
        Wait for the touch sensor to be pressed down.
        """
        return self._wait(True, timeout_ms, sleep_ms)

    def wait_for_released(self, timeout_ms=None, sleep_ms=10):
        """
        Wait for the touch sensor to be released.
        """
        return self._wait(False, timeout_ms, sleep_ms)

    def wait_for_bump(self, timeout_ms=None, sleep_ms=10):
        """
        Wait for the touch sensor to be pressed down and then released.
        Both actions must happen within timeout_ms.
        """
        start_time = time.time()

        if self.wait_for_pressed(timeout_ms, sleep_ms):
            if timeout_ms is not None:
                timeout_ms -= int((time.time() - start_time) * 1000)
            return self.wait_for_released(timeout_ms, sleep_ms)

        return False


class GyroSensor(Sensor):
    """
    LEGO EV3 gyro sensor.
    """

    VREP_GYRO_NAME = 'Giroscopio'
    # VREP_GYRO_NAME = 'GyroSensor_G'
    # VREP_GYRO_NAME = 'GyroSensor_reference_G'
    # VREP_GYRO_NAME = 'GyroSensor_VA'
    # VREP_GYRO_NAME = 'GyroSensor_reference'

    # vrep
    def __init__(self, **kwargs):
        self.kwargs = kwargs

        if "clientID" in self.kwargs:  # local preference
            self.clientID = self.kwargs['clientID']
        elif "clientID" in globals():  # global question
            if VERBOSE:
                print("global ID")
            self.clientID = clientID
            self.kwargs['clientID'] = clientID
        else:
            self.kwargs['clientID'] = connection(dummy=not True)
            # print("Need clientID")
            # sys.exit("No client")

        if "gyroName" in self.kwargs:
            self.VREP_GYRO_NAME = self.kwargs.get('gyroName')
        else:
            self.kwargs['gyroName'] = self.VREP_GYRO_NAME

        # object handle
        errorCode, self.gyro = vrep.simxGetObjectHandle(
            self.kwargs.get('clientID'),
            # GyroSensor.VREP_GYRO_NAME,
            self.VREP_GYRO_NAME,
            vrep.simx_opmode_oneshot_wait)

        if VERBOSE:
            print("GyroError, GyroCodeNr", errorCode, self.gyro)

        self.connected = True
        self.units = 'degrees'
        self.mode = 'GYRO-ANG'  # or 'GYRO-RATE'

    @property
    def mode(self):
        return self.__mode

    @mode.setter
    def mode(self, var):
        if var in ['GYRO_ANG', 'GYRO_RATE']:
            self.__mode = var
        else:
            self.__mode = 'GYRO_ANG'

    def __str__(self):
        return f"{self.gyro} in {self.mode} mode"

    def __repr__(self):
        return self.__str__()

    def angle(self):
        emptyBuff = bytearray()
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'SensorGyroA',
            [], [], [], emptyBuff,
            vrep.simx_opmode_oneshot_wait
        )

        returnCode, outInts, outFloats, outStrings, outBuffer = out
        _angle = outInts[0]
        return _angle

    def angular_velocity(self):
        emptyBuff = bytearray()
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'SensorGyroVA',
            [], [], [], emptyBuff,
            vrep.simx_opmode_oneshot_wait
        )

        returnCode, outInts, outFloats, outStrings, outBuffer = out
        _angular_velocity = outInts[0]
        return _angular_velocity

    def value(self):
        if self.mode == 'GYRO-RATE':
            return self.angular_velocity()
        else:  # 'GYRO-ANG':
            return self.angle()

    def velocity(self):
        return self.angular_velocity()

    def state(self):
        return self.angle(), self.angular_velocity()


class ColorSensor(Sensor):
    """
    LEGO EV3 color sensor.
    """

    # __slots__ = ['red_max', 'green_max', 'blue_max']

    SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    #: Reflected light. Red LED on.
    MODE_COL_REFLECT = 'COL-REFLECT'

    #: Ambient light. Blue LEDs on.
    MODE_COL_AMBIENT = 'COL-AMBIENT'

    #: Color. All LEDs rapidly cycling, appears white.
    MODE_COL_COLOR = 'COL-COLOR'

    #: Raw reflected. Red LED on
    MODE_REF_RAW = 'REF-RAW'

    #: Raw Color Components. All LEDs rapidly cycling, appears white.
    MODE_RGB_RAW = 'RGB-RAW'

    #: No color.
    COLOR_NOCOLOR = 0

    #: Black color.
    COLOR_BLACK = 1

    #: Blue color.
    COLOR_BLUE = 2

    #: Green color.
    COLOR_GREEN = 3

    #: Yellow color.
    COLOR_YELLOW = 4

    #: Red color.
    COLOR_RED = 5

    #: White color.
    COLOR_WHITE = 6

    #: Brown color.
    COLOR_BROWN = 7

    MODES = (MODE_COL_REFLECT, MODE_COL_AMBIENT, MODE_COL_COLOR, MODE_REF_RAW, MODE_RGB_RAW)

    COLORS = (
        'NoColor',
        'Black',
        'Blue',
        'Green',
        'Yellow',
        'Red',
        'White',
        'Brown',
    )

    VREP_COLOR_SENSOR_NAME = SYSTEM_CLASS_NAME
    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        # super(ColorSensor, self).__init__(address, name_pattern, name_exact, driver_name='lego-ev3-color', **kwargs)
        self.kwargs = kwargs

        if "clientID" in self.kwargs:  # local preference
            self.clientID = self.kwargs['clientID']
        elif "clientID" in globals():  # global question
            if VERBOSE:
                print("global ID")
            self.clientID = clientID
            self.kwargs['clientID'] = clientID
        else:
            self.kwargs['clientID'] = connection(dummy=not True)
            # print("Need clientID")
            # sys.exit("No client")

        # if "Color" in self.kwargs:
        #     self.VREP_COLOR_SENSOR_NAME = self.kwargs.get('ColorSensor')
        # else:
        #     self.kwargs['ColorSensor'] = self.SYSTEM_CLASS_NAME

        # object handle
        errorCode, self.color_sensor = vrep.simxGetObjectHandle(
            self.kwargs.get('clientID'),
            # GyroSensor.VREP_GYRO_NAME,
            self.VREP_COLOR_SENSOR_NAME,
            vrep.simx_opmode_oneshot_wait)

        if VERBOSE:
            print("ColorSensorError, ColorSensorCodeNr", errorCode, self.color_sensor)

        self.connected = True
        self.mode = 'COL-REFLECT'

        # LM
        self.color_depth = None
        self.intensity = None

        # See calibrate_white() for more details
        self.red_max = 300
        self.green_max = 300
        self.blue_max = 300

    def read_intensity(self):
        emptyBuff = bytearray()
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'SensorLight',
            [], [], [], emptyBuff,
            vrep.simx_opmode_oneshot_wait
        )
        returnCode, outInts, outFloats, outStrings, outBuffer = out
        return outInts[0]

    @property
    def intensity(self):
        'returns intensity'
        return self.read_intensity()

    @intensity.setter
    def intensity(self, val):
        self._intensity = self.read_intensity()

    def read_color(self):
        emptyBuff = bytearray()
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'SensorColor',
            [], [], [], emptyBuff,
            vrep.simx_opmode_oneshot_wait
        )
        returnCode, outInts, outFloats, outStrings, outBuffer = out
        return outFloats

    @property
    def color_depth(self):
        'returns red, green, blue, depth values'
        return self.read_color()

    @color_depth.setter
    def color_depth(self, val):
        self._color_depth = self.read_color()

    def value(self, idx):
        return self.color_depth[idx]

    @property
    def reflected_light_intensity(self):
        """
        Reflected light intensity as a percentage (0 to 100). Light on sensor is red.
        """
        self._ensure_mode(self.MODE_COL_REFLECT)
        return self.intensity  #self.get_light  # value(0)

    @property
    def ambient_light_intensity(self):
        """
        Ambient light intensity, as a percentage (0 to 100). Light on sensor is dimly lit blue.
        """
        self._ensure_mode(self.MODE_COL_AMBIENT)
        return self.intensity  # self.get_light  # value(0)

    @property
    def color(self):
        """
        Color detected by the sensor, categorized by overall value.
          - 0: No color
          - 1: Black
          - 2: Blue
          - 3: Green
          - 4: Yellow
          - 5: Red
          - 6: White
          - 7: Brown
        """
        self._ensure_mode(self.MODE_COL_COLOR)
        return self.value(0)

    @property
    def color_name(self):
        """
        Returns NoColor, Black, Blue, etc
        """
        return self.COLORS[self.color]

    @property
    def raw(self):
        """
        Red, green, and blue components of the detected color, as a tuple.
        Officially in the range 0-1020 but the values returned will never be
        that high. We do not yet know why the values returned are low, but
        pointing the color sensor at a well lit sheet of white paper will return
        values in the 250-400 range.
        If this is an issue, check out the rgb() and calibrate_white() methods.
        """
        self._ensure_mode(self.MODE_RGB_RAW)

        r, g, b, d = self.color_depth
        r *= self.red_max
        g *= self.green_max
        b *= self.blue_max
        return r, g, b

    def calibrate_white(self):
        """
        The RGB raw values are on a scale of 0-1020 but you never see a value
        anywhere close to 1020.  This function is designed to be called when
        the sensor is placed over a white object in order to figure out what
        are the maximum RGB values the robot can expect to see.  We will use
        these maximum values to scale future raw values to a 0-255 range in
        rgb().
        If you never call this function red_max, green_max, and blue_max will
        use a default value of 300.  This default was selected by measuring
        the RGB values of a white sheet of paper in a well lit room.
        Note that there are several variables that influence the maximum RGB
        values detected by the color sensor
        - the distance of the color sensor to the white object
        - the amount of light in the room
        - shadows that the robot casts on the sensor
        """
        (self.red_max, self.green_max, self.blue_max) = self.raw

    @property
    def rgb(self):
        """
        Same as raw() but RGB values are scaled to 0-255
        """
        (red, green, blue) = self.raw

        return (min(int((red * 255) / self.red_max), 255), min(int((green * 255) / self.green_max),
                                                               255), min(int((blue * 255) / self.blue_max), 255))

    @property
    def lab(self):
        """
        Return colors in Lab color space
        """
        RGB = [0, 0, 0]
        XYZ = [0, 0, 0]

        for (num, value) in enumerate(self.rgb):
            if value > 0.04045:
                value = pow(((value + 0.055) / 1.055), 2.4)
            else:
                value = value / 12.92

            RGB[num] = value * 100.0

        # http://www.brucelindbloom.com/index.html?Eqn_RGB_XYZ_Matrix.html
        # sRGB
        # 0.4124564  0.3575761  0.1804375
        # 0.2126729  0.7151522  0.0721750
        # 0.0193339  0.1191920  0.9503041
        X = (RGB[0] * 0.4124564) + (RGB[1] * 0.3575761) + (RGB[2] * 0.1804375)
        Y = (RGB[0] * 0.2126729) + (RGB[1] * 0.7151522) + (RGB[2] * 0.0721750)
        Z = (RGB[0] * 0.0193339) + (RGB[1] * 0.1191920) + (RGB[2] * 0.9503041)

        XYZ[0] = X / 95.047  # ref_X =  95.047
        XYZ[1] = Y / 100.0  # ref_Y = 100.000
        XYZ[2] = Z / 108.883  # ref_Z = 108.883

        for (num, value) in enumerate(XYZ):
            if value > 0.008856:
                value = pow(value, (1.0 / 3.0))
            else:
                value = (7.787 * value) + (16 / 116.0)

            XYZ[num] = value

        L = (116.0 * XYZ[1]) - 16
        a = 500.0 * (XYZ[0] - XYZ[1])
        b = 200.0 * (XYZ[1] - XYZ[2])

        L = round(L, 4)
        a = round(a, 4)
        b = round(b, 4)

        return (L, a, b)

    @property
    def hsv(self):
        """
        HSV: Hue, Saturation, Value
        H: position in the spectrum
        S: color saturation ("purity")
        V: color brightness
        """
        (r, g, b) = self.rgb
        maxc = max(r, g, b)
        minc = min(r, g, b)
        v = maxc

        if minc == maxc:
            return 0.0, 0.0, v

        s = (maxc - minc) / maxc
        rc = (maxc - r) / (maxc - minc)
        gc = (maxc - g) / (maxc - minc)
        bc = (maxc - b) / (maxc - minc)

        if r == maxc:
            h = bc - gc
        elif g == maxc:
            h = 2.0 + rc - bc
        else:
            h = 4.0 + gc - rc

        h = (h / 6.0) % 1.0

        return (h, s, v)

    @property
    def hls(self):
        """
        HLS: Hue, Luminance, Saturation
        H: position in the spectrum
        L: color lightness
        S: color saturation
        """
        (red, green, blue) = self.rgb
        maxc = max(red, green, blue)
        minc = min(red, green, blue)
        luminance = (minc + maxc) / 2.0

        if minc == maxc:
            return 0.0, luminance, 0.0

        if luminance <= 0.5:
            saturation = (maxc - minc) / (maxc + minc)
        else:
            if 2.0 - maxc - minc == 0:
                saturation = 0
            else:
                saturation = (maxc - minc) / (2.0 - maxc - minc)

        rc = (maxc - red) / (maxc - minc)
        gc = (maxc - green) / (maxc - minc)
        bc = (maxc - blue) / (maxc - minc)

        if red == maxc:
            hue = bc - gc
        elif green == maxc:
            hue = 2.0 + rc - bc
        else:
            hue = 4.0 + gc - rc

        hue = (hue / 6.0) % 1.0

        return (hue, luminance, saturation)

    @property
    def red(self):
        """
        Red component of the detected color, in the range 0-1020.
        """
        self._ensure_mode(self.MODE_RGB_RAW)
        return self.raw[0]

    @property
    def green(self):
        """
        Green component of the detected color, in the range 0-1020.
        """
        self._ensure_mode(self.MODE_RGB_RAW)
        return self.raw[1]

    @property
    def blue(self):
        """
        Blue component of the detected color, in the range 0-1020.
        """
        self._ensure_mode(self.MODE_RGB_RAW)
        return self.raw[2]


class LightSensor(ColorSensor):
    """
    LEGO NXT Light Sensor
    """

    SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    #: Reflected light. LED on
    MODE_REFLECT = 'REFLECT'

    #: Ambient light. LED off
    MODE_AMBIENT = 'AMBIENT'

    MODES = (
        MODE_REFLECT,
        MODE_AMBIENT,
    )

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        super(LightSensor, self).__init__(address, name_pattern, name_exact, driver_name='lego-nxt-light', **kwargs)


class ButtonBase:
    pass


class InfraredSensor(Sensor, ButtonBase):
    """
    LEGO EV3 infrared sensor.
    """

    SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    #: Proximity
    MODE_IR_PROX = 'IR-PROX'

    #: IR Seeker
    MODE_IR_SEEK = 'IR-SEEK'

    #: IR Remote Control
    MODE_IR_REMOTE = 'IR-REMOTE'

    #: IR Remote Control. State of the buttons is coded in binary
    MODE_IR_REM_A = 'IR-REM-A'

    #: Calibration ???
    MODE_IR_CAL = 'IR-CAL'

    MODES = (MODE_IR_PROX, MODE_IR_SEEK, MODE_IR_REMOTE, MODE_IR_REM_A, MODE_IR_CAL)

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        # Sensor.__init__(self, address, name_pattern, name_exact, driver_name='lego-ev3-ir', **kwargs)
        self.mode = __class__.MODE_IR_PROX

    def read_sonar_sensor(self):
        emptyBuff = bytearray()
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'SensorSonar',
            [], [], [], emptyBuff,
            vrep.simx_opmode_oneshot_wait
        )
        returnCode, outInts, outFloats, outStrings, outBuffer = out
        return outFloats[0]

    @property
    def proximity(self):
        """
        An estimate of the distance between the sensor and objects in front of
        it, as a percentage. 100% is approximately 70cm/27in.
        """
        self._ensure_mode(self.MODE_IR_PROX)
        return self.read_sonar_sensor()


class UltrasonicSensor(Sensor):
    """
    LEGO EV3 ultrasonic sensor.
    """

    SYSTEM_CLASS_NAME = Sensor.SYSTEM_CLASS_NAME
    SYSTEM_DEVICE_NAME_CONVENTION = Sensor.SYSTEM_DEVICE_NAME_CONVENTION

    #: Continuous measurement in centimeters.
    MODE_US_DIST_CM = 'US-DIST-CM'

    #: Continuous measurement in inches.
    MODE_US_DIST_IN = 'US-DIST-IN'

    #: Listen.
    MODE_US_LISTEN = 'US-LISTEN'

    #: Single measurement in centimeters.
    MODE_US_SI_CM = 'US-SI-CM'

    #: Single measurement in inches.
    MODE_US_SI_IN = 'US-SI-IN'

    MODES = (
        MODE_US_DIST_CM,
        MODE_US_DIST_IN,
        MODE_US_LISTEN,
        MODE_US_SI_CM,
        MODE_US_SI_IN,
    )

    def __init__(self, address=None, name_pattern=SYSTEM_DEVICE_NAME_CONVENTION, name_exact=False, **kwargs):
        # super(UltrasonicSensor, self).__init__(address,
        #                                        name_pattern,
        #                                        name_exact,
        #                                        driver_name=['lego-ev3-us', 'lego-nxt-us'],
                                               # **kwargs)
        self.mode = __class__.MODE_US_DIST_CM

    def _scale(self, val):
        if val == 'US_DIST_IN':
            return 1 / 2.54
        return 1

    def read_sonar_sensor(self):
        emptyBuff = bytearray()
        out = vrep.simxCallScriptFunction(
            clientID,
            'Funciones',
            vrep.sim_scripttype_childscript,
            'SensorSonar',
            [], [], [], emptyBuff,
            vrep.simx_opmode_oneshot_wait
        )
        returnCode, outInts, outFloats, outStrings, outBuffer = out
        return outFloats[0]

    @property
    def distance_centimeters_continuous(self):
        """
        Measurement of the distance detected by the sensor,
        in centimeters.
        The sensor will continue to take measurements so
        they are available for future reads.
        Prefer using the equivalent :meth:`UltrasonicSensor.distance_centimeters` property.
        """
        self._ensure_mode(self.MODE_US_DIST_CM)
        return self.read_sonar_sensor()
        # self.value(0) * self._scale('US_DIST_CM')

    @property
    def distance_centimeters_ping(self):
        """
        Measurement of the distance detected by the sensor,
        in centimeters.
        The sensor will take a single measurement then stop
        broadcasting.
        If you use this property too frequently (e.g. every
        100msec), the sensor will sometimes lock up and writing
        to the mode attribute will return an error. A delay of
        250msec between each usage seems sufficient to keep the
        sensor from locking up.
        """
        # This mode is special; setting the mode causes the sensor to send out
        # a "ping", but the mode isn't actually changed.
        self.mode = self.MODE_US_SI_CM
        return self.distance_centimeters_continuous
        # return self.value(0) * self._scale('US_DIST_CM')

    @property
    def distance_centimeters(self):
        """
        Measurement of the distance detected by the sensor,
        in centimeters.
        Equivalent to :meth:`UltrasonicSensor.distance_centimeters_continuous`.
        """
        return self.distance_centimeters_continuous

    @property
    def distance_inches_continuous(self):
        """
        Measurement of the distance detected by the sensor,
        in inches.
        The sensor will continue to take measurements so
        they are available for future reads.
        Prefer using the equivalent :meth:`UltrasonicSensor.distance_inches` property.
        """
        self._ensure_mode(self.MODE_US_DIST_IN)
        return self.distance_centimeters_continuous * self._scale('US_DIST_IN')

    @property
    def distance_inches_ping(self):
        """
        Measurement of the distance detected by the sensor,
        in inches.
        The sensor will take a single measurement then stop
        broadcasting.
        If you use this property too frequently (e.g. every
        100msec), the sensor will sometimes lock up and writing
        to the mode attribute will return an error. A delay of
        250msec between each usage seems sufficient to keep the
        sensor from locking up.
        """
        # This mode is special; setting the mode causes the sensor to send out
        # a "ping", but the mode isn't actually changed.
        self.mode = self.MODE_US_SI_IN
        return self.distance_centimeters_continuous * self._scale('US_DIST_IN')

    @property
    def distance_inches(self):
        """
        Measurement of the distance detected by the sensor,
        in inches.
        Equivalent to :meth:`UltrasonicSensor.distance_inches_continuous`.
        """
        return self.distance_inches_continuous

    @property
    def other_sensor_present(self):
        """
        Boolean indicating whether another ultrasonic sensor could
        be heard nearby.
        """
        raise NotImplementedError('Please be patient, this will never come.')
