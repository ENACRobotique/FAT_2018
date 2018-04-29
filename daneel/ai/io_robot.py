from enum import *
import threading, serial

import math
import RPi.GPIO as GPIO

from drivers.neato_xv11_lidar import lidar_points, read_v_2_4
from drivers.vl6180x_laser import VL6180x, VL6180X_DEFAULT_I2C_ADDR, VL6180X_ERROR_NONE


LIDAR_SERIAL_PATH = "/dev/ttyUSB0"
LIDAR_SERIAL_BAUDRATE = 115200

BIT10_TO_BATTERY_FACTOR = 0.014774881516587679

ORANGE_LASER_SHUTDOWN_GPIO_PIN = 13  # In board number Todo: change it
INTER_LASER_DISTANCE = 100  # mm Todo : change it

class ActuatorID(Enum):
    WATER_COLLECTOR_GREEN = 0  # Dynamixel (not the Dynamixel id ! But the id defined in base/InputOutputs.h/eMsgActuatorId)
    WATER_COLLECTOR_ORANGE = 1  # Dynamixel (not the Dynamixel id ! But the id defined in base/InputOutputs.h/eMsgActuatorId)
    WATER_CANNON_GREEN = 2     # DC motor
    WATER_CANNON_ORANGE = 3    # DC motor
    SCORE_COUNTER = 4
    BEE_ARM_GREEN = 5    # Chinese servomotor
    BEE_ARM_ORANGE = 6   # Chinese servomotor


class IO(object):
    def __init__(self, robot):
        self.robot = robot
        self.cord_state = None
        self.button1_state = None
        self.button2_state = None
        self.led_color = None
        self.green_water_collector_state = None
        self.green_water_cannon_state = None
        self.orange_water_collector_state = None
        self.orange_water_cannon_state = None
        self.score_display_text = None
        self.battery_power_voltage = None
        self.battery_signal_voltage = None
        self.bee_arm_green_state = None
        self.bee_arm_orange_state = None
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(ORANGE_LASER_SHUTDOWN_GPIO_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(ORANGE_LASER_SHUTDOWN_GPIO_PIN, GPIO.LOW)
        self.green_laser = VL6180x(VL6180X_DEFAULT_I2C_ADDR)
        self.green_laser.change_address(VL6180X_DEFAULT_I2C_ADDR + 1)
        GPIO.output(ORANGE_LASER_SHUTDOWN_GPIO_PIN, GPIO.HIGH)
        self.orange_laser = VL6180x(VL6180X_DEFAULT_I2C_ADDR)
        self.lidar_serial = serial.Serial(LIDAR_SERIAL_PATH, LIDAR_SERIAL_BAUDRATE)
        self.lidar_thread = threading.Thread(target=read_v_2_4, args=(self.lidar_serial,))
        self.lidar_thread.start()
        self.robot.communication.register_callback(self.robot.communication.eTypeUp.HMI_STATE, self._on_hmi_state_receive)
        self.robot.communication.register_callback(self.robot.communication.eTypeUp.SENSOR_VALUE, self._on_sensor_value_receive)

        self.stop_green_water_cannon()
        self.stop_green_water_collector()
        self.stop_orange_water_cannon()
        self.stop_orange_water_collector()
        self.raise_bee_arm_orange()
        self.raise_bee_arm_green()
        self.score_display_fat()

    @property
    def lidar_points(self):
        return lidar_points[:] # returns a copy of the lidar point to avoid modification while iterating over the array

    @property
    def green_laser_distance(self):
        range = self.green_laser.range
        if self.green_laser.range_status == VL6180X_ERROR_NONE:
            return range
        return None

    @property
    def orange_laser_distance(self):
        range = self.orange_laser.range
        if self.orange_laser.range_status == VL6180X_ERROR_NONE:
            return range
        return None

    class SensorId(Enum):
        BATTERY_SIGNAL = 0
        BATTERY_POWER = 1

    class SensorState(Enum):
        STOPPED = 0
        ON_CHANGE = 1
        PERIODIC = 2

    def change_sensor_read_state(self, sensor_id: SensorId, sensor_state: SensorState):
        return self.robot.communication.send_sensor_command(sensor_id.value, sensor_state.value)

    class LedColor(Enum):
        BLACK = (False, False, False)
        RED = (True, False, False)
        GREEN = (False, True, False)
        BLUE = (False, False, True)
        YELLOW = (True, True, False)
        PURPLE = (True, False, True)
        CYAN = (False, True, True)
        WHITE = (True, True, True)

    class CordState(Enum):
        IN = "in"
        OUT = "out"

    class ButtonState(Enum):
        PRESSED = "pressed"
        RELEASED = "released"

    class WaterCollectorState(Enum):
        ACTIVATED = "activated"
        STOPPED = "stopped"

    class WaterCannonState(Enum):
        FIRING = "firing"
        STOPPED = "stopped"

    class ScoreDisplayTexts(Enum):  # As defined in base/InputOutputs.cpp/InputOutputs::handleActuatorMessage
        ENAC = 20001
        FAT = 20002

    class BeeArmState(Enum):
        # [green side degrees, orange side degrees]
        RAISED = [40, 170]
        LOWERED = [105, 98]

    def start_green_water_collector(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_COLLECTOR_GREEN.value, 1) == 0:
            self.green_water_collector_state = self.WaterCollectorState.ACTIVATED
            if __debug__:
                print("[IO] Start green water collector")

    def start_orange_water_collector(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_COLLECTOR_ORANGE.value, 1) == 0:
            self.orange_water_collector_state = self.WaterCollectorState.ACTIVATED
            if __debug__:
                print("[IO] Start orange water collector")

    def stop_green_water_collector(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_COLLECTOR_GREEN.value, 0) == 0:
            self.green_water_collector_state = self.WaterCollectorState.STOPPED
            if __debug__:
                print("[IO] Stop green water collector")

    def stop_orange_water_collector(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_COLLECTOR_ORANGE.value, 0) == 0:
            self.orange_water_collector_state = self.WaterCollectorState.STOPPED
            if __debug__:
                print("[IO] Stop orange water collector")

    def start_green_water_cannon(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_CANNON_GREEN.value, 75) == 0:
            self.green_water_cannon_state = self.WaterCannonState.FIRING
            if __debug__:
                print("[IO] Start green water cannon")

    def start_orange_water_cannon(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_CANNON_ORANGE.value, 75) == 0:
            self.orange_water_cannon_state = self.WaterCannonState.FIRING
            if __debug__:
                print("[IO] Start orange water cannon")

    def stop_green_water_cannon(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_CANNON_GREEN.value, 0) == 0:
            self.green_water_cannon_state = self.WaterCannonState.STOPPED
            if __debug__:
                print("[IO] Stop green water cannon")

    def stop_orange_water_cannon(self):
        if self.robot.communication.send_actuator_command(ActuatorID.WATER_CANNON_ORANGE.value, 0) == 0:
            self.orange_water_cannon_state = self.WaterCannonState.STOPPED
            if __debug__:
                print("[IO] Stop orange water cannon")

    def set_led_color(self, color):
        if self.robot.communication.send_hmi_command(*color.value) == 0:
            self.led_color = color
            if __debug__:
                print("[IO] Led switched to {}".format(color))

    def score_display_fat(self):
        if self.robot.communication.send_actuator_command(ActuatorID.SCORE_COUNTER.value, self.ScoreDisplayTexts.FAT.value) == 0:
            self.score_display_text = "FAT"
            print("[IO] Score display displays " + self.score_display_text)

    def score_display_enac(self):
        if self.robot.communication.send_actuator_command(ActuatorID.SCORE_COUNTER.value, self.ScoreDisplayTexts.ENAC.value) == 0:
            self.score_display_text = "ENAC"
            print("[IO] Score display displays " + self.score_display_text)

    def score_display_number(self, number, with_two_points=False):
        command = number
        if with_two_points:
            command += 10000
        if self.robot.communication.send_actuator_command(ActuatorID.SCORE_COUNTER.value, command) == 0:
            if with_two_points:
                self.score_display_text = str(number)[:-2] + ":" + str(number)[-2:]
                #FIXME: Not working with number = 4 (text = ":4" instead of ": 4")
            else:
                self.score_display_text = str(number)
            print("[IO] Score display displays " + self.score_display_text)

    def raise_bee_arm_green(self):
        if self.robot.communication.send_actuator_command(ActuatorID.BEE_ARM_GREEN.value, self.BeeArmState.RAISED.value[0]) == 0:
            self.bee_arm_green_state = self.BeeArmState.RAISED
            print("[IO] Green bee arm raised")

    def raise_bee_arm_orange(self):
        if self.robot.communication.send_actuator_command(ActuatorID.BEE_ARM_ORANGE.value, self.BeeArmState.RAISED.value[1]) == 0:
            self.bee_arm_orange_state = self.BeeArmState.RAISED
            print("[IO] Orange bee arm raised")

    def lower_bee_arm_green(self):
        if self.robot.communication.send_actuator_command(ActuatorID.BEE_ARM_GREEN.value, self.BeeArmState.LOWERED.value[0]) == 0:
            self.bee_arm_green_state = self.BeeArmState.LOWERED
            print("[IO] Green bee arm lowered")

    def lower_bee_arm_orange(self):
        if self.robot.communication.send_actuator_command(ActuatorID.BEE_ARM_ORANGE.value, self.BeeArmState.LOWERED.value[1]) == 0:
            self.bee_arm_orange_state = self.BeeArmState.LOWERED
            print("[IO] Orange bee arm lowered")

    def find_laser_angle(self):
        range_green = self.green_laser_distance
        range_orange = self.orange_laser_distance
        if range_green is not None and range_orange is not None:
            return math.atan((range_green - range_orange) / INTER_LASER_DISTANCE)
        return None

    def _on_hmi_state_receive(self, cord_state, button1_state, button2_state, red_led_state, green_led_state, blue_led_state):
        self.cord_state = self.CordState.IN if cord_state else self.CordState.OUT
        self.button1_state = self.ButtonState.RELEASED if button1_state else self.ButtonState.PRESSED
        self.button2_state = self.ButtonState.RELEASED if button2_state else self.ButtonState.PRESSED
        self.led_color = self.LedColor((red_led_state, green_led_state, blue_led_state))

    def _on_sensor_value_receive(self, sensor_id, sensor_value):
        if sensor_id == self.SensorId.BATTERY_SIGNAL.value:
            self.battery_signal_voltage = self._bit10_to_battery_voltage(sensor_value)
        elif sensor_id == self.SensorId.BATTERY_POWER.value:
            self.battery_power_voltage = self._bit10_to_battery_voltage(sensor_value)

    def _bit10_to_battery_voltage(self, bit10):
        return bit10 * BIT10_TO_BATTERY_FACTOR

    def is_obstacle_in_cone(self, direction, cone_angle, distance):
        direction = round(math.degrees(direction))
        for i in range(len(self.lidar_points)):
            pt = self.lidar_points[i]
            if pt is None:
                return
            a = (pt.azimut - direction + 180) % 360 - 180
            if abs(a) <= cone_angle:

                #print(pt.azimut)
                #print(pt.distance)
                if pt.valid and not pt.warning and pt.distance < distance:
                    x_t = self.robot.locomotion.x + pt.distance * math.cos(
                        math.radians(pt.azimut) + self.robot.locomotion.theta)
                    y_t = self.robot.locomotion.y + pt.distance * math.sin(
                        math.radians(pt.azimut) + self.robot.locomotion.theta)
                    if not self.robot.map.lidar_table_bb.contains(x_t, y_t):
                        continue
                    in_mask = False
                    for mask in self.robot.map.lidar_static_obstacles_bb:
                        if mask.contains(x_t, y_t):
                            self.robot.ivy.highlight_point(50, x_t, y_t)
                            in_mask = True
                            break
                    if in_mask:
                        continue
                    self.robot.ivy.highlight_point(51, x_t, y_t)
                    print(x_t, y_t)
                    print(pt.azimut, pt.distance)
                    return True
        return False


# class USReader(threading.Thread):
#     def __init__(self):
#         threading.Thread.__init__(self)
#         # self.i2c = smbus.SMBus(1)
#
#     def run(self):
#         global us_sensors, us_sensors_distance
#         while True:
#             for i, sensor in enumerate(us_sensors):
#                 self.i2c.write_byte_data(sensor.address, 0, 81)
#             time.sleep(0.070)
#             for i, sensor in enumerate(us_sensors):
#                 dst = self.i2c.read_word_data(sensor.address, 2) / 255
#                 if dst != 0:
#                     us_sensors_distance[us_sensors[i]] = dst
