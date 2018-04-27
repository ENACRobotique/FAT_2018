from enum import Enum
import time
import math
import os

from behavior import Behavior

END_MATCH_TIME = 100  # in seconds


class Color(Enum):
    GREEN = "green"
    ORANGE = "orange"


class FSMMatch(Behavior):
    def __init__(self, robot):
        self.robot = robot
        self.color = None
        self.start_time = None
        self.state = StateColorSelection(self)
        self.shutdown_button_press_time = 0

    def loop(self):
        time_now = time.time()
        if self.shutdown_button_press_time == 0 and self.robot.io.button1_state == self.robot.io.ButtonState.PRESSED:
            self.shutdown_button_press_time = time.time()
        elif self.shutdown_button_press_time != 0 and self.robot.io.button1_state == self.robot.io.ButtonState.RELEASED:
            self.shutdown_button_press_time = 0
        elif self.shutdown_button_press_time != 0 and time.time() - self.shutdown_button_press_time >= 5:
            self.robot.locomotion.stop()
            for i in range(3):
                self.robot.io.set_led_color(self.robot.io.LedColor.RED)
                time.sleep(0.5)  # Oh my god ! A "sleep" ! Don't worry baby, we are going to shut down any way.
                self.robot.io.set_led_color(self.robot.io.LedColor.BLACK)
                time.sleep(0.5)
            os.system("sudo shutdown -h now")
            exit(0)
        if self.start_time is not None and time_now - self.start_time >= END_MATCH_TIME and self.state.__class__ != StateEnd:
            if __debug__:
                print("[FSMMatch] End match")
            next_state = StateEnd
        else:
            next_state = self.state.test()
        if next_state is not None:
            if __debug__:
                print("[FSMMatch] Leaving {}, entering {}".format(self.state.__class__.__name__, next_state.__name__))
            self.state.deinit()
            self.state = next_state(self)

    def start_match(self):
        if __debug__:
            print("[FSMMatch] Match Started")
        self.start_time = time.time()


class FSMState:
    def __init__(self, behavior):
        self.behavior = behavior
        self.robot = self.behavior.robot

    def test(self):
        raise NotImplementedError("test of this state is not defined yet !")

    def deinit(self):
        raise NotImplementedError("deinit of this state is not defined yet !")


class StateColorSelection(FSMState):
    class ColorState(Enum):
        IDLE = "idle"
        PRESSED = "pressed"

    def __init__(self, behavior):
        super().__init__(behavior)
        self.behavior.color = Color.GREEN
        self.behavior.robot.io.set_led_color(self.behavior.robot.io.LedColor.GREEN)
        if self.robot.io.button2_state == self.robot.io.ButtonState.RELEASED and not self.behavior.color == Color.GREEN:
            self.behavior.color = Color.GREEN
            self.behavior.robot.io.set_led_color(self.behavior.robot.io.LedColor.GREEN)
        elif self.robot.io.button2_state == self.robot.io.ButtonState.PRESSED and not self.behavior.color == Color.ORANGE:
            self.behavior.color = Color.ORANGE
            self.robot.io.set_led_color(self.robot.io.LedColor.YELLOW)

    def test(self):
        if self.robot.io.button2_state == self.robot.io.ButtonState.RELEASED and not self.behavior.color == Color.GREEN:
            self.behavior.color = Color.GREEN
            self.behavior.robot.io.set_led_color(self.behavior.robot.io.LedColor.GREEN)
        elif self.robot.io.button2_state == self.robot.io.ButtonState.PRESSED and not self.behavior.color == Color.ORANGE:
            self.behavior.color = Color.ORANGE
            self.robot.io.set_led_color(self.robot.io.LedColor.YELLOW)

        if self.robot.io.cord_state == self.robot.io.CordState.OUT:
            return StatePreMatch

    def deinit(self):
        if self.behavior.color == Color.ORANGE:
            self.robot.locomotion.reposition_robot(2846, 1650, math.pi / 3)
        else:
            self.robot.locomotion.reposition_robot(154, 1650, 2 * math.pi / 3)
        self.robot.io.set_led_color(self.robot.io.LedColor.WHITE)

class StatePreMatch(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)

    def test(self):
        if self.robot.io.cord_state == self.behavior.robot.io.CordState.IN:
            return StateWaterCollectorTrajectory

    def deinit(self):
        self.behavior.start_match()

class StateWaterCollectorTrajectory(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        if self.behavior.color == Color.GREEN:
            self.robot.io.lower_bee_arm_green()
            self.robot.locomotion.go_to_orient(200, 1280, 2 * math.pi / 3)
        else:
            self.robot.io.lower_bee_arm_orange()
            self.robot.locomotion.go_to_orient(2800, 1280, 1.2)

    def test(self):
        if self.robot.locomotion.is_trajectory_finished():
            if self.behavior.color == Color.GREEN:
                return StateWaterCollectorGreen
            else:
                return StateWaterCollectorOrange

    def deinit(self):
        pass

class StateWaterCollectorGreen(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.time = 0
        self.robot.io.start_green_water_cannon()
        self.robot.io.start_green_water_collector()
        self.robot.locomotion.go_to_orient(190, 1210, 2 * math.pi / 3)

    def test(self):
        if self.time == 0 and self.robot.locomotion.is_trajectory_finished():
            self.time = time.time()
        if self.time != 0 and (time.time() - self.time) % 4 <= 1:
            self.robot.locomotion.set_direct_speed(-30, 0, 0)
        elif self.time != 0 and (time.time() - self.time) % 4 <= 2:
            self.robot.locomotion.set_direct_speed(0, -30, 0)
        elif self.time != 0 and (time.time() - self.time) % 4 <= 3:
            self.robot.locomotion.set_direct_speed(30, 0, 0)
        elif self.time != 0 and (time.time() - self.time) % 4 <= 4:
            self.robot.locomotion.set_direct_speed(0, 30, 0)
        if self.time != 0 and time.time() - self.time >= 20:
            return StateSwitchTrajectory

    def deinit(self):
        self.robot.io.stop_green_water_cannon()
        self.robot.io.stop_green_water_collector()


class StateWaterCollectorOrange(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.time = 0
        self.robot.io.start_orange_water_cannon()
        self.robot.io.start_orange_water_collector()
        self.robot.locomotion.go_to_orient(2810, 1210, 1.2)

    def test(self):
        if self.time == 0 and self.robot.locomotion.is_trajectory_finished():
            self.time = time.time()
        if self.time != 0 and (time.time() - self.time) % 4 <= 1:
            self.robot.locomotion.set_direct_speed(30, 0, 0)
        elif self.time != 0 and (time.time() - self.time) % 4 <= 2:
            self.robot.locomotion.set_direct_speed(0, 30, 0)
        elif self.time != 0 and (time.time() - self.time) % 4 <= 3:
            self.robot.locomotion.set_direct_speed(-30, 0, 0)
        elif self.time != 0 and (time.time() - self.time) % 4 <= 4:
            self.robot.locomotion.set_direct_speed(0, -30, 0)
        if self.time != 0 and time.time() - self.time >= 20:
            return StateEnd

    def deinit(self):
        self.robot.io.stop_orange_water_cannon()
        self.robot.io.stop_orange_water_collector()

class StateSwitchTrajectory(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.robot.io.raise_bee_arm_green()
        self.robot.locomotion.go_to_orient(1140, 1650, -math.pi / 2)

    def test(self):
        if not self.robot.io.arm_base_position_is_close(self.robot.io.ArmBaseState.FOR_SWITCH) and \
                self.robot.locomotion.is_trajectory_finished():
            self.robot.io.move_arm_base(self.robot.io.ArmBaseState.FOR_SWITCH)

        if self.robot.io.arm_base_position_is_close(self.robot.io.ArmBaseState.FOR_SWITCH):
            self.robot.locomotion.go_to_orient(1140, 1770, -math.pi / 2)
            return StateSwitch

    def deinit(self):
        pass

class StateSwitch(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)

    def test(self):
        if self.robot.locomotion.is_trajectory_finished():
            return StateEnd

    def deinit(self):
        self.robot.io.move_arm_base(self.robot.io.ArmBaseState.RAISED)


class StateEnd(FSMState):
    def __init__(self, behavior):
        super().__init__(behavior)
        self.behavior = behavior
        self.behavior.robot.locomotion.set_direct_speed(0,0,0)
        self.behavior.robot.locomotion.reposition_robot(0, 0, 0)  # To stop recalage if any
        self.behavior.robot.io.stop_green_water_collector()
        self.behavior.robot.io.stop_green_water_cannon()

    def test(self):
        pass

    def deinit(self):
        pass
