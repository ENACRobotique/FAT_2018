import math
import time
from collections import namedtuple
from enum import Enum

ACCELERATION_MAX = 300  # mm/s²
LINEAR_SPEED_MAX = 200  # mm/s
ADMITTED_POSITION_ERROR = 10  # mm

ROTATION_ACCELERATION_MAX = 0.8  # rad/s²
ROTATION_SPEED_MAX = 0.7  # rad/s
ADMITTED_ANGLE_ERROR = 0.05  # rad

Speed = namedtuple("Speed", ['vx', 'vy', 'vtheta'])
GoalPoint = namedtuple("GoalPoint", ['goal_point', 'goal_speed'])


def center_radians(value):
    while value <= - math.pi:
        value += 2 * math.pi
    while value > math.pi:
        value -= 2 * math.pi
    return value


class Locomotion:
    class LocomotionState(Enum):
        POSITION_CONTROL = 0
        DIRECT_SPEED_CONTROL = 1
        STOPPED = 2

    def __init__(self, robot):
        self.robot = robot
        self.x = 0
        self.y = 0
        self.theta = 0
        self.previous_mode = None
        self.mode = self.LocomotionState.POSITION_CONTROL

        # Position Control
        # self.trajectory[0].goal_point must be equal to self.current_point_objective
        self.trajectory = []  # type: list[GoalPoint]
        self.current_point_objective = None  # type: self.PointOrient
        self.position_control_speed_goal = 0

        # Direct speed control
        self.direct_speed_goal = Speed(0, 0, 0)  # for DIRECT_SPEED_CONTROL_MODE

        # Locomotion stopped
        self.time_at_stop = 0  # 0 if not stopped, time.time() when locomotion enters in STOPPED mode.

        self.current_speed = Speed(0, 0, 0)  # type: Speed
        self.robot.communication.register_callback(self.robot.communication.eTypeUp.ODOM_REPORT,
                                                   self.handle_new_odometry_report)
        self._last_position_control_time = None
        self._odometry_reports = {}  # type: dict[(int, int): (float, float, float)]
        self._latest_odometry_report = 0

    def handle_new_odometry_report(self, old_report_id, new_report_id, dx, dy, dtheta):
        if (new_report_id - self._latest_odometry_report + 256) % 256 < 128:
            if old_report_id - self._latest_odometry_report > 128:
                #  Need to find previous report and add the new information
                for ids, deltas in list(self._odometry_reports.items()):
                    if ids[0] == old_report_id and ids[1] == self._latest_odometry_report:
                        self.x -= deltas[0]
                        self.y -= deltas[1]
                        self.theta = center_radians(self.theta - deltas[2])
                        break
                # in fact we search for old_report_id -> _latest_odemtry_report, but what we need is
                #  a CHAIN going from old_report_id to _latest_odemetry_report and substract each node (and add them if
                # they overlap).

                self.x += dx
                self.y += dy
                self.theta = center_radians(self.theta + dtheta)

                self._odometry_reports[(old_report_id, new_report_id)] = (dx, dy, dtheta)
            elif old_report_id == self._latest_odometry_report:
                # Nominal case, the new report brings only new information
                self.x += dx
                self.y += dy
                self.theta = center_radians(self.theta + dtheta)

                self._odometry_reports[(old_report_id, new_report_id)] = (dx, dy, dtheta)
            else:
                # Should not happen, information has been missed
                pass
            self._latest_odometry_report = new_report_id
        elif new_report_id == self._latest_odometry_report:
            # We already have this information, but it may worth to update it
            pass
        else:
            # Only old information; it probably don't worth to update it...
            pass

        # Delete all the old reports (those taken into account by the teensy)
        for ids, delta in list(self._odometry_reports.items()):
            if (ids[1] - old_report_id + 256) % 256 == 0 or (ids[1] - old_report_id + 256) % 256 > 128:
                del (self._odometry_reports[ids])

    def go_to_orient(self, x, y, theta):
        self.mode = self.LocomotionState.POSITION_CONTROL
        self.trajectory.clear()
        self.trajectory.append(GoalPoint(self.PointOrient(x, y, center_radians(theta)), 0.))
        self.current_point_objective = self.trajectory[0].goal_point
        self.position_control_speed_goal = self.trajectory[0].goal_speed

    def go_to_orient_point(self, point):
        self.go_to_orient(point.x, point.y, point.theta)

    def handle_obstacle(self, speed, detection_angle, stop_distance):
        if math.hypot(speed.vx, speed.vy) < 0.01:
            return
        a = math.atan2(speed.vy, speed.vx)
        if self.robot.io.is_obstacle_in_cone(a, detection_angle, stop_distance):
            if self.mode != self.LocomotionState.STOPPED:
                self.stop()
        else:
            if self.mode == self.LocomotionState.STOPPED:
                self.restart()

    def is_at_point_orient(self, point=None):
        if point is None:
            point = self.current_point_objective
        if self.mode != self.LocomotionState.POSITION_CONTROL and self.mode != self.LocomotionState.STOPPED:
            return True
        if point is None:
            return True
        return self.distance_to(point.x, point.y) <= ADMITTED_POSITION_ERROR \
            and abs(self.theta - point.theta) <= ADMITTED_ANGLE_ERROR

    def is_trajectory_finished(self):
        if len(self.trajectory) == 0:
            return self.is_at_point_orient()
        return False

    def locomotion_loop(self, obstacle_detection=False):
        control_time = time.time()
        if self._last_position_control_time is None:
            delta_time = 0
        else:
            delta_time = control_time - self._last_position_control_time
        self._last_position_control_time = control_time

        if self.mode == self.LocomotionState.STOPPED:
            speed = Speed(0, 0, 0)

        elif self.mode == self.LocomotionState.POSITION_CONTROL:
            speed = self.position_control_loop(delta_time)
        elif self.mode == self.LocomotionState.DIRECT_SPEED_CONTROL:
            if self.direct_speed_goal is not None:
                speed = self.direct_speed_goal
            else:
                speed = Speed(0, 0, 0)
        else:
            # This should not happen
            speed = Speed(0, 0, 0)
        # print("Speed wanted : " + str(speed))
        # self.current_speed = self.comply_speed_constraints(speed, delta_time)
        # print("Speed after saturation : " + str(self.current_speed))

        if obstacle_detection:
            if self.mode == self.LocomotionState.STOPPED:
                if self.previous_mode == self.LocomotionState.POSITION_CONTROL:
                    wanted_speed = self.position_control_loop(delta_time)
                elif self.previous_mode == self.LocomotionState.DIRECT_SPEED_CONTROL:
                    wanted_speed = self.direct_speed_goal
                else:
                    wanted_speed = Speed(0, 0, 0)
            else:
                wanted_speed = speed
            vx_r = wanted_speed.vx * math.cos(self.theta) + wanted_speed.vy * math.sin(self.theta)
            vy_r = wanted_speed.vx * -math.sin(self.theta) + wanted_speed.vy * math.cos(self.theta)
            self.handle_obstacle(Speed(vx_r, vy_r, 0), 45, 500)
        self.current_speed = speed
        self.robot.communication.send_speed_command(*self.current_speed)

    def stop(self):
        self.time_at_stop = time.time()
        self.previous_mode = self.mode
        self.mode = self.LocomotionState.STOPPED

    def restart(self):
        self.time_at_stop = 0
        self.mode = self.previous_mode

    def set_direct_speed(self, x_speed, y_speed, theta_speed):
        if self.mode == self.LocomotionState.STOPPED:
            self.previous_mode = self.LocomotionState.DIRECT_SPEED_CONTROL
        else:
            self.mode = self.LocomotionState.DIRECT_SPEED_CONTROL
        self.direct_speed_goal = Speed(x_speed, y_speed, theta_speed)

    def position_control_loop(self, delta_time):
        if len(self.trajectory) > 0:
            if self.is_at_point_orient():
                # We reached a point in the trajectory, remove it.
                self.trajectory.pop(0)
                if len(self.trajectory) > 0:
                    # Go to next point in the trajectory
                    self.current_point_objective = self.trajectory[0].goal_point
                    self.position_control_speed_goal = self.trajectory[0].goal_speed

        if self.current_point_objective is not None:
            self.robot.ivy.highlight_point(0, self.current_point_objective.x, self.current_point_objective.y)
            distance_to_objective = self.current_point_objective.lin_distance_to(self.x, self.y)
            # if distance_to_objective <= ADMITTED_POSITION_ERROR:
            #     self.current_point_objective = None
            #     return

            # Acceleration part
            alpha = math.atan2(self.current_point_objective.y - self.y, self.current_point_objective.x - self.x)
            current_linear_speed = math.hypot(self.current_speed.vx, self.current_speed.vy)
            new_speed = min((LINEAR_SPEED_MAX, current_linear_speed + delta_time * ACCELERATION_MAX))

            # Check if we need to decelerate
            if new_speed > self.position_control_speed_goal:
                t_reach_speed = (current_linear_speed - self.position_control_speed_goal) / ACCELERATION_MAX
                reach_speed_length = 2 * (
                    current_linear_speed * t_reach_speed - 1 / 2 * ACCELERATION_MAX * t_reach_speed ** 2)  # Why 2 times ? Don't know, without the factor, it is largely underestimated... Maybe because of the reaction time of the motors ?
                # current_speed_alpha = math.atan2(self.current_speed[1], self.current_speed[0])
                planned_stop_point = self.Point(self.x + reach_speed_length * math.cos(alpha),
                                                self.y + reach_speed_length * math.sin(alpha))
                self.robot.ivy.highlight_point(1, planned_stop_point.x, planned_stop_point.y)
                planned_stop_error = planned_stop_point.lin_distance_to_point(self.current_point_objective)
                if planned_stop_error <= ADMITTED_POSITION_ERROR or planned_stop_point.lin_distance_to(self.x,
                                                                                                       self.y) > distance_to_objective:
                    # Deceleration time
                    new_speed = max((0, current_linear_speed - ACCELERATION_MAX * delta_time))

            # Rotation part
            omega = min((ROTATION_SPEED_MAX, abs(self.current_speed.vtheta) + delta_time * ROTATION_ACCELERATION_MAX))
            rotation_error_sign = math.copysign(1, center_radians(self.current_point_objective.theta - self.theta))
            t_rotation_stop = abs(self.current_speed.vtheta) / ROTATION_ACCELERATION_MAX
            planned_stop_angle = center_radians(
                self.theta + rotation_error_sign * 2.5 * (
                    abs(
                        self.current_speed.vtheta) * t_rotation_stop - 1 / 2 * ROTATION_ACCELERATION_MAX * t_rotation_stop ** 2))
            self.robot.ivy.highlight_robot_angle(0, self.current_point_objective.theta)
            self.robot.ivy.highlight_robot_angle(1, planned_stop_angle)
            if abs(center_radians(
                            planned_stop_angle - self.current_point_objective.theta)) <= ADMITTED_ANGLE_ERROR or abs(
                center_radians(planned_stop_angle - self.theta)) > abs(
                center_radians(self.current_point_objective.theta - self.theta)):
                omega = max((0, abs(self.current_speed.vtheta) - ROTATION_ACCELERATION_MAX * delta_time))

            speed_command = Speed(new_speed * math.cos(alpha), new_speed * math.sin(alpha), math.copysign(
                omega, rotation_error_sign))
        else:
            speed_command = Speed(0, 0, 0)
        return speed_command

    def comply_speed_constraints(self, speed_cmd, dt, alpha_step=0.05):
        if dt == 0:
            return Speed(0, 0, 0)

        # Verify if maximum acceleration is respected
        alpha = alpha_step
        vx, vy, vtheta = speed_cmd
        while math.hypot((vx - self.current_speed.vx) / dt,
                         (vy - self.current_speed.vy) / dt) > ACCELERATION_MAX:
            print("diff : {}".format(math.hypot(vx - self.current_speed.vx,
                                                vy - self.current_speed.vy)))
            vx = speed_cmd.vx * (1 - alpha) + self.current_speed.vx * alpha
            vy = speed_cmd.vy * (1 - alpha) + self.current_speed.vy * alpha
            alpha += alpha_step

        alpha = alpha_step
        while abs(vtheta - self.current_speed.vtheta) / dt > ROTATION_ACCELERATION_MAX:
            vtheta = speed_cmd.vtheta * (1 - alpha) + self.current_speed.vtheta * alpha
            alpha += alpha_step

        return Speed(vx, vy, vtheta)

    def distance_to(self, x, y):
        return math.sqrt((self.x - x) ** 2 + (self.y - y) ** 2)

    def reposition_robot(self, x, y, theta):
        self.x = x
        self.y = y
        if self.robot.communication.send_theta_repositionning(theta) == 0:
            self.theta = theta

    def follow_trajectory(self, points_list):
        """

        :param points_list:
        :type points_list: list[tuple[int, int]]|list[Locomotion.PointOrient]
        :return:
        """
        self.mode = self.LocomotionState.POSITION_CONTROL
        self.trajectory.clear()
        if len(points_list) > 0:
            for i, pt in enumerate(points_list):
                if i != len(points_list) - 1:
                    xe = points_list[i + 1][0]
                    ye = points_list[i + 1][1]

                    if i == 0:
                        xs = self.x
                        ys = self.y
                    else:
                        xs = points_list[i - 1][0]
                        ys = points_list[i - 1][1]
                    a1 = math.atan2(pt[1] - ys, pt[0] - xs)
                    a2 = math.atan2(ye - pt[1], xe - pt[0])
                    angle = center_radians(abs(a1) - abs(a2))
                    if abs(angle) >= math.pi / 2:
                        goal_speed = 0.
                    else:
                        goal_speed = LINEAR_SPEED_MAX * (1 - abs(angle) / (math.pi / 2))
                else:
                    goal_speed = 0.

                self.trajectory.append(GoalPoint(self.PointOrient(pt[0], pt[1], pt[2]), goal_speed))
        self.current_point_objective = self.trajectory[0].goal_point
        self.position_control_speed_goal = self.trajectory[0].goal_speed

    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y

        def lin_distance_to_point(self, pt):
            return self.lin_distance_to(pt.x, pt.y)

        def lin_distance_to(self, x, y):
            return math.hypot(x - self.x, y - self.y)

        def __getitem__(self, item):
            if item == 0 or item == 'x':
                return self.x
            elif item == 1 or item == 'y':
                return self.y

    class PointOrient(Point):
        def __init__(self, x, y, theta=0):
            super().__init__(x, y)
            self.theta = theta

        def __getitem__(self, item):
            if item == 2 or item == "theta":
                return self.theta
            else:
                return super(Locomotion.PointOrient, self).__getitem__(item)
