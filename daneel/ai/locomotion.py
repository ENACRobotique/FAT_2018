import math
from collections import namedtuple


def center_radians(value):
    while value <= - math.pi:
        value += 2 * math.pi
    while value > math.pi:
        value -= 2 * math.pi
    return value


class Locomotion:
    def __init__(self, robot):
        self.robot = robot
        self.x = 0
        self.y = 0
        self.theta = 0
        self.is_trajectory_finished = True
        self.is_stopped = True
        self.is_recalage_ended = False
        self.current_trajectory = []
        self._odometry_reports = {}  # type: dict[(int, int): (float, float, float)]
        self._latest_odometry_report = 0

    def handle_new_odometry_report(self, old_report_id, new_report_id, dx, dy, dtheta):
        if new_report_id > self._latest_odometry_report:
            if old_report_id < self._latest_odometry_report:
                # Need to find previous report and add the new information
                for ids, deltas in list(self._odometry_reports.items()):
                    if ids[0] == old_report_id and ids[1] == self._latest_odometry_report:
                        self.x -= deltas[0]
                        self.y -= deltas[1]
                        self.theta = center_radians(self.theta - deltas[2])
                        break
                #  in fact we search for old_report_id -> _latest_odemtry_report, but what we need is
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

        #  Delete all the old reports (those taken into account by the teensy)
        for ids, delta in list(self._odometry_reports.items()):
            if ids[1] <= old_report_id:
                del(self._odometry_reports[ids])

    def follow_trajectory(self, points, theta, speed):
        if len(points) > 10:
            raise AttributeError("Too much points given -- Message Size restriction")

        message = self.robot.communication.sMessageDown()
        message.message_type = self.robot.communication.eTypeDown.TRAJECTORY
        message.payload = self.robot.communication.sTrajectory()
        message.payload.nb_traj = len(points)
        message.payload.speed = speed
        message.payload.theta_final = theta
        message.payload.element = []
        for pt in points:
            traj_elt = self.robot.communication.sTrajElement()
            traj_elt.x = pt.x
            traj_elt.y = pt.y
            message.payload.element.append(traj_elt)
        self.robot.communication.send_message(message)

        for i, pt in enumerate(points):
            self.current_trajectory.append(trajectory_point(self.robot.communication._current_msg_id - 1, i, pt))
        self.is_trajectory_finished = False
        self.is_stopped = False
        if __debug__:
            print("[LOCOMOTION] Following trajectory {} at speed {} with final theta {}".format(points, speed, theta))

    def go_to_orient(self, x, y, theta, speed):
        message = self.robot.communication.sMessageDown()
        message.message_type = self.robot.communication.eTypeDown.TRAJECTORY
        message.payload = self.robot.communication.sTrajectory()
        message.payload.nb_traj = 1
        message.payload.speed = speed
        message.payload.theta_final = theta
        traj_elt = self.robot.communication.sTrajElement()
        traj_elt.x = x
        traj_elt.y = y
        message.payload.element = [traj_elt]
        self.robot.communication.send_message(message)
        self.current_trajectory.append(trajectory_point(self.robot.communication._current_msg_id - 1, 0, self.Point(x , y)))
        self.is_trajectory_finished = False
        self.is_stopped = False
        if __debug__:
            print("[LOCOMOTION] Go to {};{}, theta : {} at speed : {}".format(x, y, theta, speed))

    def go_to_orient_point(self, point, speed):
        self.go_to_orient(point.x, point.y, point.theta, speed)

    def stop_robot(self):
        message = self.robot.communication.sMessageDown()
        message.message_type = self.robot.communication.eTypeDown.STOP
        self.robot.communication.send_message(message)
        self.is_stopped = True
        if __debug__:
            print("[LOCOMOTION] Robot stopped")

    def restart_robot(self):
        message = self.robot.communication.sMessageDown()
        message.message_type = self.robot.communication.eTypeDown.RESTART
        self.robot.communication.send_message(message)
        self.is_stopped = self.is_trajectory_finished
        if __debug__:
            print("[LOCOMOTION] Robot restarted")

    def reposition_robot(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.synchronize_position()
        if __debug__:
            print("[LOCOMOTION] Set robot position @ ({},{},{})".format(x,y,theta))

    def do_recalage(self):
        message = self.robot.communication.sMessageDown()
        message.message_type = self.robot.communication.eTypeDown.DO_RECALAGE
        self.robot.communication.send_message(message)
        self.is_recalage_ended = False
        if __debug__:
            print("[LOCOMOTION] Repositionning...")

    def recalage_ok(self):
        self.is_recalage_ended = True
        self.is_stopped = True
        if __debug__:
            print("[LOCOMOTION] Repositionning ok !")


    def synchronize_position(self):
        message = self.robot.communication.sMessageDown()
        message.message_type = self.robot.communication.eTypeDown.REPOSITIONING
        message.payload = self.robot.communication.sRepositionning()
        message.payload.x = self.x
        message.payload.y = self.y
        message.payload.theta = self.theta
        self.robot.communication.send_message(message)

    def point_reached(self, traj_id, point_id, x, y, theta):
        if __debug__:
            print('[LOCOMOTION] Point reached, traj_id :{}, point_id:{}, @[{},{},{}]'.format(traj_id, point_id, x, y, theta))
        self.x = x
        self.y = y
        self.theta = theta
        index_to_remove = None
        for i, traj_elt in enumerate(self.current_trajectory):
            if traj_elt.traj_id == traj_id and traj_elt.point_number == point_id:
                index_to_remove = i
        if index_to_remove is not None:
            for i in range(index_to_remove + 1):
                self.current_trajectory.pop(0)
        else:
            raise IndexError("Reached Point is not stored !")
        if len(self.current_trajectory) == 0:
            self.is_trajectory_finished = True
            self.is_stopped = True
            if __debug__:
                print("[LOCOMOTION] Traj Finished !")

    def distance_to(self, x, y):
        return math.sqrt((self.x - x)**2 + (self.y - y)**2)

    class Point:
        def __init__(self, x, y):
            self.x = x
            self.y = y

    class PointOrient(Point):
        def __init__(self, x, y, theta=0):
            super().__init__(x, y)
            self.theta = theta


trajectory_point = namedtuple("TrajectoryPoint", ['traj_id','point_number','point'])
