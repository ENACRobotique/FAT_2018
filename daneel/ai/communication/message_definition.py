from enum import Enum
import bitstring


UP_MESSAGE_SIZE = 17  # maximum size of a up message (teensy -> raspi) in bytes
UP_HEADER_SIZE = 3  # size of the header (all except the data) of an up message
DOWN_MESSAGE_SIZE = 9  # maximum size of a down message (raspi -> teensy) in bytes

# Data converters (from and to what is sent over the wire and what is used as data).
LINEAR_ODOM_TO_MSG_ADDER = 32768
LINEAR_SPEED_TO_MSG_ADDER = 32768
ANGULAR_SPEED_TO_MSG_FACTOR = 1043.0378350470453
ANGULAR_SPEED_TO_MSG_ADDER = 10 * 3.14159265358979323846
RADIAN_TO_MSG_FACTOR = 10430.378350470453
RADIAN_TO_MSG_ADDER = 3.14159265358979323846


class DeserializationException(Exception):
    """
    Raised on deserialization error (when trying to read what is sent over the wire into usable data)
    """
    def __init__(self, message):
        """
        Ctor of the deserialization exception.

        :param message: Message that will be thrown with the exception (and may be printed on the user screen)
        :type message: str
        """
        super().__init__(message)


# ======= Up (Prop -> raspi) message declaration ======== #

class eTypeUp(Enum):
    """
    All the message types that can be sent from the base (Teensy) to the ai.
    Must be the same as base/code/communication/Communication.h:eUpMessageType
    """
    ACK_DOWN = 0
    ODOM_REPORT = 1
    HMI_STATE = 2
    SENSOR_VALUE = 3


class sAckDown:
    """
    Payload of an up message (from base to ai) acknowledging a down message (from ai to base).
    """
    def __init__(self):
        """
        Ctor
        """
        self.ack_down_id = None  # uint:8

    def deserialize(self, bytes_packed):
        """
        Fills the id of the acknowledged message with no alteration from what is received from the wire.

        :param bytes_packed: The bytes received from serial.
        :type bytes_packed: list[bytes]
        """
        s = bitstring.BitStream(bytes_packed)
        self.ack_down_id, = s.unpack('uint:8')

    def serialize(self):
        """
        Not used.

        :return:
        :rtype:
        """
        return bitstring.pack('uint:8', self.ack_down_id)


class sOdomReport:
    def __init__(self):
        self.previous_report_id = None  # uint:8
        self.new_report_id = None  # uint:8
        self._dx = None  # uint:16
        self._dy = None  # uint:16
        self._dtheta = None  # uint:16
        self._vx = None
        self._vy = None
        self._vtheta = None

    @property
    def dx(self):
        return self._dx - LINEAR_ODOM_TO_MSG_ADDER

    @dx.setter
    def dx(self, dx):
        self._dx = dx + LINEAR_ODOM_TO_MSG_ADDER

    @property
    def dy(self):
        return self._dy - LINEAR_ODOM_TO_MSG_ADDER

    @dy.setter
    def dy(self, dy):
        self._dy = dy + LINEAR_ODOM_TO_MSG_ADDER

    @property
    def dtheta(self):
        return self._dtheta / RADIAN_TO_MSG_FACTOR - RADIAN_TO_MSG_ADDER

    @dtheta.setter
    def dtheta(self, dtheta):
        self._dtheta = (dtheta + RADIAN_TO_MSG_ADDER) * RADIAN_TO_MSG_FACTOR

    @property
    def vx(self):
        return self._vx - LINEAR_SPEED_TO_MSG_ADDER

    @property
    def vy(self):
        return self._vy - LINEAR_SPEED_TO_MSG_ADDER

    @property
    def vtheta(self):
        return self._vtheta / ANGULAR_SPEED_TO_MSG_FACTOR - ANGULAR_SPEED_TO_MSG_ADDER

    def deserialize(self, bytes_packed):
        s = bitstring.BitStream(bytes_packed)
        self.previous_report_id, self.new_report_id, self._dx, self._dy, self._dtheta, self._vx,\
            self._vy, self._vtheta = s.unpack(
            'uint:8, uint:8, uintle:16, uintle:16, uintle:16, uintle:16, uintle:16, uintle:16')

    def serialize(self):
        return bitstring.pack('uint:8, uint:8, uintle:16, uintle:16, uintle:16',
                              self.previous_report_id, self.new_report_id,
                              self._dx, self._dy, self._dtheta)


class sHMIState:
    def __init__(self):
        self.hmi_state = None  # uint:8

    def deserialize(self, bytes_packed):
        s = bitstring.BitStream(bytes_packed)
        self.hmi_state, = s.unpack('uint:8')

    def serialize(self):
        return bitstring.pack('uint:8', self.hmi_state)


class sSensorValue:
    def __init__(self):
        self.sensor_id = None
        self.sensor_value = None

    def deserialize(self, bytes_packed):
        s = bitstring.BitStream(bytes_packed)
        self.sensor_id, self.sensor_value = s.unpack('uint:8, uintle:16')


class sMessageUp:
    """
    Class defining the up (teensy -> raspi) messages
    :type type: eTypeUp
    :type up_id: int
    :type checksum: int
    :type data: sAckDown|sHMIState|sOdomReport|sSensorValue
    """
    def __init__(self):
        self.up_id = None
        self.type = None
        self.checksum = None
        self.data = None

    def deserialize(self, packed):
        header = bitstring.BitStream(packed[0:UP_HEADER_SIZE])
        try:
            self.up_id, type_value, self.data = header.unpack('uint:8, uint:8, uint:8')
            self.type = eTypeUp(type_value)
        except ValueError as e:
            raise DeserializationException("Can't deserialize up message header : {}".format(str(e)))

        if self.type == eTypeUp.ACK_DOWN:
            self.data = sAckDown()
        elif self.type == eTypeUp.HMI_STATE:
            self.data = sHMIState()
        elif self.type == eTypeUp.ODOM_REPORT:
            self.data = sOdomReport()
        elif self.type == eTypeUp.SENSOR_VALUE:
            self.data = sSensorValue()
        try:
            self.data.deserialize(packed[UP_HEADER_SIZE:])
        except ValueError as e:
            raise DeserializationException("Can't deserialize up message payload : {}".format(str(e)))

    def serialize(self):

        ser2 = None
        if self.data is not None:
            ser2 = self.data.serialize()
            self.checksum = 0
            for octet in ser2.tobytes():
                self.checksum ^= octet
            self.checksum = self.checksum % 0xFF

        ser = bitstring.pack('uint:8, uint:8, uint:8', self.up_id, self.type.value, self.checksum)
        serialized_msg = ser + ser2
        pad = bitstring.pack('pad:{}'.format((DOWN_MESSAGE_SIZE - len(serialized_msg.tobytes())) * 8))
        return serialized_msg + pad


# ====== End up message declaration
# ====== Down (raspi -> prop) message declaration #


class eTypeDown(Enum):
    ACK_UP = 0
    ACK_ODOM_REPORT = 1
    SPEED_COMMAND = 2
    ACTUATOR_COMMAND = 3
    HMI_COMMAND = 4
    RESET = 5
    THETA_REPOSITIONING = 6
    SENSOR_COMMAND = 7


class sAckUp:
    def __init__(self):
        self.ack_up_id = None  # uint:8

    def serialize(self):
        return bitstring.pack('uint:8', self.ack_up_id)


class sAckOdomReport:
    def __init__(self):
        self.ack_up_id = None  # uint:8
        self.ack_odom_report_id = None  # uint:8

    def serialize(self):
        return bitstring.pack('uint:8, uint:8', self.ack_up_id, self.ack_odom_report_id)


class sSpeedCommand:
    def __init__(self):
        self._vx = None  # uint:16
        self._vy = None  # uint:16
        self._vtheta = None  # uint:16

    @property
    def vx(self):
        return self._vx - LINEAR_SPEED_TO_MSG_ADDER

    @vx.setter
    def vx(self, vx):
        self._vx = vx + LINEAR_SPEED_TO_MSG_ADDER

    @property
    def vy(self):
        return self._vy - LINEAR_SPEED_TO_MSG_ADDER

    @vy.setter
    def vy(self, vy):
        self._vy = vy + LINEAR_SPEED_TO_MSG_ADDER

    @property
    def vtheta(self):
        return self._vtheta / ANGULAR_SPEED_TO_MSG_FACTOR - ANGULAR_SPEED_TO_MSG_ADDER

    @vtheta.setter
    def vtheta(self, vtheta):
        self._vtheta = (vtheta + ANGULAR_SPEED_TO_MSG_ADDER) * ANGULAR_SPEED_TO_MSG_FACTOR

    def serialize(self):
        return bitstring.pack('uintle:16, uintle:16, uintle:16', self._vx, self._vy,
                              self._vtheta)


class sActuatorCommand:
    def __init__(self):
        self.actuator_id = None  # uint:8
        self.actuator_command = None  # uint:16

    def serialize(self):
        return bitstring.pack('uint:8, uintle:16', self.actuator_id, self.actuator_command)


class sHMICommand:
    def __init__(self):
        self.hmi_command = None

    def serialize(self):
        return bitstring.pack('uint:8', self.hmi_command)


class sThetaRepositioning:
    def __init__(self):
        self._theta_repositioning = None

    @property
    def theta_repositioning(self):
        return self._theta_repositioning / RADIAN_TO_MSG_FACTOR - RADIAN_TO_MSG_ADDER

    @theta_repositioning.setter
    def theta_repositioning(self, value):
        self._theta_repositioning = round((value + RADIAN_TO_MSG_ADDER) * RADIAN_TO_MSG_FACTOR)

    def serialize(self):
        return bitstring.pack('uintle:16', self._theta_repositioning)


class sSensorCommand:
    def __init__(self):
        self.sensor_id = None
        self.sensor_state = None

    def serialize(self):
        return bitstring.pack('uint:8, uint:8', self.sensor_id, self.sensor_state)

class sMessageDown:
    """
    Class defining the down (raspi -> teensy) messages
    :type type: eTypeDown
    :type down_id: int
    :type checksum: int
    :type data: sAckOdomReport|sAckUp|sActuatorCommand|sSpeedCommand|sHMICommand|sThetaRepositioning
    """
    def __init__(self):
        self.down_id = 0  # :8
        self.type = None  # :8
        self.checksum = 0  # :8
        self.data = None

    def serialize(self):

        ser2 = None
        if self.data is not None:
            ser2 = self.data.serialize()

            self.checksum = 0

            for octet in ser2.tobytes():
                self.checksum ^= octet

        self.checksum = self.checksum % 0xFF

        ser = bitstring.pack('uint:8, uint:8, uint:8', self.down_id, self.type.value, self.checksum)
        serialized_msg = ser + ser2
        pad = bitstring.pack('pad:{}'.format((DOWN_MESSAGE_SIZE - len(serialized_msg.tobytes())) * 8))
        return serialized_msg + pad


# ====== End down message declaration
