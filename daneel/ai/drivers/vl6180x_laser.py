# Based on https://github.com/adafruit/Adafruit_VL6180X and https://github.com/luciengaitskell/python-adafruit-vl6180x

from smbus2 import SMBus, SMBusWrapper

VL6180X_DEFAULT_I2C_ADDR = 0x29  # The default I2C address

# Registers
VL6180X_REG_IDENTIFICATION_MODEL_ID = 0x000  # Device model identification number
VL6180X_REG_SYSTEM_INTERRUPT_CONFIG = 0x014  # Interrupt configuration
VL6180X_REG_SYSTEM_INTERRUPT_CLEAR = 0x015   # Interrupt clear bits
VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET = 0x016  # Fresh out of reset bit
VL6180X_REG_SYSRANGE_START = 0x018  # Trigger ranging
VL6180X_REG_SYSALS_START = 0x038    # Trigger lux reading
VL6180X_REG_SYSALS_ANALOGUE_GAIN = 0x03F  # Lux reading gain
VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI = 0x040  # Integration perido for ALs mode, high byte
VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO = 0x041  # Integration period for ALS mode, low byte
VL6180X_REG_RESULT_ALS_VAL = 0x050  # Light reading value
VL6180X_REG_RESULT_RANGE_VAL = 0x062  # Ranging reading value
VL6180X_REG_RESULT_RANGE_STATUS = 0x04d  # Specific error codes
VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO = 0x04f  # Interrupt status

# Gains
VL6180X_ALS_GAIN_1 = 0x06     # 1x gain
VL6180X_ALS_GAIN_1_25 = 0x05  # 1.25x gain
VL6180X_ALS_GAIN_1_67 = 0x04  # 1.67x gain
VL6180X_ALS_GAIN_2_5 = 0x03   # 2.5x gain
VL6180X_ALS_GAIN_5 = 0x02     # 5x gain
VL6180X_ALS_GAIN_10 = 0x01    # 10x gain
VL6180X_ALS_GAIN_20 = 0x00    # 20x gain
VL6180X_ALS_GAIN_40 = 0x07    # 40x gain

# Error codes
VL6180X_ERROR_NONE = 0  # Success!
VL6180X_ERROR_SYSERR_1 = 1  # System error
VL6180X_ERROR_SYSERR_5 = 5  # System error
VL6180X_ERROR_ECEFAIL = 6   # Early convergence estimate fail
VL6180X_ERROR_NOCONVERGE = 7  # No target detected
VL6180X_ERROR_RANGEIGNORE = 8  # Ignore threshold check failed
VL6180X_ERROR_SNR = 11  # Ambient conditions too high
VL6180X_ERROR_RAWUFLOW = 12  # Raw range algo underflow
VL6180X_ERROR_RAWOFLOW = 13  # Raw range algo overflow
VL6180X_ERROR_RANGEUFLOW = 14  # Range algo underflow
VL6180X_ERROR_RANGEOFLOW = 15  # Range algo overflow

class VL6180x:
    def __init__(self, address):
        self.address = address
        self.bus = SMBus(1)
        if self.bus.read_byte_data(self.address, VL6180X_REG_IDENTIFICATION_MODEL_ID) != 0xB4:
            raise BaseException("VL6180X : Model ID does not match.")
        self._load_settings()
        self.bus.write_byte_data(self.address, VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00)


    @property
    def range(self):
        # wait for device to be ready for range measurement
        while not (self.bus.read_byte_data(self.address, VL6180X_REG_RESULT_RANGE_STATUS) & 0x01):
            pass
        # Start a range measurement
        self.bus.write_byte_data(self.address, VL6180X_REG_SYSRANGE_START, 0x01)

        # Poll until bit 2 is set
        while not (self.bus.read_byte_data(self.address, VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04):
            pass

        # Read range in mm
        range = self.bus.read_byte_data(self.address, VL6180X_REG_RESULT_RANGE_VAL)

        # Clear interrupt
        self.bus.write_byte_data(self.address, VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07)

        return range


    @property
    def lux(self, gain):
        reg = self.bus.read_byte_data(self.address, VL6180X_REG_SYSTEM_INTERRUPT_CONFIG)
        reg &= (~0x38)
        reg |= (0x4 << 3)  # IRQ on ALS ready
        self.bus.write_byte_data(self.address, VL6180X_REG_SYSTEM_INTERRUPT_CONFIG, reg)

        # 100ms integration period
        self.bus.write_byte_data(self.address, VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI, 0)
        self.bus.write_byte_data(self.address, VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO, 100)

        # analog gain
        if gain > VL6180X_ALS_GAIN_40:
            gain = VL6180X_ALS_GAIN_40

        # start ALS
        self.bus.write_byte_data(self.address, VL6180X_REG_SYSALS_START, 0x1)

        # Poll until "New Sample Ready threshold event" is set
        while 4 != ((self.bus.read_byte_data(self.address, VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) >> 3) & 0x7):
            pass

        # Read lux
        lux = self.bus.read_word_data(self.address, VL6180X_REG_RESULT_ALS_VAL)

        # Clear interrupt
        self.bus.write_byte_data(self.address, VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07)

        lux *= 0.32  # calibrated count/lux
        if gain == VL6180X_ALS_GAIN_1:
            pass
        elif gain == VL6180X_ALS_GAIN_1_25:
            lux /= 1.25
        elif gain == VL6180X_ALS_GAIN_1_67:
            lux /= 1.67
        elif gain == VL6180X_ALS_GAIN_2_5:
            lux /= 2.5
        elif gain == VL6180X_ALS_GAIN_5:
            lux /= 5
        elif gain == VL6180X_ALS_GAIN_10:
            lux /= 10
        elif gain == VL6180X_ALS_GAIN_20:
            lux /= 20
        elif gain == VL6180X_ALS_GAIN_40:
            lux /= 40

        lux *= 100
        lux /= 100 # integration time in ms

        return lux


    @property
    def range_status(self):
        return self.bus.read_byte_data(self.address, VL6180X_REG_RESULT_RANGE_STATUS) >> 4

    def change_address(self, new_address):
        self.bus.write_byte_data(self.address, 0x212, new_address & 0x7F)
        self.address = new_address

    def _load_settings(self):
        self.bus.write_byte_data(self.address, 0x0207, 0x01)
        self.bus.write_byte_data(self.address, 0x0208, 0x01)
        self.bus.write_byte_data(self.address, 0x0096, 0x00)
        self.bus.write_byte_data(self.address, 0x0097, 0xfd)
        self.bus.write_byte_data(self.address, 0x00e3, 0x00)
        self.bus.write_byte_data(self.address, 0x00e4, 0x04)
        self.bus.write_byte_data(self.address, 0x00e5, 0x02)
        self.bus.write_byte_data(self.address, 0x00e6, 0x01)
        self.bus.write_byte_data(self.address, 0x00e7, 0x03)
        self.bus.write_byte_data(self.address, 0x00f5, 0x02)
        self.bus.write_byte_data(self.address, 0x00d9, 0x05)
        self.bus.write_byte_data(self.address, 0x00db, 0xce)
        self.bus.write_byte_data(self.address, 0x00dc, 0x03)
        self.bus.write_byte_data(self.address, 0x00dd, 0xf8)
        self.bus.write_byte_data(self.address, 0x009f, 0x00)
        self.bus.write_byte_data(self.address, 0x00a3, 0x3c)
        self.bus.write_byte_data(self.address, 0x00b7, 0x00)
        self.bus.write_byte_data(self.address, 0x00bb, 0x3c)
        self.bus.write_byte_data(self.address, 0x00b2, 0x09)
        self.bus.write_byte_data(self.address, 0x00ca, 0x09)
        self.bus.write_byte_data(self.address, 0x0198, 0x01)
        self.bus.write_byte_data(self.address, 0x01b0, 0x17)
        self.bus.write_byte_data(self.address, 0x01ad, 0x00)
        self.bus.write_byte_data(self.address, 0x00ff, 0x05)
        self.bus.write_byte_data(self.address, 0x0100, 0x05)
        self.bus.write_byte_data(self.address, 0x0199, 0x05)
        self.bus.write_byte_data(self.address, 0x01a6, 0x1b)
        self.bus.write_byte_data(self.address, 0x01ac, 0x3e)
        self.bus.write_byte_data(self.address, 0x01a7, 0x1f)
        self.bus.write_byte_data(self.address, 0x0030, 0x00)

        # Recommended : Public registers - See data sheet for more detail
        self.bus.write_byte_data(self.address, 0x0011, 0x10)       # Enables polling for 'New Sample ready'
                                                                   # when measurement completes
        self.bus.write_byte_data(self.address, 0x010a, 0x30)       # Set the averaging sample period
                                                                   # (compromise between lower noise and
                                                                   # increased execution time)
        self.bus.write_byte_data(self.address, 0x003f, 0x46)       # Sets the light and dark gain (upper
                                                                   # nibble). Dark gain should not be
                                                                   # changed.
        self.bus.write_byte_data(self.address, 0x0031, 0xFF)       # sets the # of range measurements after
                                                                   # which auto calibration of system is
                                                                   # performed
        self.bus.write_byte_data(self.address, 0x0040, 0x63)       # Set ALS integration time to 100ms
        self.bus.write_byte_data(self.address, 0x002e, 0x01)       # perform a single temperature calibration
                                                                   # of the ranging sensor

        # Optional: Public registers - See data sheet for more detail
        self.bus.write_byte_data(self.address, 0x001b, 0x09)       # Set default ranging inter-measurement
                                                                   # period to 100ms
        self.bus.write_byte_data(self.address, 0x003e, 0x31)       # Set default ALS inter-measurement period
                                                                   # to 500ms
        self.bus.write_byte_data(self.address, 0x0014, 0x24)       # Configures interrupt on 'New Sample
                                                                   # Ready threshold event'