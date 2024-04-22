import logging

from . import bus

REPORT_TIME = 0.1
BMP388_CHIP_ADDR = 0x77

BMP388_REGS = {
    "CHIP_ID": 0x00,
    "CMD": 0x7E,
    "STATUS": 0x03,
    "PWR_CTRL": 0x1B,
    "OSR": 0x1C,
    "ORD": 0x1D,
    "INT_CTRL": 0x19,
    "CAL_1": 0x31,
    "TEMP_MSB": 0x09,
    "TEMP_LSB": 0x08,
    "TEMP_XLSB": 0x07,
    "PRESS_MSB": 0x06,
    "PRESS_LSB": 0x05,
    "PRESS_XLSB": 0x04,
}
BMP388_CHIP_ID = 0x50
BMP388_REG_VAL_STATUS_CMD_READY = 1 << 4
BMP388_REG_VAL_PRESS_EN = 0x01
BMP388_REG_VAL_TEMP_EN = 0x02
BMP388_REG_VAL_PRESS_OS_NO = 0b000
BMP388_REG_VAL_TEMP_OS_NO = 0b000000
BMP388_REG_VAL_ODR_50_HZ = 0x02
BMP388_REG_VAL_DRDY_EN = 0b100000
BMP388_REG_VAL_NORMAL_MODE = 0x30
BMP388_REG_VAL_RESET_CHIP_VALUE = 0xB6


def get_twos_complement(val, bit_size):
    if val & (1 << (bit_size - 1)):
        val -= 1 << bit_size
    return val


def get_unsigned_short(bits):
    return bits[1] << 8 | bits[0]


def get_signed_short(bits):
    val = get_unsigned_short(bits)
    return get_twos_complement(val, 16)


def get_signed_byte(bits):
    return get_twos_complement(bits, 8)


def get_unsigned_short_msb(bits):
    return bits[0] << 8 | bits[1]


def get_signed_short_msb(bits):
    val = get_unsigned_short_msb(bits)
    return get_twos_complement(val, 16)


class BMP388:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=BMP388_CHIP_ADDR, default_speed=100000
        )
        self.mcu = self.i2c.get_mcu()

        self.temp = self.pressure = self.t_fine = 0.0
        self.min_temp = self.max_temp = 0.0
        self.max_sample_time = None
        self.dig = self.sample_timer = None
        self.chip_type = "BMP388"
        self.chip_registers = BMP388_REGS
        self.printer.add_object("bmp388 " + self.name, self)
        if self.printer.get_start_args().get("debugoutput") is not None:
            return
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

    def handle_connect(self):
        self._init()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def _load_calibration_data(self, calibration_data_1):
        dig = {}
        dig["T1"] = get_unsigned_short(calibration_data_1[0:2]) / 0.00390625
        dig["T2"] = get_unsigned_short(calibration_data_1[2:4]) / 1073741824.0
        dig["T3"] = get_signed_byte(calibration_data_1[4]) / 281474976710656.0

        dig["P1"] = (get_signed_short(calibration_data_1[5:7]) - 16384) / 1048576.0
        dig["P2"] = (get_signed_short(calibration_data_1[7:9]) - 16384) / 536870912.0
        dig["P3"] = get_signed_byte(calibration_data_1[9]) / 4294967296.0
        dig["P4"] = get_signed_byte(calibration_data_1[10]) / 137438953472.0
        dig["P5"] = get_unsigned_short(calibration_data_1[11:13]) / 0.125
        dig["P6"] = get_unsigned_short(calibration_data_1[13:15]) / 64.0
        dig["P7"] = get_signed_byte(calibration_data_1[15]) / 256.0
        dig["P8"] = get_signed_byte(calibration_data_1[16]) / 32768.0
        dig["P9"] = get_signed_short(calibration_data_1[17:19]) / 281474976710656.0
        dig["P10"] = get_signed_byte(calibration_data_1[19]) / 281474976710656.0
        dig["P11"] = get_signed_byte(calibration_data_1[20]) / 36893488147419103232.0
        return dig

    def _init(self):
        status = self.read_register("STATUS", 1)[0]
        while not (status & BMP388_REG_VAL_STATUS_CMD_READY):
            self.reactor.pause(self.reactor.monotonic() + 0.01)
            status = self.read_register("STATUS", 1)[0]

        chip_id = self.read_id()
        if chip_id != BMP388_CHIP_ID:
            logging.warn("BMP388: Unknown Chip ID received %#x" % chip_id)

        # Reset chip
        self.write_register("CMD", [BMP388_REG_VAL_RESET_CHIP_VALUE])
        self.reactor.pause(self.reactor.monotonic() + 0.5)

        self.write_register(
            "PWR_CTRL",
            [
                BMP388_REG_VAL_PRESS_EN
                | BMP388_REG_VAL_TEMP_EN
                | BMP388_REG_VAL_NORMAL_MODE
            ],
        )
        self.write_register(
            "OSR", [BMP388_REG_VAL_PRESS_OS_NO | BMP388_REG_VAL_TEMP_OS_NO]
        )
        self.write_register("ORD", [BMP388_REG_VAL_ODR_50_HZ])
        self.write_register("INT_CTRL", [BMP388_REG_VAL_DRDY_EN])
        cal_1 = self.read_register("CAL_1", 21)
        self.dig = self._load_calibration_data(cal_1)
        self.max_sample_time = 0.5
        self.sample_timer = self.reactor.register_timer(self._sample)

    def _sample(self, eventtime):
        status = self.read_register("STATUS", 1)
        if status[0] & 0b100000:
            self.temp = self._sample_temperature()
            if self.temp < self.min_temp or self.temp > self.max_temp:
                self.printer.invoke_shutdown(
                    "BMP388 temperature %0.1f outside range of %0.1f:%.01f"
                    % (self.temp, self.min_temp, self.max_temp)
                )
            # logging.info("BMP388: Temperature: %0.2f" % self.temp)

        if status[0] & 0b010000:
            self.pressure = self._sample_pressure() / 100.0
            # logging.info("BMP388: Pressure: %0.2f" % self.pressure)

        measured_time = self.reactor.monotonic()
        self._callback(self.mcu.estimated_print_time(measured_time), self.temp)
        return measured_time + REPORT_TIME

    def _sample_temperature(self):
        xlsb = self.read_register("TEMP_XLSB", 1)
        lsb = self.read_register("TEMP_LSB", 1)
        msb = self.read_register("TEMP_MSB", 1)
        adc_T = (msb[0] << 16) + (lsb[0] << 8) + (xlsb[0])
        return self._compensate_temperature(adc_T)

    def _compensate_temperature(self, adc_T):
        partial_data1 = adc_T - self.dig["T1"]
        partial_data2 = self.dig["T2"] * partial_data1

        self.t_fine = partial_data2 + (partial_data1 * partial_data1) * self.dig["T3"]

        if self.t_fine < -40.0:
            self.t_fine = -40.0

        if self.t_fine > 85.0:
            self.t_fine = 85.0

        return self.t_fine

    def _sample_pressure(self):
        xlsb = self.read_register("PRESS_XLSB", 1)
        lsb = self.read_register("PRESS_LSB", 1)
        msb = self.read_register("PRESS_MSB", 1)
        adc_P = (msb[0] << 16) + (lsb[0] << 8) + (xlsb[0])
        return self._compensate_pressure(adc_P)

    def _compensate_pressure(self, adc_P):
        partial_data1 = self.dig["P6"] * self.t_fine
        partial_data2 = self.dig["P7"] * (self.t_fine * self.t_fine)
        partial_data3 = self.dig["P8"] * (self.t_fine * self.t_fine * self.t_fine)
        partial_out1 = self.dig["P5"] + partial_data1 + partial_data2 + partial_data3

        partial_data1 = self.dig["P2"] * self.t_fine
        partial_data2 = self.dig["P3"] * (self.t_fine * self.t_fine)
        partial_data3 = self.dig["P4"] * (self.t_fine * self.t_fine * self.t_fine)
        partial_out2 = adc_P * (
            self.dig["P1"] + partial_data1 + partial_data2 + partial_data3
        )

        partial_data1 = adc_P * adc_P
        partial_data2 = self.dig["P9"] + (self.dig["P10"] * self.t_fine)
        partial_data3 = partial_data1 * partial_data2
        partial_data4 = partial_data3 + adc_P * adc_P * adc_P * self.dig["P11"]

        comp_press = partial_out1 + partial_out2 + partial_data4

        if comp_press < 30000:
            comp_press = 30000

        if comp_press > 125000:
            comp_press = 125000

        return comp_press

    def read_id(self):
        regs = [self.chip_registers["CHIP_ID"]]
        params = self.i2c.i2c_read(regs, 1)
        return bytearray(params["response"])[0]

    def read_register(self, reg_name, read_len):
        # read a single register
        regs = [self.chip_registers[reg_name]]
        params = self.i2c.i2c_read(regs, read_len)
        return bytearray(params["response"])

    def write_register(self, reg_name, data):
        if type(data) is not list:
            data = [data]
        reg = self.chip_registers[reg_name]
        data.insert(0, reg)
        self.i2c.i2c_write(data)

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, cb):
        self._callback = cb

    def get_report_time_delta(self):
        return REPORT_TIME

    def get_status(self, eventtime):
        data = {"temperature": round(self.temp, 2), "pressure": self.pressure}
        return data


def load_config(config):
    # Register sensor
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("BMP388", BMP388)
