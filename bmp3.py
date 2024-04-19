import logging

from klippy.extras import bus

REPORT_TIME = 0.1
BMP388_CHIP_ADDR = 0x77

BMP388_REGS = {
    "CHIP_ID": 0x00,
    "CMD": 0x7E,
    "STATUS": 0x03,
    "PWR_CTRL": 0x1B,
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
        dig["T1"] = get_unsigned_short(calibration_data_1[0:2])
        dig["T2"] = get_unsigned_short(calibration_data_1[2:4])
        dig["T3"] = get_signed_byte(calibration_data_1[4])

        dig["P1"] = get_signed_short(calibration_data_1[5:7])
        dig["P2"] = get_signed_short(calibration_data_1[7:9])
        dig["P3"] = get_signed_byte(calibration_data_1[9])
        dig["P4"] = get_signed_byte(calibration_data_1[10])
        dig["P5"] = get_unsigned_short(calibration_data_1[11:13])
        dig["P6"] = get_unsigned_short(calibration_data_1[13:15])
        dig["P7"] = get_signed_byte(calibration_data_1[15])
        dig["P8"] = get_signed_byte(calibration_data_1[16])
        dig["P9"] = get_signed_short(calibration_data_1[17:19])
        dig["P10"] = get_signed_byte(calibration_data_1[19])
        dig["P11"] = get_signed_byte(calibration_data_1[20])
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
        cal_1 = self.read_register("CAL_1", 21)
        self.dig = self._load_calibration_data(cal_1)
        self.max_sample_time = 0.5
        self.sample_timer = self.reactor.register_timer(self._sample)

    def _sample(self, eventtime):
        self.temp = self._sample_temperature() / 100.0
        self.pressure = self._sample_pressure() / 100.0
        if self.temp < self.min_temp or self.temp > self.max_temp:
            self.printer.invoke_shutdown(
                "BMP388 temperature %0.1f outside range of %0.1f:%.01f"
                % (self.temp, self.min_temp, self.max_temp)
            )
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
        partial_data1 = adc_T - (256 * (self.dig["T1"]))
        partial_data2 = self.dig["T2"] * partial_data1
        partial_data3 = partial_data1 * partial_data1
        partial_data4 = (partial_data3) * (self.dig["T3"])
        partial_data5 = ((partial_data2) * 262144) + partial_data4
        partial_data6 = (partial_data5) / 4294967296
        self.t_fine = partial_data6
        comp_temp = (partial_data6 * 25) / 16384
        return comp_temp

    def _sample_pressure(self):
        xlsb = self.read_register("PRESS_XLSB", 1)
        lsb = self.read_register("PRESS_LSB", 1)
        msb = self.read_register("PRESS_MSB", 1)
        adc_P = (msb[0] << 16) + (lsb[0] << 8) + (xlsb[0])
        return self._compensate_pressure(adc_P)

    def _compensate_pressure(self, adc_P):
        partial_data1 = self.t_fine * self.t_fine
        partial_data2 = partial_data1 / 64
        partial_data3 = (partial_data2 * self.t_fine) / 256
        partial_data4 = (self.dig["P8"] * partial_data3) / 32
        partial_data5 = (self.dig["P7"] * partial_data1) * 16
        partial_data6 = (self.dig["P6"] * self.t_fine) * 4194304
        offset = (
            ((self.dig["P5"]) * 140737488355328)
            + partial_data4
            + partial_data5
            + partial_data6
        )

        partial_data2 = ((self.dig["P4"]) * partial_data3) / 32
        partial_data4 = (self.dig["P3"] * partial_data1) * 4
        partial_data5 = ((self.dig["P2"]) - 16384) * (self.t_fine) * 2097152
        sensitivity = (
            (((self.dig["P1"]) - 16384) * 70368744177664)
            + partial_data2
            + partial_data4
            + partial_data5
        )

        partial_data1 = (sensitivity / 16777216) * adc_P
        partial_data2 = (self.dig["P10"]) * (self.t_fine)
        partial_data3 = partial_data2 + (65536 * (self.dig["P9"]))
        partial_data4 = (partial_data3 * adc_P) / 8192
        partial_data5 = (partial_data4 * adc_P) / 512
        partial_data6 = adc_P * adc_P
        partial_data2 = ((self.dig["P11"]) * (partial_data6)) / 65536
        partial_data3 = (partial_data2 * adc_P) / 128
        partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3
        comp_press = (partial_data4 * 25) / 1099511627776
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
