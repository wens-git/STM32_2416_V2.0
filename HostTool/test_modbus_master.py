"""Modbus RTU 主站与 2416_F407 高层协议封装的单元测试。

测试使用内存串口替身，不依赖真实 COM 口或控制板，主要验证 CRC、响应帧校验、
寄存器地址计算、轴命令打包、配置读写和定位点名称编码。该文件可作为归档后的
协议行为回归基线。
"""

import struct
import unittest

from modbus_master import (
    AX_SPEED,
    AX_SPEED_HI,
    CMD_JOG_POS,
    CMD_MOVE_ABS,
    CMD_RESUME,
    NUM_AXES,
    POINT_NAME_UNITS,
    POINT_SPEED_DEFAULT,
    POINT_SPEED_MAX,
    POINT_SPEED_MIN,
    REG_AXIS_BASE,
    REG_AXIS_STRIDE,
    REG_CFG_EXT_BASE,
    REG_CFG_EXT_STRIDE,
    REG_POINT_NAME_BASE,
    REG_POINT_NAME_STRIDE,
    REG_POINT_SPEED_BASE,
    REG_POINT_SPEED_STRIDE,
    REG_SOFT_LIMIT_BASE,
    REG_SOFT_LIMIT_STRIDE,
    SL_ENABLE,
    SL_NEG_LO,
    SL_POS_LO,
    SL_SAVE,
    SOFT_LIMIT_CMD_SAVE,
    REG_SAVE_POINTS,
    REG_HOME_FAST_HI,
    REG_HOME_BACKOFF_HZ,
    REG_SYS_BASE,
    FLASH_SAVE_TIMEOUT_S,
    ModbusFrameError,
    ModbusMaster,
    modbus_crc16,
)


def with_crc(frame: bytes) -> bytes:
    return frame + struct.pack("<H", modbus_crc16(frame))


class FakeSerial:
    """实现 ModbusMaster 所需最小接口的内存串口替身。"""
    def __init__(self, response_factory):
        self.is_open = True
        self.timeout = 0.2
        self._response_factory = response_factory
        self._rx = bytearray()

    def reset_input_buffer(self):
        self._rx.clear()

    def reset_output_buffer(self):
        pass

    def write(self, request):
        self._rx.extend(self._response_factory(bytes(request)))
        return len(request)

    def flush(self):
        pass

    def read(self, size):
        data = bytes(self._rx[:size])
        del self._rx[:size]
        return data


class RecordingMaster(ModbusMaster):
    """记录高层 API 最终发出的寄存器事务，便于断言地址和数据。"""
    def __init__(self):
        super().__init__()
        self.calls = []
        self.read_responses = {}
        self.write_options = {}

    def read_holding(self, start, count):
        self.calls.append(("read_holding", start, count))
        response = self.read_responses.get((start, count))
        if response is None:
            return [0] * count
        return list(response)

    def write_single(self, addr, value, **kwargs):
        self.calls.append(("write_single", addr, value))
        self.write_options[(addr, value)] = kwargs

    def write_multiple(self, start, values):
        self.calls.append(("write_multiple", start, list(values)))


class ModbusMasterProtocolTests(unittest.TestCase):
    def test_crc_known_vector(self):
        request = bytes.fromhex("01 03 00 00 00 0A")
        self.assertEqual(modbus_crc16(request), 0xCDC5)

    def test_rejects_mismatched_response_function(self):
        def response(_request):
            return with_crc(bytes.fromhex("01 04 02 00 01"))

        master = ModbusMaster()
        master.set_serial(FakeSerial(response))

        with self.assertRaisesRegex(ModbusFrameError, "功能码不匹配"):
            master._transact(0x03, bytes.fromhex("00 00 00 01"), retries=0)

    def test_rejects_wrong_fc06_echo(self):
        def response(request):
            body = bytearray(request[:-2])
            body[-1] ^= 0x01
            return with_crc(bytes(body))

        master = ModbusMaster()
        master.set_serial(FakeSerial(response))

        with self.assertRaisesRegex(ModbusFrameError, "FC06.*回显"):
            master.write_single(207, 2)

    def test_i32_register_pair_round_trip(self):
        values = (0, 1, -1, 0x7FFFFFFF, -0x80000000, 123456789, -123456789)
        for value in values:
            with self.subTest(value=value):
                lo, hi = ModbusMaster._i32_to_u16_pair(value)
                self.assertEqual(ModbusMaster._u16_pair_to_i32(lo, hi), value)

    def test_zero_axis_status_does_not_access_transport(self):
        master = RecordingMaster()
        self.assertEqual(master.read_axis_status_range(0), [])
        self.assertEqual(master.calls, [])

    def test_read_io_combines_all_24_inputs(self):
        master = RecordingMaster()
        master.read_responses[(300, 3)] = [0x0001, 0x0081, 0x8000]

        di, do = master.read_io()

        self.assertTrue(di[0])
        self.assertTrue(di[16])
        self.assertTrue(di[23])
        self.assertEqual(sum(di), 3)
        self.assertTrue(do[15])

    def test_save_points_uses_dedicated_register_and_unlock_value(self):
        master = RecordingMaster()
        master.save_points_to_flash()
        self.assertEqual(master.calls, [
            ("write_single", REG_SAVE_POINTS, 0xA55A),
        ])
        self.assertEqual(master.write_options[(REG_SAVE_POINTS, 0xA55A)], {
            "retries": 0,
            "timeout_s": FLASH_SAVE_TIMEOUT_S,
        })

    def test_move_parameters_and_command_are_separate_frames(self):
        master = RecordingMaster()
        master.axis_move_abs(2, 2500, -123456)
        lo, hi = master._i32_to_u16_pair(-123456)
        base = REG_AXIS_BASE + 2 * REG_AXIS_STRIDE
        self.assertEqual(master.calls, [
            ("write_single", base + AX_SPEED_HI, 0),
            ("write_multiple", base + 5, [2500, lo, hi]),
            ("write_single", base + 8, CMD_MOVE_ABS),
        ])

    def test_jog_100khz_writes_speed_high_then_low_and_command(self):
        master = RecordingMaster()
        master.axis_jog(1, 100000, True)
        base = REG_AXIS_BASE + REG_AXIS_STRIDE
        self.assertEqual(master.calls, [
            ("write_single", base + AX_SPEED_HI, 1),
            ("write_single", base + AX_SPEED, 34464),
            ("write_single", base + 8, CMD_JOG_POS),
        ])

    def test_resume_remaining_writes_only_resume_command(self):
        master = RecordingMaster()
        master.axis_resume_remaining(3)
        base = REG_AXIS_BASE + 3 * REG_AXIS_STRIDE
        self.assertEqual(master.calls, [
            ("write_single", base + 8, CMD_RESUME),
        ])

    def test_axis_status_combines_32bit_speed(self):
        master = RecordingMaster()
        regs = [0] * REG_AXIS_STRIDE
        regs[AX_SPEED] = 34464
        regs[AX_SPEED_HI] = 1
        master.read_responses[(REG_AXIS_BASE, REG_AXIS_STRIDE)] = regs

        status = master.read_axis_status(0)

        self.assertEqual(status.speed, 100000)

    def test_invalid_motion_axis_sends_nothing(self):
        methods = (
            lambda m: m.axis_jog(NUM_AXES, 1000, True),
            lambda m: m.axis_move_rel(NUM_AXES, 1000, 10),
            lambda m: m.axis_move_abs(NUM_AXES, 1000, 10),
            lambda m: m.axis_set_position(NUM_AXES, 0),
        )
        for invoke in methods:
            master = RecordingMaster()
            with self.assertRaises(ValueError):
                invoke(master)
            self.assertEqual(master.calls, [])

    def test_point_name_utf16_round_trip_preserves_complete_characters(self):
        name = "工位A😀结束超长字符"
        encoded = ModbusMaster._encode_point_name(name)
        self.assertEqual(len(encoded), POINT_NAME_UNITS)
        decoded = ModbusMaster._decode_point_name(encoded)
        self.assertNotIn("\ufffd", decoded)
        self.assertTrue(name.startswith(decoded))

    def test_point_name_register_addresses_and_lengths(self):
        master = RecordingMaster()
        axis = 3
        base = REG_POINT_NAME_BASE + axis * REG_POINT_NAME_STRIDE
        regs = []
        for name in ("点一", "P2", "", "😀", "五", "六", "七", "八"):
            regs.extend(master._encode_point_name(name))
        master.read_responses[(base, REG_POINT_NAME_STRIDE)] = regs

        names = master.read_position_point_names(axis)
        master.write_position_point_names(axis, names)

        self.assertEqual(names[:4], ["点一", "P2", "", "😀"])
        self.assertEqual(master.calls[0],
                         ("read_holding", base, REG_POINT_NAME_STRIDE))
        self.assertEqual(master.calls[1][0:2], ("write_multiple", base))
        self.assertEqual(len(master.calls[1][2]), REG_POINT_NAME_STRIDE)

    # ---- 点位速度 (1600 起, 每轴 16 寄存器) ----

    def test_point_speed_register_layout_matches_firmware(self):
        self.assertEqual(REG_POINT_SPEED_BASE, 1600)
        self.assertEqual(REG_POINT_SPEED_STRIDE, 16)
        # 软限位区 1500..1563 与速度区不得重叠
        self.assertGreater(REG_POINT_SPEED_BASE, 1500 + 8 * NUM_AXES - 1)

    def test_read_position_point_speeds_uses_16_registers(self):
        master = RecordingMaster()
        axis = 5
        base = REG_POINT_SPEED_BASE + axis * REG_POINT_SPEED_STRIDE
        speeds = [100, 1250, 8000, 100000, 1, 0, 65535, 65536]
        regs = []
        for spd in speeds:
            regs.append(spd & 0xFFFF)
            regs.append((spd >> 16) & 0xFFFF)
        master.read_responses[(base, 16)] = regs

        result = master.read_position_point_speeds(axis)

        self.assertEqual(result, speeds)
        self.assertEqual(master.calls[0], ("read_holding", base, 16))

    def test_write_position_point_speeds_encodes_lo_hi_pairs(self):
        master = RecordingMaster()
        axis = 2
        base = REG_POINT_SPEED_BASE + axis * REG_POINT_SPEED_STRIDE
        speeds = [8000] * 8

        master.write_position_point_speeds(axis, speeds)

        self.assertEqual(master.calls[0][0:2], ("write_multiple", base))
        vals = master.calls[0][2]
        self.assertEqual(len(vals), 16)
        self.assertEqual(vals[0:2], [8000, 0])

    def test_write_point_speeds_rejects_out_of_range(self):
        for bad in (POINT_SPEED_MIN - 1, POINT_SPEED_MAX + 1, 0, -1):
            master = RecordingMaster()
            with self.subTest(speed=bad):
                with self.assertRaisesRegex(ValueError, "点位速度"):
                    master.write_position_point_speeds(0, [bad] * 8)
                self.assertEqual(master.calls, [])

    def test_write_point_speeds_pads_missing_entries(self):
        master = RecordingMaster()
        master.write_position_point_speeds(0, [900])
        vals = master.calls[0][2]
        self.assertEqual(vals[0:2], [900, 0])
        self.assertEqual(vals[2:4], [POINT_SPEED_DEFAULT, 0])

    def test_read_position_point_data_includes_speeds(self):
        master = RecordingMaster()
        axis = 1
        master.read_responses[(REG_POINT_NAME_BASE + axis * REG_POINT_NAME_STRIDE,
                               REG_POINT_NAME_STRIDE)] = [0] * REG_POINT_NAME_STRIDE

        data = master.read_position_point_data(axis)

        self.assertIn("speeds", data)
        self.assertEqual(len(data["speeds"]), 8)
        self.assertEqual(len(data["positions"]), 8)
        self.assertEqual(len(data["names"]), 8)

    def test_write_position_point_data_can_skip_speeds(self):
        master = RecordingMaster()
        master.write_position_point_data(0, [0] * 8, ["P"] * 8)
        starts = [c[1] for c in master.calls if c[0] == "write_multiple"]
        self.assertNotIn(REG_POINT_SPEED_BASE, starts)

    def test_write_position_point_data_writes_speeds_when_given(self):
        master = RecordingMaster()
        master.write_position_point_data(0, [0] * 8, ["P"] * 8, [2000] * 8)
        starts = [c[1] for c in master.calls if c[0] == "write_multiple"]
        self.assertIn(REG_POINT_SPEED_BASE, starts)

    def test_read_all_config_uses_system_and_home_speed_high_words(self):
        master = RecordingMaster()
        sys_regs = [0] * 18
        sys_regs[1] = 0
        sys_regs[7] = 1
        sys_regs[16] = NUM_AXES
        master.read_responses[(REG_SYS_BASE, 18)] = sys_regs
        master.read_responses[(REG_HOME_FAST_HI, 2)] = [1, 1]
        master.read_responses[(REG_HOME_BACKOFF_HZ, 1)] = [300]
        for axis in range(NUM_AXES):
            base = REG_CFG_EXT_BASE + axis * REG_CFG_EXT_STRIDE
            master.read_responses[(base, REG_CFG_EXT_STRIDE)] = [
                0, axis, 0xFFFF, axis * 2, 0xFFFF, 0xFFFF, 0xFFFF, 0, 0xFFFF
            ]

        config = master.read_all_config()

        self.assertEqual(config["system"]["axis_count"], 0)
        self.assertEqual(config["system"]["axis_capacity"], NUM_AXES)
        self.assertEqual(config["system"]["home_fast"], 65536)
        self.assertEqual(config["system"]["home_slow"], 65536)
        self.assertEqual(config["system"]["home_backoff_hz"], 300)
        self.assertEqual(len(config["axes"]), NUM_AXES)
        self.assertEqual(master.calls[0], ("read_holding", REG_SYS_BASE, 18))
        self.assertEqual(master.calls[1], ("read_holding", REG_HOME_FAST_HI, 2))
        self.assertEqual(master.calls[2], ("read_holding", REG_HOME_BACKOFF_HZ, 1))
        self.assertEqual(
            master.calls[3:],
            [("read_holding",
              REG_CFG_EXT_BASE + axis * REG_CFG_EXT_STRIDE,
              REG_CFG_EXT_STRIDE)
             for axis in range(NUM_AXES)])

    # ---- 软限位 (1500 起, 每轴 8 寄存器) ----

    def test_soft_limit_register_layout_matches_firmware(self):
        self.assertEqual(REG_SOFT_LIMIT_BASE, 1500)
        self.assertEqual(REG_SOFT_LIMIT_STRIDE, 8)
        self.assertEqual((SL_ENABLE, SL_NEG_LO, SL_POS_LO, SL_SAVE),
                         (0, 1, 3, 7))
        self.assertEqual(SOFT_LIMIT_CMD_SAVE, 0xA55A)
        # 1500..1563 软限位区不得与 320 区 / 1600 区重叠
        self.assertLess(REG_SOFT_LIMIT_BASE, REG_POINT_SPEED_BASE)
        self.assertLess(REG_SOFT_LIMIT_BASE + REG_SOFT_LIMIT_STRIDE * NUM_AXES,
                        REG_POINT_SPEED_BASE)

    def test_read_soft_limit_addresses_and_decoding(self):
        master = RecordingMaster()
        axis = 6
        base = REG_SOFT_LIMIT_BASE + axis * REG_SOFT_LIMIT_STRIDE
        min_lo, min_hi = master._i32_to_u16_pair(-123456)
        max_lo, max_hi = master._i32_to_u16_pair(654321)
        master.read_responses[(base, REG_SOFT_LIMIT_STRIDE)] = [
            1, min_lo, min_hi, max_lo, max_hi, 1, 0, 0]

        data = master.read_soft_limit(axis)

        self.assertEqual(master.calls[0],
                         ("read_holding", base, REG_SOFT_LIMIT_STRIDE))
        self.assertTrue(data["enable"])
        self.assertTrue(data["valid"])
        self.assertEqual(data["min"], -123456)
        self.assertEqual(data["max"], 654321)

    def test_read_soft_limit_zero_registers_means_disabled(self):
        master = RecordingMaster()
        data = master.read_soft_limit(0)
        self.assertFalse(data["enable"])
        self.assertFalse(data["valid"])
        self.assertEqual((data["min"], data["max"]), (0, 0))

    def test_write_soft_limit_uses_safe_sequence(self):
        master = RecordingMaster()
        axis = 2
        base = REG_SOFT_LIMIT_BASE + axis * REG_SOFT_LIMIT_STRIDE
        min_lo, min_hi = master._i32_to_u16_pair(-5000)
        max_lo, max_hi = master._i32_to_u16_pair(9000)

        master.write_soft_limit(axis, True, -5000, 9000, verify=False)

        self.assertEqual(master.calls, [
            ("write_single", base + SL_ENABLE, 0),
            ("write_multiple", base + SL_NEG_LO,
             [min_lo, min_hi, max_lo, max_hi]),
            ("write_single", base + SL_ENABLE, 1),
            ("write_single", base + SL_SAVE, SOFT_LIMIT_CMD_SAVE),
        ])

    def test_write_soft_limit_without_save_skips_flash_trigger(self):
        master = RecordingMaster()
        base = REG_SOFT_LIMIT_BASE
        master.write_soft_limit(0, False, -1000, 1000, save=False, verify=False)
        self.assertEqual(master.calls[-1], ("write_single", base + SL_ENABLE, 0))
        self.assertNotIn(("write_single", base + SL_SAVE, SOFT_LIMIT_CMD_SAVE),
                         master.calls)

    def test_write_soft_limit_rejects_inverted_range_when_enabled(self):
        master = RecordingMaster()
        with self.assertRaisesRegex(ValueError, "负向限位 < 正向限位"):
            master.write_soft_limit(0, True, 100, 100)
        with self.assertRaisesRegex(ValueError, "负向限位 < 正向限位"):
            master.write_soft_limit(0, True, 500, -500)
        self.assertEqual(master.calls, [])

    def test_write_soft_limit_allows_equal_bounds_when_disabled(self):
        master = RecordingMaster()
        master.write_soft_limit(0, False, 10, 10, verify=False)
        self.assertEqual(len(master.calls), 4)
        self.assertEqual(master.calls[3],
                         ("write_single", REG_SOFT_LIMIT_BASE + SL_SAVE,
                          SOFT_LIMIT_CMD_SAVE))

    def test_write_soft_limit_rejects_out_of_int32_range(self):
        master = RecordingMaster()
        with self.assertRaisesRegex(ValueError, "int32"):
            master.write_soft_limit(0, False, -0x80000001, 0, verify=False)
        with self.assertRaisesRegex(ValueError, "int32"):
            master.write_soft_limit(0, False, 0, 0x80000000, verify=False)

    def test_write_soft_limit_raises_on_readback_mismatch(self):
        master = RecordingMaster()
        base = REG_SOFT_LIMIT_BASE
        min_lo, min_hi = master._i32_to_u16_pair(-5000)
        max_lo, max_hi = master._i32_to_u16_pair(9000)
        # 回读返回错误的上限, 模拟固件未接受写入
        master.read_responses[(base, REG_SOFT_LIMIT_STRIDE)] = [
            1, min_lo, min_hi, max_lo, max_hi, 1, 0, 0][:3] + [
            master._i32_to_u16_pair(1)[0], master._i32_to_u16_pair(1)[1], 1, 0, 0]
        with self.assertRaisesRegex(ModbusFrameError, "软限位读回不一致"):
            master.write_soft_limit(0, True, -5000, 9000)

    def test_disable_soft_limit_writes_only_enable_and_save(self):
        master = RecordingMaster()
        base = REG_SOFT_LIMIT_BASE + 3 * REG_SOFT_LIMIT_STRIDE
        master.disable_soft_limit(3)
        self.assertEqual(master.calls, [
            ("write_single", base + SL_ENABLE, 0),
            ("write_single", base + SL_SAVE, SOFT_LIMIT_CMD_SAVE),
        ])



if __name__ == "__main__":
    unittest.main()
