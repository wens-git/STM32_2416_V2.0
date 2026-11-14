#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
modbus_master.py
=================
Modbus RTU 主站底层实现 + 2416_F407 协议封装。

设计原则
--------
1. 同步阻塞接口 (调用方应在通信线程中使用，避免阻塞 UI)。
2. 严格的 CRC 校验与超时控制；任何异常以 ModbusError 子类抛出。
3. 内置串口锁，允许多个高层线程同时调用，由内部串行化保证 RTU 帧不被打断。
4. 所有寄存器 / 命令地址通过常量定义集中维护，便于固件协议变更时同步。

寄存器地址定义参考 Firmware/Core/Src/app/debug/motor_debug_service.c 文件头注释。
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import serial


# ============================================================================
#  协议常量 — 与 STM32 固件 motor_debug_service.c 保持一致
# ============================================================================

# ------ 波特率 / 通信参数 ------
DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT_S = 0.20   # 单次事务超时
FLASH_SAVE_TIMEOUT_S = 10.0  # 128KB Sector 7 擦除和回读校验专用超时
DEFAULT_INTERFRAME_S = 0.002  # RTU 帧间最小间隔, 115200 下留足余量
DEFAULT_RETRIES = 1        # 通信异常时的重试次数

# ------ 轴数 / I/O 数 ------
NUM_AXES = 8
NUM_DI = 24
NUM_DO = 16
NUM_PWM = 4
SUBDIVISION_DEFAULT = 5000

# ------ 寄存器基址 ------
REG_AXIS_BASE = 320   # 每轴 10 寄存器: Axis0@320, Axis1@330
REG_AXIS_STRIDE = 10
REG_CFG_BASE = 100    # 每轴 8 寄存器: 旧版只读轴绑定配置
REG_CFG_STRIDE = 8
REG_CFG_EXT_BASE = 1400
REG_CFG_EXT_STRIDE = 9
REG_POINTS_BASE = 400 # 轴定位点读写: 每轴 16 寄存器 (8点 × 2 lo/hi)
REG_POINTS_STRIDE = 16
REG_POINT_NAME_BASE = 600
REG_POINT_NAME_STRIDE = 96
REG_POINT_SPEED_BASE = 1600  # 点位速度: 每轴 16 寄存器 (8点 × 2 lo/hi), 单位 Hz
REG_POINT_SPEED_STRIDE = 16
REG_SOFT_LIMIT_BASE = 1500   # 软限位: 每轴 8 寄存器
REG_SOFT_LIMIT_STRIDE = 8
SL_ENABLE = 0        # RW: 0=关闭, 非 0=启用
SL_NEG_LO = 1        # RW: 负向软限位低 16 位 (int32)
SL_NEG_HI = 2        # RW: 负向软限位高 16 位
SL_POS_LO = 3        # RW: 正向软限位低 16 位 (int32)
SL_POS_HI = 4        # RW: 正向软限位高 16 位
SL_VALID = 5         # R : 1=已启用且 min<max
SL_RESERVED = 6      # R : 保留
SL_SAVE = 7          # W : 写 0xA55A 立即保存到 Flash
SOFT_LIMIT_CMD_SAVE = 0xA55A
POINT_NAME_UNITS = 12
POINT_SPEED_MIN = 100
POINT_SPEED_MAX = 100000
POINT_SPEED_DEFAULT = 1250
PIN_NONE = 0xFFFF
PULSE_MODE_PWM = 0
PULSE_MODE_SOFT_PWM = 1
REG_SYS_BASE = 200
REG_IO_BASE = 300

# ------ 每轴寄存器偏移 ------
AX_POS_LO = 0
AX_POS_HI = 1
AX_STATE = 2
AX_HOMED = 3
AX_RESERVED = 4   # 原报警码, 固件已删除轴报警体系, 恒读 0
AX_SPEED = 5
AX_TARGET_LO = 6
AX_TARGET_HI = 7
AX_CMD = 8
AX_SPEED_HI = 9

# ------ 命令码 (写 AX_CMD) ------
CMD_NOP = 0
CMD_STOP = 1
CMD_JOG_POS = 2
CMD_JOG_NEG = 3
CMD_MOVE_ABS = 4
CMD_MOVE_REL = 5
CMD_HOME = 6
CMD_SET_POS = 7
# 8 保留 (原清警命令, 固件已删除轴报警体系, 固件返回非法数据值)
CMD_SAVE_POINTS = 9  # 保存定位点到 Flash
CMD_RESUME = 10

# ------ 系统寄存器 ------
REG_VERSION = 200
REG_AXIS_COUNT = 201
REG_HOME_FAST = 202
REG_HOME_SLOW = 203
REG_HOME_BACK = 204      # 已废弃 (旧: 回退步数), 固件仅保留写入兼容
REG_HOME_MODE = 205
REG_APPLY_CFG = REG_HOME_MODE
REG_E_STOP = 206
REG_SLAVE_ADDR = 207
REG_SUBDIV = 208
REG_JOG_STEP = 209
REG_COMM_WDT_MS = 210
REG_COMM_WDT_ST = 211
REG_COMM_FRAME = 212
REG_COMM_ERR = 213
REG_HEARTBEAT = 214
REG_OP_MODE   = 215
REG_AXIS_CAPACITY = 216
REG_SAVE_POINTS = 217
REG_HOME_FAST_HI = 218
REG_HOME_SLOW_HI = 219
REG_HOME_BACKOFF_HZ = 221  # 回原触发后反向回退速度 (Hz), 固件 v2 起生效

# ------ 运行模式 ------
MODE_NORMAL = 0
MODE_DEBUG  = 1
MODE_NAMES = {MODE_NORMAL: "正常运行", MODE_DEBUG: "调试模式"}

# ------ I/O 寄存器 ------
REG_DI_LO = 300
REG_DI_HI = 301
REG_DO_STATE = 302

# ------ 状态码 ------
# 固件已删除轴报警体系: 不存在报警状态, 寄存器 +4 恒读 0。
STATE_IDLE = 0
STATE_JOG = 1
STATE_MOVING = 2
STATE_HOMING = 3

STATE_NAMES = {
    STATE_IDLE: "空闲",
    STATE_JOG: "JOG点动",
    STATE_MOVING: "定位中",
    STATE_HOMING: "回原中",
}


# ============================================================================
#  异常类
# ============================================================================

class ModbusError(Exception):
    """Modbus 通信基类异常。"""


class ModbusTimeoutError(ModbusError):
    """超时无响应。"""


class ModbusCRCError(ModbusError):
    """CRC 校验失败。"""


class ModbusFrameError(ModbusError):
    """帧格式错误 / 长度不足 / 地址不匹配。"""


class ModbusExceptionResp(ModbusError):
    """从机返回异常码 (功能码|0x80)。"""

    def __init__(self, func: int, exc: int):
        self.func = func
        self.exc = exc
        super().__init__(f"Modbus 异常 func=0x{func:02X} code=0x{exc:02X}")


# ============================================================================
#  CRC16
# ============================================================================

def modbus_crc16(data: bytes) -> int:
    """标准 Modbus CRC-16/IBM (低字节在前发送)。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


# ============================================================================
#  Modbus RTU 主站
# ============================================================================

@dataclass
class AxisStatus:
    """单轴运行时快照 (来自寄存器组 320..329 / 330..339)。"""
    position: int = 0
    state: int = 0
    homed: bool = False
    speed: int = 0
    target: int = 0


@dataclass
class CommDiag:
    """通信诊断快照。"""
    wdt_ms: int = 0
    wdt_triggered: int = 0
    frame_count: int = 0
    err_count: int = 0


class ModbusMaster:
    """Modbus RTU 主站。

    线程安全: 所有方法在内部对串口加锁，可由多线程调用。
    端口由调用方负责 open/close (通过 set_serial)。
    """

    def __init__(self, slave_addr: int = 1, timeout: float = DEFAULT_TIMEOUT_S):
        self._ser: Optional[serial.Serial] = None
        self._slave_addr = slave_addr
        self._lock = threading.RLock()
        self._timeout = timeout
        self._frame_count = 0
        self._err_count = 0

    # ------------------------------------------------------------------
    # 端口 / 地址管理
    # ------------------------------------------------------------------

    def set_serial(self, ser: Optional[serial.Serial]) -> None:
        """绑定/解绑底层 pyserial 实例 (None 表示断开)。"""
        with self._lock:
            self._ser = ser
            if ser is not None:
                ser.timeout = self._timeout

    @property
    def serial(self) -> Optional[serial.Serial]:
        return self._ser

    @property
    def slave_addr(self) -> int:
        return self._slave_addr

    @slave_addr.setter
    def slave_addr(self, addr: int) -> None:
        if not (1 <= addr <= 247):
            raise ValueError("从站地址超范围 (1-247)")
        self._slave_addr = addr

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    @property
    def stats(self) -> tuple:
        """(成功帧数, 错误次数)。"""
        return (self._frame_count, self._err_count)

    def reset_stats(self) -> None:
        self._frame_count = 0
        self._err_count = 0

    # ------------------------------------------------------------------
    # 帧收发底层
    # ------------------------------------------------------------------

    def _build(self, func: int, payload: bytes) -> bytes:
        frame = bytes([self._slave_addr, func]) + payload
        crc = modbus_crc16(frame)
        return frame + struct.pack('<H', crc)

    def _read_exact(self, n: int, timeout_s: Optional[float] = None) -> bytes:
        """阻塞读取 n 字节, 超时抛 ModbusTimeoutError。"""
        ser = self._ser
        if ser is None:
            raise ModbusError("串口未连接")
        timeout = self._timeout if timeout_s is None else timeout_s
        end = time.monotonic() + timeout
        buf = bytearray()
        while len(buf) < n:
            remain = end - time.monotonic()
            if remain <= 0:
                raise ModbusTimeoutError(f"读超时 (期望 {n} 字节, 实收 {len(buf)})")
            ser.timeout = max(0.005, remain)
            chunk = ser.read(n - len(buf))
            if chunk:
                buf.extend(chunk)
        return bytes(buf)

    def _transact(self, func: int, payload: bytes,
                  retries: int = DEFAULT_RETRIES,
                  timeout_s: Optional[float] = None) -> bytes:
        """同步事务: 发送请求, 读取响应, 校验 CRC, 返回响应净荷 (不含地址/功能码/CRC)。"""
        if self._ser is None or not self._ser.is_open:
            raise ModbusError("串口未连接")

        last_exc: Optional[Exception] = None

        for attempt in range(retries + 1):
            try:
                with self._lock:
                    ser = self._ser
                    # 清理残留并保证帧间间隔
                    ser.reset_input_buffer()
                    ser.reset_output_buffer()
                    time.sleep(DEFAULT_INTERFRAME_S)

                    req = self._build(func, payload)
                    ser.write(req)
                    ser.flush()

                    # 先读包头 3 字节: addr + func + (count | exc_code)
                    header = self._read_exact(3, timeout_s)
                    addr_r, func_r, third = header[0], header[1], header[2]

                    if addr_r != self._slave_addr:
                        # 接收方不是我们寻址的从机, 但只接收同地址帧
                        raise ModbusFrameError(
                            f"返回地址不匹配: 期望 {self._slave_addr}, 实际 {addr_r}")

                    # 异常响应: func | 0x80
                    if func_r & 0x80:
                        crc_bytes = self._read_exact(2, timeout_s)
                        body = bytes([addr_r, func_r, third])
                        recv_crc = struct.unpack('<H', crc_bytes)[0]
                        calc_crc = modbus_crc16(body)
                        if recv_crc != calc_crc:
                            raise ModbusCRCError("异常响应 CRC 错误")
                        raise ModbusExceptionResp(func_r & 0x7F, third)

                    if func_r != func:
                        raise ModbusFrameError(
                            f"返回功能码不匹配: 期望 0x{func:02X}, 实际 0x{func_r:02X}")

                    # 正常响应: 计算剩余字节数
                    if func_r in (0x01, 0x02, 0x03, 0x04):
                        # 读类: 第三字节是字节计数, 后面跟 N 字节数据 + 2 字节 CRC
                        byte_count = third
                        rest = self._read_exact(byte_count + 2, timeout_s)
                        body = header + rest[:byte_count]
                        crc_bytes = rest[byte_count:]
                    elif func_r in (0x05, 0x06, 0x0F, 0x10):
                        # 写类: 还有 3 字节 + 2 字节 CRC (固定 8 字节响应)
                        # 已读 3 字节, 还需 5 字节
                        rest = self._read_exact(5, timeout_s)
                        body = header + rest[:3]
                        crc_bytes = rest[3:]
                    else:
                        raise ModbusFrameError(f"未知功能码 0x{func_r:02X}")

                    recv_crc = struct.unpack('<H', crc_bytes)[0]
                    calc_crc = modbus_crc16(body)
                    if recv_crc != calc_crc:
                        raise ModbusCRCError(
                            f"响应 CRC 错误: 计算 0x{calc_crc:04X}, 收到 0x{recv_crc:04X}")

                    self._frame_count += 1
                    # 返回去掉地址 / 功能码的净荷 (含 byte_count 等)
                    return body[2:]

            except ModbusError as e:
                last_exc = e
                self._err_count += 1
                if isinstance(e, ModbusExceptionResp):
                    raise  # 异常响应直接抛, 不重试
                if attempt < retries:
                    # 发生 IO 错误: 短暂等待后重试
                    time.sleep(0.05)
                    continue
                raise
            except (serial.SerialException, OSError) as e:
                last_exc = ModbusError(f"串口异常: {e}")
                self._err_count += 1
                if attempt < retries:
                    time.sleep(0.05)
                    continue
                raise last_exc

        # 不应到达, 保险
        raise last_exc if last_exc else ModbusError("未知错误")

    # ------------------------------------------------------------------
    # 标准功能码封装
    # ------------------------------------------------------------------

    def read_holding(self, start: int, count: int) -> List[int]:
        """FC03: 读保持寄存器, 返回长度为 count 的整数列表。"""
        if not (1 <= count <= 125):
            raise ValueError("count 范围 1..125")
        payload = struct.pack('>HH', start, count)
        body = self._transact(0x03, payload)
        byte_count = body[0]
        if byte_count != count * 2:
            raise ModbusFrameError("字节计数不匹配")
        regs = list(struct.unpack(f'>{count}H', body[1:1 + byte_count]))
        return regs

    def write_single(self, addr: int, value: int, *,
                     retries: int = DEFAULT_RETRIES,
                     timeout_s: Optional[float] = None) -> None:
        """FC06: 写单个保持寄存器。"""
        payload = struct.pack('>HH', addr, value & 0xFFFF)
        echo = self._transact(0x06, payload, retries=retries, timeout_s=timeout_s)
        if echo != payload:
            raise ModbusFrameError("FC06 写入回显与请求不一致")

    def write_multiple(self, start: int, values: List[int]) -> None:
        """FC16: 写多个保持寄存器。"""
        cnt = len(values)
        if not (1 <= cnt <= 123):
            raise ValueError("写入寄存器数量超范围 (1..123)")
        body = struct.pack('>HHB', start, cnt, cnt * 2)
        body += b''.join(struct.pack('>H', v & 0xFFFF) for v in values)
        echo = self._transact(0x10, body)
        expected = struct.pack('>HH', start, cnt)
        if echo != expected:
            raise ModbusFrameError("FC16 写入回显与请求不一致")

    def read_coils(self, start: int, count: int) -> List[bool]:
        """FC01: 读线圈 (输出状态)。"""
        if not (1 <= count <= 2000):
            raise ValueError("count 范围 1..2000")
        payload = struct.pack('>HH', start, count)
        body = self._transact(0x01, payload)
        byte_count = body[0]
        expected_bytes = (count + 7) // 8
        if byte_count != expected_bytes or len(body) != byte_count + 1:
            raise ModbusFrameError("FC01 字节计数不匹配")
        bits: List[bool] = []
        for byte_idx in range(byte_count):
            b = body[1 + byte_idx]
            for bit_idx in range(8):
                if len(bits) < count:
                    bits.append(bool(b & (1 << bit_idx)))
        return bits

    def write_coil(self, addr: int, value: bool) -> None:
        """FC05: 写单个线圈。"""
        v = 0xFF00 if value else 0x0000
        payload = struct.pack('>HH', addr, v)
        echo = self._transact(0x05, payload)
        if echo != payload:
            raise ModbusFrameError("FC05 写入回显与请求不一致")

    def read_discrete(self, start: int, count: int) -> List[bool]:
        """FC02: 读离散输入 (输入状态)。"""
        if not (1 <= count <= 2000):
            raise ValueError("count 范围 1..2000")
        payload = struct.pack('>HH', start, count)
        body = self._transact(0x02, payload)
        byte_count = body[0]
        expected_bytes = (count + 7) // 8
        if byte_count != expected_bytes or len(body) != byte_count + 1:
            raise ModbusFrameError("FC02 字节计数不匹配")
        bits: List[bool] = []
        for byte_idx in range(byte_count):
            b = body[1 + byte_idx]
            for bit_idx in range(8):
                if len(bits) < count:
                    bits.append(bool(b & (1 << bit_idx)))
        return bits

    # ------------------------------------------------------------------
    # 2416_F407 高层协议封装
    # ------------------------------------------------------------------

    @staticmethod
    def _u16_pair_to_i32(lo: int, hi: int) -> int:
        raw = (hi << 16) | (lo & 0xFFFF)
        return raw - 0x100000000 if raw >= 0x80000000 else raw

    @staticmethod
    def _i32_to_u16_pair(value: int) -> tuple:
        u = value & 0xFFFFFFFF
        return u & 0xFFFF, (u >> 16) & 0xFFFF

    def read_axis_status(self, axis: int) -> AxisStatus:
        """连续读取一个轴的 8 个寄存器 (每轴组内 +0..+7), 返回 AxisStatus。"""
        if not (0 <= axis < NUM_AXES):
            raise ValueError("axis 越界")
        base = REG_AXIS_BASE + axis * REG_AXIS_STRIDE
        regs = self.read_holding(base, REG_AXIS_STRIDE)
        return AxisStatus(
            position=self._u16_pair_to_i32(regs[AX_POS_LO], regs[AX_POS_HI]),
            state=regs[AX_STATE],
            homed=bool(regs[AX_HOMED]),
            speed=(regs[AX_SPEED_HI] << 16) | regs[AX_SPEED],
            target=self._u16_pair_to_i32(regs[AX_TARGET_LO], regs[AX_TARGET_HI]),
        )

    def read_all_axis_status(self) -> List[AxisStatus]:
        """一次读取全部轴的状态 (减少 RS485 总线占用)。"""
        return self.read_axis_status_range(NUM_AXES)

    def read_axis_status_range(self, axis_count: int = NUM_AXES) -> List[AxisStatus]:
        """一次读取实际注册轴状态，避免访问未注册轴寄存器。"""
        axis_count = max(0, min(int(axis_count), NUM_AXES))
        if axis_count == 0:
            return []
        regs = self.read_holding(REG_AXIS_BASE, axis_count * REG_AXIS_STRIDE)
        result: List[AxisStatus] = []
        for i in range(axis_count):
            base = i * REG_AXIS_STRIDE
            result.append(AxisStatus(
                position=self._u16_pair_to_i32(regs[base + AX_POS_LO],
                                               regs[base + AX_POS_HI]),
                state=regs[base + AX_STATE],
                homed=bool(regs[base + AX_HOMED]),
                speed=((regs[base + AX_SPEED_HI] << 16) |
                       regs[base + AX_SPEED]),
                target=self._u16_pair_to_i32(regs[base + AX_TARGET_LO],
                                             regs[base + AX_TARGET_HI]),
        ))
        return result

    def read_axis_count(self) -> int:
        """读取固件实际注册轴数，并拒绝无效协议值。"""
        count = int(self.read_holding(REG_AXIS_COUNT, 1)[0])
        if not (0 <= count <= NUM_AXES):
            raise ModbusFrameError(f"固件轴数无效: {count}, 上位机支持 0..{NUM_AXES}")
        return count

    def read_axis_capacity(self) -> int:
        capacity = int(self.read_holding(REG_AXIS_CAPACITY, 1)[0])
        if not (1 <= capacity <= NUM_AXES):
            raise ModbusFrameError(
                f"固件轴容量无效: {capacity}, 上位机最大支持 {NUM_AXES}")
        return capacity

    def read_io(self) -> tuple:
        """读取 I/O 状态: 返回 (di_bits[16], do_bits[12])。"""
        regs = self.read_holding(REG_IO_BASE, 3)
        di_lo, di_hi, do_state = regs[0], regs[1], regs[2]
        di_state = di_lo | (di_hi << 16)
        di = [bool((di_state >> i) & 1) for i in range(NUM_DI)]
        do = [bool((do_state >> i) & 1) for i in range(NUM_DO)]
        return di, do

    def read_diag(self) -> CommDiag:
        regs = self.read_holding(REG_COMM_WDT_MS, 4)
        return CommDiag(
            wdt_ms=regs[0], wdt_triggered=regs[1],
            frame_count=regs[2], err_count=regs[3])

    # ---- 命令接口 ----

    def cmd_axis(self, axis: int, cmd_code: int) -> None:
        """直接写命令寄存器 (用于停止/JOG/回原等)。"""
        if not (0 <= axis < NUM_AXES):
            raise ValueError("axis 越界")
        addr = REG_AXIS_BASE + axis * REG_AXIS_STRIDE + AX_CMD
        self.write_single(addr, cmd_code)

    def axis_stop(self, axis: int) -> None:
        self.cmd_axis(axis, CMD_STOP)

    def axis_jog(self, axis: int, speed_hz: int, positive: bool) -> None:
        """JOG 点动 (持续运行直到 axis_stop)。

        先写 SPEED 再写 CMD; 不动 TARGET, 避免覆盖用户已设置的目标点。
        固件在两次 FC06 之间处理完即可生效, RS485 多余 ~6ms 可接受。
        """
        if not (0 <= axis < NUM_AXES):
            raise ValueError("axis 越界")
        if not (100 <= speed_hz <= 100000):
            raise ValueError("speed_hz 必须在 100..100000 Hz")
        base = REG_AXIS_BASE + axis * REG_AXIS_STRIDE
        self.write_single(base + AX_SPEED_HI, (speed_hz >> 16) & 0xFFFF)
        self.write_single(base + AX_SPEED, speed_hz & 0xFFFF)
        self.write_single(base + AX_CMD, CMD_JOG_POS if positive else CMD_JOG_NEG)

    def axis_move_rel(self, axis: int, speed_hz: int, steps: int) -> None:
        """相对定位: FC16 原子写参数，再用 FC06 触发命令。"""
        if not (0 <= axis < NUM_AXES):
            raise ValueError("axis 越界")
        if not (100 <= speed_hz <= 100000):
            raise ValueError("speed_hz 必须在 100..100000 Hz")
        base = REG_AXIS_BASE + axis * REG_AXIS_STRIDE + AX_SPEED  # = base+5
        lo, hi = self._i32_to_u16_pair(steps)
        self.write_single(base - AX_SPEED + AX_SPEED_HI,
                          (speed_hz >> 16) & 0xFFFF)
        self.write_multiple(base, [speed_hz & 0xFFFF, lo, hi])
        self.cmd_axis(axis, CMD_MOVE_REL)

    def axis_move_abs(self, axis: int, speed_hz: int, target_pos: int) -> None:
        if not (0 <= axis < NUM_AXES):
            raise ValueError("axis 越界")
        if not (100 <= speed_hz <= 100000):
            raise ValueError("speed_hz 必须在 100..100000 Hz")
        base = REG_AXIS_BASE + axis * REG_AXIS_STRIDE + AX_SPEED
        lo, hi = self._i32_to_u16_pair(target_pos)
        self.write_single(base - AX_SPEED + AX_SPEED_HI,
                          (speed_hz >> 16) & 0xFFFF)
        self.write_multiple(base, [speed_hz & 0xFFFF, lo, hi])
        self.cmd_axis(axis, CMD_MOVE_ABS)

    def axis_home(self, axis: int) -> None:
        self.cmd_axis(axis, CMD_HOME)

    def axis_set_position(self, axis: int, pos: int) -> None:
        """设置当前位置 (清零或对刀)。先写 target 再写 SET_POS 命令。"""
        if not (0 <= axis < NUM_AXES):
            raise ValueError("axis 越界")
        base = REG_AXIS_BASE + axis * REG_AXIS_STRIDE + AX_TARGET_LO  # = base+6
        lo, hi = self._i32_to_u16_pair(pos)
        self.write_multiple(base, [lo, hi])
        self.cmd_axis(axis, CMD_SET_POS)

    def axis_resume_remaining(self, axis: int) -> None:
        self.cmd_axis(axis, CMD_RESUME)

    # ---- 定位点 (NVRAM — 内部 Flash 存储) ----

    def save_points_to_flash(self) -> None:
        # Flash 擦除可能明显慢于普通 RTU 请求，且超时后不可自动重复擦写。
        self.write_single(REG_SAVE_POINTS, 0xA55A,
                          retries=0, timeout_s=FLASH_SAVE_TIMEOUT_S)

    def read_position_points(self, axis: int) -> List[int]:
        if not (0 <= axis < NUM_AXES):
            raise ValueError("axis 越界")
        base = REG_POINTS_BASE + axis * REG_POINTS_STRIDE
        regs = self.read_holding(base, 16)
        result = []
        for i in range(8):
            lo = regs[i * 2]
            hi = regs[i * 2 + 1]
            result.append(self._u16_pair_to_i32(lo, hi))
        return result

    def write_position_point(self, axis: int, index: int, pos: int) -> None:
        if not (0 <= axis < NUM_AXES) or not (0 <= index < 8):
            raise ValueError("axis/index 越界")
        base = REG_POINTS_BASE + axis * REG_POINTS_STRIDE + index * 2
        lo, hi = self._i32_to_u16_pair(pos)
        self.write_multiple(base, [lo, hi])

    def write_all_position_points(self, axis: int, positions: List[int]) -> None:
        if not (0 <= axis < NUM_AXES):
            raise ValueError("axis 越界")
        base = REG_POINTS_BASE + axis * REG_POINTS_STRIDE
        vals = []
        for i in range(min(8, len(positions))):
            lo, hi = self._i32_to_u16_pair(positions[i])
            vals.append(lo)
            vals.append(hi)
        while len(vals) < 16:
            vals.append(0)
        self.write_multiple(base, vals)

    @staticmethod
    def _encode_point_name(name: str) -> List[int]:
        """编码为固定 12 个 UTF-16 码元，不截断代理对。"""
        units: List[int] = []
        for char in str(name):
            encoded = char.encode("utf-16-le")
            char_units = [
                encoded[i] | (encoded[i + 1] << 8)
                for i in range(0, len(encoded), 2)
            ]
            if len(units) + len(char_units) > POINT_NAME_UNITS:
                break
            units.extend(char_units)
        return units + [0] * (POINT_NAME_UNITS - len(units))

    @staticmethod
    def _decode_point_name(units: List[int]) -> str:
        data = bytearray()
        for unit in units[:POINT_NAME_UNITS]:
            if unit == 0:
                break
            data.extend(struct.pack("<H", unit & 0xFFFF))
        return bytes(data).decode("utf-16-le", errors="replace")

    def read_position_point_names(self, axis: int) -> List[str]:
        if not (0 <= axis < NUM_AXES):
            raise ValueError("axis 越界")
        base = REG_POINT_NAME_BASE + axis * REG_POINT_NAME_STRIDE
        regs = self.read_holding(base, REG_POINT_NAME_STRIDE)
        return [
            self._decode_point_name(
                regs[i * POINT_NAME_UNITS:(i + 1) * POINT_NAME_UNITS])
            for i in range(8)
        ]

    def write_position_point_names(self, axis: int, names: List[str]) -> None:
        if not (0 <= axis < NUM_AXES):
            raise ValueError("axis 越界")
        values: List[int] = []
        for i in range(8):
            values.extend(self._encode_point_name(names[i] if i < len(names) else ""))
        base = REG_POINT_NAME_BASE + axis * REG_POINT_NAME_STRIDE
        self.write_multiple(base, values)

    def read_position_point_speeds(self, axis: int) -> List[int]:
        """读取某轴 8 个定位点的运行速度 (Hz)。"""
        if not (0 <= axis < NUM_AXES):
            raise ValueError("axis 越界")
        base = REG_POINT_SPEED_BASE + axis * REG_POINT_SPEED_STRIDE
        regs = self.read_holding(base, 16)
        result = []
        for i in range(8):
            lo = regs[i * 2]
            hi = regs[i * 2 + 1]
            result.append(self._u16_pair_to_i32(lo, hi) & 0xFFFFFFFF)
        return result

    def write_position_point_speeds(self, axis: int, speeds: List[int]) -> None:
        """写入某轴 8 个定位点的运行速度 (Hz)，掉电保持需再调用保存命令。"""
        if not (0 <= axis < NUM_AXES):
            raise ValueError("axis 越界")
        vals: List[int] = []
        for i in range(8):
            spd = speeds[i] if i < len(speeds) else POINT_SPEED_DEFAULT
            spd = int(spd)
            if not (POINT_SPEED_MIN <= spd <= POINT_SPEED_MAX):
                raise ValueError(f"点位速度 {spd} 超出 {POINT_SPEED_MIN}..{POINT_SPEED_MAX} Hz")
            vals.append(spd & 0xFFFF)
            vals.append((spd >> 16) & 0xFFFF)
        base = REG_POINT_SPEED_BASE + axis * REG_POINT_SPEED_STRIDE
        self.write_multiple(base, vals)

    def read_position_point_data(self, axis: int) -> dict:
        return {
            'positions': self.read_position_points(axis),
            'names': self.read_position_point_names(axis),
            'speeds': self.read_position_point_speeds(axis),
        }

    def write_position_point_data(
            self, axis: int, positions: List[int], names: List[str],
            speeds: Optional[List[int]] = None) -> None:
        """一次事务写入坐标、名称、速度。

        speeds 为 None 时保持设备当前保存的点位速度不变。
        固件侧对同一事务内的寄存器写入做原子提交，任一项非法则整批不生效。
        """
        self.write_all_position_points(axis, positions)
        if speeds is not None:
            self.write_position_point_speeds(axis, speeds)
        self.write_position_point_names(axis, names)

    # ---- 软限位 (可选行程保护, 掉电保持) ----

    def read_soft_limit(self, axis: int) -> dict:
        """读取某轴软限位设置。

        返回 {'enable': bool, 'min': int, 'max': int, 'valid': bool}。
        valid 由固件计算: 只有已启用且 min < max 时才为 True。
        """
        if not (0 <= axis < NUM_AXES):
            raise ValueError("axis 越界")
        base = REG_SOFT_LIMIT_BASE + axis * REG_SOFT_LIMIT_STRIDE
        regs = self.read_holding(base, REG_SOFT_LIMIT_STRIDE)
        return {
            'enable': bool(regs[SL_ENABLE]),
            'min': self._u16_pair_to_i32(regs[SL_NEG_LO], regs[SL_NEG_HI]),
            'max': self._u16_pair_to_i32(regs[SL_POS_LO], regs[SL_POS_HI]),
            'valid': bool(regs[SL_VALID]),
        }

    def write_soft_limit(self, axis: int, enable: bool,
                         min_pos: int, max_pos: int,
                         save: bool = True, verify: bool = True) -> None:
        """写入某轴软限位。

        固件在启用状态下要求 min < max, 且 FC16 逐寄存器校验,
        所以写入顺序必须是: 先写 0 关闭 -> FC16 写四个边界寄存器 ->
        再写 +0 使能 -> 写 +7=0xA55A 保存 -> 读回校验。

        save=False 时只改 RAM (掉电丢失), 用于确认前预览。
        """
        if not (0 <= axis < NUM_AXES):
            raise ValueError("axis 越界")
        min_pos = int(min_pos)
        max_pos = int(max_pos)
        if not (-0x80000000 <= min_pos <= 0x7FFFFFFF):
            raise ValueError("负向软限位超出 int32 范围")
        if not (-0x80000000 <= max_pos <= 0x7FFFFFFF):
            raise ValueError("正向软限位超出 int32 范围")
        if enable and min_pos >= max_pos:
            raise ValueError("启用软限位时要求 负向限位 < 正向限位")

        base = REG_SOFT_LIMIT_BASE + axis * REG_SOFT_LIMIT_STRIDE

        # 1) 先禁用, 避免写入过程中出现 min >= max 的中间态而被固件拒绝。
        self.write_single(base + SL_ENABLE, 0)
        # 2) 一次事务写四个边界寄存器。
        neg_lo, neg_hi = self._i32_to_u16_pair(min_pos)
        pos_lo, pos_hi = self._i32_to_u16_pair(max_pos)
        self.write_multiple(base + SL_NEG_LO,
                            [neg_lo, neg_hi, pos_lo, pos_hi])
        # 3) 再按目标状态使能。
        self.write_single(base + SL_ENABLE, 1 if enable else 0)
        # 4) 需要掉电保持时立即落盘。
        if save:
            self.write_single(base + SL_SAVE, SOFT_LIMIT_CMD_SAVE)

        if verify:
            got = self.read_soft_limit(axis)
            if (got['enable'] != bool(enable) or got['min'] != min_pos or
                    got['max'] != max_pos):
                raise ModbusFrameError(
                    f"软限位读回不一致: 期望 en={int(enable)} "
                    f"min={min_pos} max={max_pos}, "
                    f"实读 en={int(got['enable'])} "
                    f"min={got['min']} max={got['max']}")

    def disable_soft_limit(self, axis: int, save: bool = True) -> None:
        """关闭某轴软限位, 边界值保持不变。"""
        if not (0 <= axis < NUM_AXES):
            raise ValueError("axis 越界")
        base = REG_SOFT_LIMIT_BASE + axis * REG_SOFT_LIMIT_STRIDE
        self.write_single(base + SL_ENABLE, 0)
        if save:
            self.write_single(base + SL_SAVE, SOFT_LIMIT_CMD_SAVE)

    # ---- 系统 ----

    def emergency_stop(self) -> None:
        """急停所有轴 (E_STOP)。"""
        self.write_single(REG_E_STOP, 0x00FF)

    def heartbeat(self) -> None:
        """心跳: 上位机周期写入, 刷新固件通信看门狗。"""
        self.write_single(REG_HEARTBEAT, 1)

    def read_op_mode(self) -> int:
        """读取当前运行模式: 0=正常运行, 1=调试模式。"""
        return self.read_holding(REG_OP_MODE, 1)[0]

    def write_op_mode(self, mode: int) -> None:
        """设置运行模式: 0=正常运行, 1=调试模式。"""
        self.write_single(REG_OP_MODE, mode & 0x01)

    def set_home_params(self, fast_hz: int, slow_hz: int, backoff_hz: int, mode: int = 0) -> None:
        """写入回原参数。backoff_hz 为触发后反向回退速度 (Hz), 写入 221 号寄存器;
        204 (回退步数) 已废弃, 仍按旧值写入以兼容旧固件, 固件会忽略。"""
        if not (100 <= fast_hz <= 100000 and 100 <= slow_hz <= 100000):
            raise ValueError("回原频率必须在 100..100000 Hz")
        self.write_multiple(REG_HOME_FAST_HI,
                            [(fast_hz >> 16) & 0xFFFF,
                             (slow_hz >> 16) & 0xFFFF])
        self.write_multiple(REG_HOME_FAST,
                            [fast_hz & 0xFFFF, slow_hz & 0xFFFF,
                             backoff_hz, 1 if mode == 1 else 0])
        self.write_single(REG_HOME_BACKOFF_HZ, backoff_hz & 0xFFFF)

    def set_comm_wdt_ms(self, timeout_ms: int) -> None:
        self.write_single(REG_COMM_WDT_MS, timeout_ms & 0xFFFF)

    def get_version(self) -> int:
        return self.read_holding(REG_VERSION, 1)[0]

    # ---- 轴配置接口 ----

    def read_axis_config(self, axis: int, use_extended: bool = True) -> dict:
        """读取轴绑定配置。配置只读, 不允许上位机写入。"""
        if not (0 <= axis < NUM_AXES):
            raise ValueError("轴号超范围")
        try:
            if not use_extended:
                raise ModbusError("use legacy axis config")
            regs = self.read_holding(REG_CFG_EXT_BASE + axis * REG_CFG_EXT_STRIDE,
                                     REG_CFG_EXT_STRIDE)
            flags = regs[7]
            return {
                'pulse_mode': regs[0],
                'pwm': None if regs[1] == PIN_NONE else regs[1],
                'soft': None if regs[2] == PIN_NONE else regs[2],
                'dir': None if regs[3] == PIN_NONE else regs[3],
                'home': None if regs[4] == PIN_NONE else regs[4],
                'limp': None if regs[5] == PIN_NONE else regs[5],
                'limn': None if regs[6] == PIN_NONE else regs[6],
                'inv': bool(flags & 0x0001),
                'home_dir': bool(flags & 0x0004),
            }
        except ModbusError:
            regs = self.read_holding(REG_CFG_BASE + axis * REG_CFG_STRIDE,
                                     REG_CFG_STRIDE)
            flags = regs[5]
            return {
                'pulse_mode': PULSE_MODE_PWM,
                'pwm': None if regs[0] == PIN_NONE else regs[0],
                'soft': None,
                'dir': None if regs[1] == PIN_NONE else regs[1],
                'home': None if regs[2] == PIN_NONE else regs[2],
                'limp': None if regs[3] == PIN_NONE else regs[3],
                'limn': None if regs[4] == PIN_NONE else regs[4],
                'inv': bool(flags & 0x0001),
                'home_dir': bool(flags & 0x0004),
            }

    def read_all_config(self) -> dict:
        """读取系统参数和只读轴绑定配置。"""
        sys_regs = self.read_holding(REG_SYS_BASE, 18)
        home_fast_hi = 0
        home_slow_hi = 0
        try:
            home_hi = self.read_holding(REG_HOME_FAST_HI, 2)
            home_fast_hi, home_slow_hi = home_hi
        except ModbusError:
            pass
        try:
            backoff_hz = self.read_holding(REG_HOME_BACKOFF_HZ, 1)[0]
        except ModbusError:
            backoff_hz = 0
        axis_count = max(0, min(int(sys_regs[1]), NUM_AXES))
        system = {
            'axis_count': axis_count,
            'home_fast': (home_fast_hi << 16) | sys_regs[2],
            'home_slow': (home_slow_hi << 16) | sys_regs[3],
            'home_back': sys_regs[4],        # 已废弃, 仅为兼容保留
            'home_backoff_hz': backoff_hz,   # 回退速度 (Hz), 固件 v2 起生效
            'home_mode': sys_regs[5],
            'slave_addr': sys_regs[7] if sys_regs[7] >= 1 else 1,
            'subdivision': sys_regs[8],
            'jog_step': sys_regs[9],
            'comm_wdt_ms': sys_regs[10],
            'axis_capacity': max(1, min(int(sys_regs[16]), NUM_AXES)),
        }
        use_extended_cfg = True
        axes = []
        for i in range(NUM_AXES):
            axes.append(self.read_axis_config(i, use_extended=use_extended_cfg))
        return {'system': system, 'axes': axes}

    def write_axis_configs(self, axes_cfg: list) -> None:
        """轴绑定由固件业务代码固定维护，当前协议不允许上位机写入。"""
        raise ModbusError("当前固件不允许上位机写入轴绑定配置")

    def _write_axis_configs_legacy(self, axes_cfg: list) -> None:
        """旧版批量写入全部轴的引脚配置 (保留作协议参考, 不应调用)。"""
        vals = []
        for ax in axes_cfg:
            vals.append(ax.get('pwm', 0))
            vals.append(ax.get('dir', 0))
            vals.append(ax.get('home', 0xFF) if ax.get('home', 0xFF) == 0xFF else ax.get('home', 0))
            vals.append(ax.get('limp', 0xFF) if ax.get('limp', 0xFF) == 0xFF else ax.get('limp', 0))
            vals.append(ax.get('limn', 0xFF) if ax.get('limn', 0xFF) == 0xFF else ax.get('limn', 0))
            flags = 0
            if ax.get('inv', False):
                flags |= 1
            if ax.get('home_dir', False):
                flags |= 4
            vals.append(flags)
        self.write_multiple(REG_CFG_BASE, vals)

    def write_system_params(self, sys_cfg: dict) -> None:
        """写入系统参数 (回原参数、站号、细分、看门狗等)。"""
        self.set_home_params(
            sys_cfg.get('home_fast', 1250),
            sys_cfg.get('home_slow', 300),
            sys_cfg.get('home_backoff_hz', sys_cfg.get('home_back', 300)),
            sys_cfg.get('home_mode', 0),
        )
        addr = sys_cfg.get('slave_addr', 1)
        if 1 <= addr <= 247:
            self.write_single(REG_SLAVE_ADDR, addr)
            self._slave_addr = addr
        self.write_single(REG_SUBDIV, sys_cfg.get('subdivision', 5000))
        self.write_single(REG_JOG_STEP, sys_cfg.get('jog_step', 100))
        self.write_single(REG_COMM_WDT_MS, sys_cfg.get('comm_wdt_ms', 2000))

    def apply_config(self) -> None:
        """轴绑定由固件业务代码固定维护，当前协议不允许上位机应用重配。"""
        raise ModbusError("当前固件不允许上位机应用轴绑定配置")

# ============================================================================
#  自检 (可独立运行: python modbus_master.py COM5)
# ============================================================================

if __name__ == "__main__":
    import sys

    if len(sys.argv) < 2:
        print("用法: python modbus_master.py <COM端口> [波特率=115200] [从站地址=1]")
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUD
    addr = int(sys.argv[3]) if len(sys.argv) > 3 else 1

    ser = serial.Serial(port, baudrate=baud, bytesize=8,
                        parity='N', stopbits=1, timeout=0.3)
    mb = ModbusMaster(slave_addr=addr)
    mb.set_serial(ser)
    try:
        ver = mb.get_version()
        print(f"固件版本: 0x{ver:04X}")
        mode = mb.read_op_mode()
        print(f"运行模式: {MODE_NAMES.get(mode, mode)} ({mode})")
        for i in range(NUM_AXES):
            st = mb.read_axis_status(i)
            print(f"轴{i}: pos={st.position} state={STATE_NAMES.get(st.state)} "
                  f"speed={st.speed} homed={st.homed}")
    finally:
        ser.close()
