#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
modbus_tcp_client.py
=====================
Modbus TCP 客户端 — 用于对接海康视觉系统 (VisionMaster / SC系列视觉控制器)

海康视觉系统通常通过 Modbus TCP 对外暴露寄存器:
  - 视觉触发寄存器 (写 1 触发拍照)
  - 结果寄存器 (X偏移、Y偏移、角度、OK/NG 等)
  - 状态寄存器 (视觉忙/空闲)

与 Modbus RTU 的关键区别:
  1. 物理层: 以太网 (socket) 替代串口 (pyserial)
  2. 帧格式: MBAP 头 (6字节) 替代 地址字节; 无 CRC
  3. 端口: 默认 502
"""

from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass
from threading import Lock
from typing import Optional


# ============================================================================
#  异常类
# ============================================================================

class ModbusTCPError(Exception):
    """Modbus TCP 通信异常。"""


class ModbusTCPTimeout(ModbusTCPError):
    """超时。"""


class ModbusTCPExceptionResp(ModbusTCPError):
    """从机返回异常码。"""
    def __init__(self, func: int, exc: int):
        self.func = func
        self.exc = exc
        super().__init__(f"Modbus TCP 异常 func={func} exc={exc}")


# ============================================================================
#  MBAP 头
# ============================================================================

def build_mbap(transaction_id: int, pdu_len: int, unit_id: int = 1) -> bytes:
    """构造 MBAP 头: 事务ID(2) | 协议ID(2) | 长度(2) | 单元ID(1)"""
    return struct.pack(">HHHB", transaction_id, 0, pdu_len + 1, unit_id)


# ============================================================================
#  Modbus TCP 客户端
# ============================================================================

@dataclass
class VisionResult:
    """海康视觉典型结果数据结构。

    示例寄存器映射 (需根据实际视觉方案调整):
      0-1:  X 偏移 (float 占2个寄存器)
      2-3:  Y 偏移
      4:   角度 (int16, 单位 0.01°)
      5:   结果码 (0=OK, 1=NG, 2=超时)
    """
    x_offset: float = 0.0
    y_offset: float = 0.0
    angle: float = 0.0
    result_code: int = 0

    @property
    def is_ok(self) -> bool:
        return self.result_code == 0

    @property
    def is_ng(self) -> bool:
        return self.result_code == 1


class ModbusTCPClient:
    """Modbus TCP 主站客户端。

    可与现有 ModbusMaster (RTU) 并存, 在同一个上位机程序中同时使用。

    使用示例:
        vis = ModbusTCPClient("192.168.1.100")
        vis.connect()
        result = vis.read_vision_result(start_reg=0)
        vis.disconnect()
    """

    def __init__(self, host: str, port: int = 502, timeout: float = 1.0,
                 unit_id: int = 1):
        if not 0 <= unit_id <= 247:
            raise ValueError("unit_id 必须在 0..247")
        self.host = host
        self.port = port
        self.timeout = timeout
        self.unit_id = unit_id
        self._sock: Optional[socket.socket] = None
        self._lock = Lock()
        self._tid = 0

    def _next_tid(self) -> int:
        self._tid = (self._tid + 1) & 0xFFFF
        return self._tid

    # ---------- 连接管理 ----------

    def connect(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(self.timeout)
        self._sock.connect((self.host, self.port))

    def disconnect(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    @property
    def is_connected(self) -> bool:
        return self._sock is not None

    # ---------- 底层收发 ----------

    def _send_recv(self, pdu: bytes) -> bytes:
        """发送 PDU, 接收并校验返回帧。"""
        tid = self._next_tid()
        if not pdu:
            raise ModbusTCPError("PDU 不能为空")
        frame = build_mbap(tid, len(pdu), self.unit_id) + pdu
        sock = self._sock
        if sock is None:
            raise ModbusTCPError("未连接")

        with self._lock:
            sock.sendall(frame)

            # 先收 MBAP 头 (7 字节)
            try:
                header = self._recv_exact(sock, 7)
                resp_tid, protocol_id, recv_len, unit_id = struct.unpack(">HHHB", header)
                if resp_tid != tid:
                    raise ModbusTCPError(f"事务 ID 不匹配: exp={tid} got={resp_tid}")
                if protocol_id != 0:
                    raise ModbusTCPError(f"协议 ID 非法: {protocol_id}")
                if unit_id != self.unit_id:
                    raise ModbusTCPError(
                        f"单元 ID 不匹配: exp={self.unit_id} got={unit_id}")
                if recv_len < 2 or recv_len > 254:
                    raise ModbusTCPError(f"MBAP 长度非法: {recv_len}")

                remaining = self._recv_exact(sock, recv_len - 1)
            except socket.timeout as exc:
                raise ModbusTCPTimeout(
                    f"超时: {self.host}:{self.port}") from exc
            resp = header + remaining

        func = remaining[0]
        expected_func = pdu[0]
        if func & 0x80:
            if len(remaining) != 2:
                raise ModbusTCPError("异常响应长度非法")
            raise ModbusTCPExceptionResp(func & 0x7F, remaining[1])
        if func != expected_func:
            raise ModbusTCPError(
                f"功能码不匹配: exp=0x{expected_func:02X} got=0x{func:02X}")

        return resp

    @staticmethod
    def _recv_exact(sock: socket.socket, n: int) -> bytes:
        data = b""
        while len(data) < n:
            chunk = sock.recv(n - len(data))
            if not chunk:
                raise ModbusTCPError("连接已断开")
            data += chunk
        return data

    # ---------- 标准 Modbus 功能码 ----------

    def read_holding_registers(self, start_addr: int, count: int) -> list[int]:
        if not 1 <= count <= 125:
            raise ValueError("保持寄存器数量必须在 1..125")
        pdu = struct.pack(">BHH", 0x03, start_addr, count)
        resp = self._send_recv(pdu)
        byte_count = resp[8]
        if byte_count != count * 2 or len(resp) != 9 + byte_count:
            raise ModbusTCPError(
                f"FC03 响应长度非法: byte_count={byte_count}, count={count}")
        vals = resp[9:9 + byte_count]
        return list(struct.unpack(f">{count}H", vals))

    def read_holding_register(self, addr: int) -> int:
        return self.read_holding_registers(addr, 1)[0]

    def write_single_register(self, addr: int, value: int) -> None:
        pdu = struct.pack(">BHH", 0x06, addr, value)
        resp = self._send_recv(pdu)
        if len(resp) != 12 or resp[7:] != pdu:
            raise ModbusTCPError("FC06 写回显不匹配")

    def write_multiple_registers(self, start_addr: int, values: list[int]) -> None:
        n = len(values)
        if not 1 <= n <= 123:
            raise ValueError("多寄存器写入数量必须在 1..123")
        data = struct.pack(f">{n}H", *values)
        pdu = struct.pack(">BHHB", 0x10, start_addr, n, n * 2) + data
        resp = self._send_recv(pdu)
        expected = struct.pack(">BHH", 0x10, start_addr, n)
        if len(resp) != 12 or resp[7:] != expected:
            raise ModbusTCPError("FC16 写回显不匹配")

    def read_coils(self, start_addr: int, count: int) -> list[int]:
        if not 1 <= count <= 2000:
            raise ValueError("线圈数量必须在 1..2000")
        pdu = struct.pack(">BHH", 0x01, start_addr, count)
        resp = self._send_recv(pdu)
        byte_count = resp[8]
        expected_count = (count + 7) // 8
        if byte_count != expected_count or len(resp) != 9 + byte_count:
            raise ModbusTCPError(
                f"FC01 响应长度非法: byte_count={byte_count}, count={count}")
        bits = resp[9:9 + byte_count]
        result = []
        for byte_val in bits:
            for i in range(8):
                result.append((byte_val >> i) & 1)
        return result[:count]

    def write_single_coil(self, addr: int, state: bool) -> None:
        pdu = struct.pack(">BHH", 0x05, addr, 0xFF00 if state else 0x0000)
        resp = self._send_recv(pdu)
        if len(resp) != 12 or resp[7:] != pdu:
            raise ModbusTCPError("FC05 写回显不匹配")

    # ---------- 海康视觉高层封装 ----------

    # --- 以下寄存器地址需根据实际海康视觉方案调整 ---
    # 典型海康 VisionMaster Modbus TCP 映射:
    REG_TRIGGER  = 0     # 写 1 触发拍照
    REG_X_LO     = 0     # X 偏移低字
    REG_X_HI     = 1     # X 偏移高字 (float = HI<<16 | LO)
    REG_Y_LO     = 2     # Y 偏移低字
    REG_Y_HI     = 3
    REG_ANGLE    = 4     # 角度 (int16, 单位 0.01°)
    REG_RESULT   = 5     # 结果: 0=OK 1=NG 2=超时
    REG_BUSY     = 6     # 状态: 0=空闲 1=拍照中
    VISION_REGS  = 7     # 总共读取 7 个寄存器

    def trigger(self) -> None:
        """触发视觉拍照 (写 1 到触发寄存器)。"""
        self.write_single_register(self.REG_TRIGGER, 1)
        time.sleep(0.01)
        self.write_single_register(self.REG_TRIGGER, 0)

    def wait_ready(self, timeout_s: float = 5.0) -> bool:
        """等待视觉空闲。返回 True=就绪, False=超时。"""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            busy = self.read_holding_register(self.REG_BUSY)
            if busy == 0:
                return True
            time.sleep(0.05)
        return False

    def read_vision_result(self, start_reg: int = 0) -> VisionResult:
        """读取视觉结果 (7 个寄存器, 解析为 VisionResult)。"""
        regs = self.read_holding_registers(start_reg, self.VISION_REGS)
        x_raw = (regs[1] << 16) | regs[0]
        y_raw = (regs[3] << 16) | regs[2]
        return VisionResult(
            x_offset=self._int32_to_float(x_raw),
            y_offset=self._int32_to_float(y_raw),
            angle=regs[4] * 0.01,
            result_code=regs[5],
        )

    def trigger_and_read(self, timeout_s: float = 5.0) -> VisionResult:
        """触发拍照并等待结果: 触发→等空闲→读取结果。"""
        self.trigger()
        if not self.wait_ready(timeout_s):
            raise ModbusTCPTimeout("视觉拍照超时")
        return self.read_vision_result(self.REG_X_LO)

    @staticmethod
    def _int32_to_float(raw: int) -> float:
        """将 2-register int32 转为浮点数 (海康常用放大 1000 倍存储)。"""
        if raw >= 0x80000000:
            raw -= 0x100000000
        return raw / 1000.0


# ============================================================================
#  整合示例: 视觉 + 电机联动
# ============================================================================

def demo_vision_motor_integration(
    com_port: str = "COM5",
    vision_ip: str = "192.168.1.100",
):
    """演示: 海康视觉定位 → 单片机电机移动。

    流程:
      1. 连接视觉 (Modbus TCP) 和电机 (Modbus RTU)
      2. 触发视觉拍照, 获取 X/Y 偏移
      3. 如果 OK, 将偏移作为电机目标位置移动
      4. 断开连接
    """
    from modbus_master import DEFAULT_BAUD, ModbusMaster
    import serial

    vis = ModbusTCPClient(vision_ip)
    mtr = ModbusMaster()

    try:
        # 1. 连接
        print(f"[视觉] 连接 {vision_ip}:502 ...")
        vis.connect()
        print(f"[电机] 连接 {com_port} ...")
        ser = serial.Serial(com_port, baudrate=DEFAULT_BAUD, bytesize=8,
                            parity='N', stopbits=1, timeout=0.3)
        mtr.set_serial(ser)

        # 2. 触发视觉, 读取结果
        print("[视觉] 触发拍照...")
        result = vis.trigger_and_read(timeout_s=5.0)

        if not result.is_ok:
            print(f"[视觉] NG! code={result.result_code}, 跳过定位")
            return

        print(f"[视觉] OK! X={result.x_offset:.3f} Y={result.y_offset:.3f} A={result.angle:.2f}°")

        # 3. 电机移动 (相对定位)
        # 把视觉偏移转为电机步数 (假设 每mm = 1000 步)
        scale = 1000  # steps/mm
        x_steps = int(result.x_offset * scale)
        y_steps = int(result.y_offset * scale)

        # 轴0 (X) 移动
        print(f"[电机] 轴0 相对移动 {x_steps} 步...")
        mtr.axis_move_rel(axis=0, speed_hz=1250, steps=x_steps)

        # 轴1 (Y) 移动
        print(f"[电机] 轴1 相对移动 {y_steps} 步...")
        mtr.axis_move_rel(axis=1, speed_hz=1250, steps=y_steps)

        # 等待到位
        import time
        while True:
            s0 = mtr.read_axis_status(0)
            s1 = mtr.read_axis_status(1)
            if s0.state == 0 and s1.state == 0:  # IDLE
                break
            time.sleep(0.1)

        print("[电机] 定位完成!")

    finally:
        vis.disconnect()
        ser = mtr.serial
        mtr.set_serial(None)
        if ser:
            ser.close()
        print("断开所有连接")


# ============================================================================
#  独立运行
# ============================================================================

if __name__ == "__main__":
    import sys

    if len(sys.argv) >= 2:
        ip = sys.argv[1]
    else:
        ip = "192.168.1.100"

    print(f"测试连接海康视觉: {ip}:502")
    client = ModbusTCPClient(ip)
    try:
        client.connect()
        print("连接成功!")

        # 读取结果
        result = client.read_vision_result()
        print(f"视觉结果: X={result.x_offset:.3f} Y={result.y_offset:.3f} "
              f"A={result.angle:.2f}° {'OK' if result.is_ok else 'NG'}")

    except ModbusTCPTimeout:
        print("连接超时, 请检查 IP 和端口")
    except ModbusTCPError as e:
        print(f"通信错误: {e}")
    finally:
        client.disconnect()
