#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
comm_worker.py
==================
后台通信线程: 把 ModbusMaster 与 Qt UI 解耦。

职责
----
1. 持有命令队列, UI 通过 enqueue() 提交 "立即执行" 的指令 (jog / move / stop ...)。
2. 周期性轮询轴状态、I/O 状态, 通过 Qt Signal 推送给 UI。
3. 周期心跳, 防止固件通信看门狗超时把电机急停。
4. 异常隔离: 单次通信失败不会让线程退出; 连续 N 次失败则发 disconnected 信号。
"""

from __future__ import annotations

import queue
import time
from dataclasses import dataclass
from typing import Callable, List, Optional

from PySide6.QtCore import QObject, QThread, Signal

from modbus_master import (
    AxisStatus,
    CommDiag,
    MODE_NORMAL,
    MODE_NAMES,
    ModbusError,
    ModbusExceptionResp,
    ModbusMaster,
    NUM_AXES,
)


# ============================================================================
#  Worker
# ============================================================================

@dataclass
class WorkerConfig:
    poll_period_ms: int = 150        # 轴状态轮询周期
    io_period_ms: int = 250          # I/O 轮询周期 (低频)
    diag_period_ms: int = 1000       # 通信诊断刷新周期
    heartbeat_ms: int = 800          # 心跳周期 (< 固件 wdt 1/2)
    err_disconnect_threshold: int = 5  # 连续失败次数, 超过则视为断线
    axis_count: int = NUM_AXES       # 固件实际注册轴数


class CommWorker(QObject):
    """运行在 QThread 中的通信工人。

    所有 Qt Signal 在 worker 所在线程发射, UI 端用 QueuedConnection 即可。
    """

    axis_status = Signal(list)           # List[AxisStatus]
    io_status = Signal(list, list)       # (di_bits, do_bits)
    comm_diag = Signal(object)           # CommDiag
    op_mode_changed = Signal(int)        # 运行模式: 0=正常运行, 1=调试
    config_read = Signal(object)         # read_all_config() 返回的配置字典
    error = Signal(str)
    info = Signal(str)
    connected = Signal()
    disconnected = Signal()
    cmd_finished = Signal(str, bool, str)  # (tag, ok, message)
    command_result = Signal(str, object)   # (tag, return value)

    def __init__(self, mb: ModbusMaster, config: Optional[WorkerConfig] = None):
        super().__init__()
        self._mb = mb
        self._cfg = config or WorkerConfig()
        self._cmd_q: "queue.Queue[tuple]" = queue.Queue()
        self._running = False
        self._poll_enabled = True
        self._estop_pending = False
        self._err_streak = 0
        self._axis_poll_err_streak = 0
        self._axis_poll_reported = False
        self._last_poll = 0.0
        self._last_io = 0.0
        self._last_diag = 0.0
        self._last_hb = 0.0
        self._last_mode = 0.0
        self._prev_mode = MODE_NORMAL

    # ---- 控制接口 (UI 线程调用) ----

    def stop(self) -> None:
        self._running = False
        self._estop_pending = False
        while True:
            try:
                self._cmd_q.get_nowait()
            except queue.Empty:
                break

    def set_poll(self, on: bool) -> None:
        self._poll_enabled = on

    def enqueue(self, fn: Callable, *args, tag: str = "") -> None:
        """提交一个一次性命令 (在通信线程中执行 fn(*args))。tag 用于结果回调匹配。"""
        self._cmd_q.put((fn, args, tag))

    def emergency_stop(self) -> None:
        while True:
            try:
                self._cmd_q.get_nowait()
            except queue.Empty:
                break
        self._estop_pending = True

    # ---- 主循环 ----

    def run(self) -> None:
        self._running = True
        self._err_streak = 0
        self._axis_poll_err_streak = 0
        self._axis_poll_reported = False
        self.connected.emit()
        self.info.emit("通信线程已启动")

        while self._running:
            if self._estop_pending:
                self._estop_pending = False
                self._exec_cmd(self._mb.emergency_stop, (), "ESTOP")
                now = time.monotonic() * 1000.0
                self._last_poll = now
                self._last_io = now
                self._last_diag = now
                self._last_hb = now
                self._last_mode = now

            # 1) 处理所有命令队列中的项 (最多 8 条防饿死轮询)
            for _ in range(8):
                if not self._running:
                    break
                try:
                    fn, args, tag = self._cmd_q.get_nowait()
                except queue.Empty:
                    break
                self._exec_cmd(fn, args, tag)

            # 2) 周期任务
            now = time.monotonic() * 1000.0
            if self._poll_enabled:
                if now - self._last_poll >= self._cfg.poll_period_ms:
                    self._last_poll = now
                    self._do_poll_axis()

                if now - self._last_io >= self._cfg.io_period_ms:
                    self._last_io = now
                    self._do_poll_io()

                if now - self._last_diag >= self._cfg.diag_period_ms:
                    self._last_diag = now
                    self._do_poll_diag()

                if now - self._last_hb >= self._cfg.heartbeat_ms:
                    self._last_hb = now
                    self._do_heartbeat()

                if now - self._last_mode >= self._cfg.diag_period_ms:
                    self._last_mode = now
                    self._do_poll_mode()

            # 3) 让出 CPU
            QThread.msleep(5)

        self.disconnected.emit()
        self.info.emit("通信线程已停止")

    # ---- 内部辅助 ----

    def _bump_err(self) -> None:
        self._err_streak += 1
        if self._err_streak >= self._cfg.err_disconnect_threshold:
            self.error.emit(f"连续 {self._err_streak} 次通信失败, 视为断线")
            self._running = False

    def _reset_err(self) -> None:
        self._err_streak = 0

    def _exec_cmd(self, fn: Callable, args: tuple, tag: str) -> None:
        try:
            result = fn(*args)
            self._reset_err()
            if tag == "read_config":
                self.config_read.emit(result)
            self.command_result.emit(tag, result)
            self.cmd_finished.emit(tag, True, "OK")
        except ModbusExceptionResp as e:
            self.cmd_finished.emit(tag, False, str(e))
            self.error.emit(f"[{tag}] {e}")
            # 不 bump_err: 这是从机正常拒绝, 不算链路故障
        except ModbusError as e:
            self.cmd_finished.emit(tag, False, str(e))
            self.error.emit(f"[{tag}] {e}")
            self._bump_err()
        except Exception as e:
            self.cmd_finished.emit(tag, False, str(e))
            self.error.emit(f"[{tag}] 未知错误: {e}")
            self._bump_err()

    def _do_poll_axis(self) -> None:
        if self._cfg.axis_count <= 0:
            return
        try:
            statuses = self._mb.read_axis_status_range(self._cfg.axis_count)
            self.axis_status.emit(statuses)
            if self._axis_poll_reported:
                self.info.emit("轴状态轮询已恢复")
            self._axis_poll_err_streak = 0
            self._axis_poll_reported = False
            self._reset_err()
        except ModbusError as e:
            self._axis_poll_err_streak += 1
            if (self._axis_poll_err_streak >= self._cfg.err_disconnect_threshold
                    and not self._axis_poll_reported):
                self.error.emit(
                    f"轴状态连续 {self._axis_poll_err_streak} 次读取失败: {e}")
                self._axis_poll_reported = True
            self._bump_err()

    def _do_poll_io(self) -> None:
        try:
            di, do = self._mb.read_io()
            self.io_status.emit(di, do)
            self._reset_err()
        except ModbusError:
            self._bump_err()

    def _do_poll_diag(self) -> None:
        try:
            diag = self._mb.read_diag()
            self.comm_diag.emit(diag)
            self._reset_err()
        except ModbusError:
            pass  # 不计入断线判定

    def _do_heartbeat(self) -> None:
        try:
            self._mb.heartbeat()
            self._reset_err()
        except ModbusError:
            pass

    def _do_poll_mode(self) -> None:
        try:
            mode = self._mb.read_op_mode()
            if mode != self._prev_mode:
                self._prev_mode = mode
                self.op_mode_changed.emit(mode)
        except ModbusError:
            pass


# ============================================================================
#  便捷封装: 创建 + 启动 QThread
# ============================================================================

def start_worker(mb: ModbusMaster, cfg: Optional[WorkerConfig] = None) -> tuple:
    """创建 worker + thread, 返回 (worker, thread)。线程已启动。

    调用方需在断开时:
        worker.stop()
        thread.quit()
        thread.wait()
    """
    thread = QThread()
    worker = CommWorker(mb, cfg)
    worker.moveToThread(thread)
    thread.started.connect(worker.run)
    worker.disconnected.connect(thread.quit)
    thread.start()
    return worker, thread
