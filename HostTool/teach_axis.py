#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
teach_axis.py — 2416_F407 在线示教 / 调控轴上位机 v1.0
========================================================
功能聚焦
    · 串口连接管理 (端口扫描 / 9600-115200 / 从站地址 / 心跳保活)
    · 八轴在线 JOG 调试 (连续按住 / 步进点动 / 速度可调)
    · 绝对 / 相对定位 (一次写入 speed+target+cmd, FC16)
    · 回原 / 设零 / 软限位 / 急停
    · 实时位置 / 状态 / 软限位 / I/O 监控

    说明: 固件已删除轴报警体系。硬限位只服务回原, 行程保护使用可选软限位;
          定位点的坐标、名称和速度全部由上位机写入并掉电保持。
    · 示教点记录 (记录全部轴当前位置, 可命名 / 移动 / 删除 / 保存)
    · 通信诊断 (帧统计 / WDT 状态)

    固件依赖
    Modbus RTU 115200 8N1, 寄存器映射见 Firmware/Core/Src/app/debug/motor_debug_service.c。
"""

from __future__ import annotations

import json
import os
import sys
import time
from dataclasses import asdict, dataclass, field
from typing import List, Optional

import serial
import serial.tools.list_ports

from PySide6.QtCore import QEvent, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPalette
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, QFrame,
    QGraphicsDropShadowEffect, QGridLayout, QGroupBox, QHBoxLayout, QHeaderView, QInputDialog, QLabel,
    QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton,
    QScrollArea, QSizePolicy, QSpinBox, QSplitter, QStatusBar, QTableWidget,
    QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget, QStackedWidget,
)

import modbus_master as mm
from comm_worker import CommWorker, WorkerConfig, start_worker
from modbus_master import (
    AxisStatus, CommDiag, MODE_NORMAL, MODE_DEBUG, MODE_NAMES,
    ModbusError, ModbusMaster, NUM_AXES, NUM_DI, NUM_DO, NUM_PWM,
    STATE_NAMES, STATE_IDLE,
    POINT_SPEED_MIN, POINT_SPEED_MAX, POINT_SPEED_DEFAULT,
)

# 引脚名称常量: 必须与 Firmware/Core/Inc/IODEF.h 保持一致。
X_NAMES = [
    "X0 (PA0)", "X1 (PA1)", "X2 (PB15)", "X3 (PA15)",
    "X4 (PB3)", "X5 (PC4)", "X6 (PD6)", "X7 (PD5)",
    "X10 (PD10)", "X11 (PC8)", "X12 (PC9)", "X13 (PA12)",
    "X14 (PA11)", "X15 (PD15)", "X16 (PD14)", "X17 (PD13)",
    "X20 (PD12)", "X21 (PD11)", "X22 (PE15)", "X23 (PE14)",
    "X24 (PE13)", "X25 (PE12)", "X26 (PE11)", "X27 (PE10)",
]

Y_NAMES = [
    "Y0 (PA6)", "Y1 (PC6)", "Y2 (PC7)", "Y3 (PA8)",
    "Y4 (PB7)", "Y5 (PB8)", "Y6 (PD7)", "Y7 (PD4)",
    "Y10 (PC13)", "Y11 (PE2)", "Y12 (PE1)", "Y13 (PE0)",
    "Y14 (PB9)", "Y15 (PB6)", "Y16 (PB5)", "Y17 (PB4)",
]

PWM_CH_NAMES = [
    "CH0 (Y1/PC6/TIM8_CH1)", "CH1 (Y3/PA8/TIM1_CH1)",
    "CH2 (Y4/PB7/TIM4_CH2)", "CH3 (Y16/PB5/TIM3_CH2)",
]
PWM_TO_Y = {0: 1, 1: 3, 2: 4, 3: 14}
SOFT_PWM_CH_NAMES = [
    "CH0 (Y6/PD7/TIM5软件)", "CH1 (Y10/PC13/TIM5软件)",
    "CH2 (Y12/PE1/TIM5软件)", "CH3 (Y14/PB9/TIM5软件)",
]
SOFT_PWM_TO_Y = {0: 6, 1: 8, 2: 10, 3: 12}

AXIS_COLORS = [
    "#2563eb", "#d97706", "#059669", "#7c3aed",
    "#dc2626", "#0891b2", "#4f46e5", "#65a30d",
]
AXIS_BADGES = ["❶", "❷", "❸", "❹", "❺", "❻", "❼", "❽"]


APP_VERSION = "1.0"
APP_TITLE = f"2416_F407 在线示教调控轴 v{APP_VERSION}"
EXPECTED_FW_VERSION = 0x2416

DEFAULT_AXIS_NAMES = [
    "轴0 (X)", "轴1 (Y)", "轴2 (Z)", "轴3 (U)",
    "轴4 (V)", "轴5 (W)", "轴6 (A)", "轴7 (B)",
]
DEFAULT_POINT_NAMES = ["初始位", "放料位", "取料位", "待机位", "位5", "位6", "位7", "位8"]
TEACH_FILE_DEFAULT = "teach_points.json"
CONFIG_FILE = "last_config.json"


def app_base_dir() -> str:
    """返回配置/示教文件的基准目录。

    PyInstaller onefile 打包后 __file__ 指向临时解包目录 (_MEIxxxx),
    写在那里的文件退出即丢失, 因此冻结时改用 EXE 所在目录。
    """
    if getattr(sys, "frozen", False):
        return os.path.dirname(os.path.abspath(sys.executable))
    return os.path.dirname(os.path.abspath(__file__))


def app_data_path(filename: str) -> str:
    """把相对文件名解析为基准目录下的绝对路径。"""
    return os.path.join(app_base_dir(), filename)


def build_default_axis_point_names() -> List[List[str]]:
    return [list(DEFAULT_POINT_NAMES) for _ in range(NUM_AXES)]


# ============================================================================
#  样式表 — 简洁明亮
# ============================================================================

QSS = """
QMainWindow, QWidget { background: #edf2f7; font-family: 'Microsoft YaHei', 'Segoe UI', sans-serif; font-size: 13px; color: #1f2937; }
QGroupBox { border: 1px solid #cbd5e1; border-radius: 7px; margin-top: 12px; padding-top: 12px; padding-bottom: 8px; background: #ffffff; font-weight: 600; }
QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 6px; color: #334155; }
QPushButton { background: #ffffff; border: 1px solid #cbd5e1; border-radius: 6px; padding: 6px 12px; min-height: 28px; color: #334155; font-weight: 500; }
QPushButton:hover { background: #f8fafc; border-color: #94a3b8; color: #0f172a; }
QPushButton:pressed { background: #f1f5f9; }
QPushButton:disabled { background: #f3f4f6; color: #9ca3af; border-color: #e5e7eb; }
QPushButton#btnEStop { background: #dc2626; border: none; color: #ffffff; font-size: 15px; font-weight: bold; min-height: 44px; min-width: 152px; padding: 10px 22px; border-radius: 8px; }
QPushButton#btnEStop:hover { background: #b91c1c; }
QPushButton#btnEStop:pressed { background: #991b1b; }
QPushButton#btnPrimary { background: #2563eb; color: white; border: 1px solid #1d4ed8; font-weight: 600; }
QPushButton#btnPrimary:hover { background: #1d4ed8; }
QPushButton#btnSuccess { background: #15803d; color: white; border: 1px solid #166534; font-weight: 600; }
QPushButton#btnSuccess:hover { background: #166534; }
QPushButton#btnDanger { background: #dc2626; color: white; border: 1px solid #b91c1c; font-weight: 600; }
QPushButton#btnDanger:hover { background: #b91c1c; }
QPushButton#btnWarn { background: #d97706; color: white; border: 1px solid #b45309; font-weight: 600; }
QPushButton#btnWarn:hover { background: #b45309; }
QPushButton#btnJogNeg { background: #f1f5f9; color: #0f172a; border: 1px solid #94a3b8; font-size: 14px; font-weight: 600; min-height: 40px; }
QPushButton#btnJogNeg:hover { background: #e2e8f0; border-color: #64748b; }
QPushButton#btnJogPos { background: #f1f5f9; color: #0f172a; border: 1px solid #94a3b8; font-size: 14px; font-weight: 600; min-height: 40px; }
QPushButton#btnJogPos:hover { background: #e2e8f0; border-color: #64748b; }
QPushButton#btnStep { min-height: 34px; font-weight: 500; }
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox { background: #ffffff; border: 1.5px solid #d1d5db; border-radius: 6px; padding: 4px 8px; min-height: 22px; }
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus { border-color: #3b82f6; }
QLabel#lblPos { font-family: 'Cascadia Code', 'Consolas', monospace; font-size: 26px; font-weight: 700; color: #0f172a; padding: 10px 12px; background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 8px; }
QLabel#lblState { padding: 4px 12px; border-radius: 12px; color: white; font-weight: 600; }
QLabel#cfgValue { background: #f8fafc; border: 1.5px solid #dbe3ef; border-radius: 6px; padding: 6px 10px; color: #0f172a; font-weight: 600; min-height: 24px; }
QLabel#cfgMuted { background: #f8fafc; border: 1.5px dashed #dbe3ef; border-radius: 6px; padding: 6px 10px; color: #64748b; font-weight: 600; min-height: 24px; }
QLabel#lblConn { font-weight: bold; padding: 2px 10px; border-radius: 10px; }
QLabel#lblConnOK { background: #dcfce7; color: #166534; }
QLabel#lblConnErr { background: #fee2e2; color: #991b1b; }
QFrame#card, QFrame#card_axis0, QFrame#card_axis1, QFrame#card_axis2, QFrame#card_axis3, QFrame#card_axis4, QFrame#card_axis5 { background: #ffffff; border: 1px solid #e2e8f0; border-radius: 10px; }
QFrame#card_axis0 { border-left: 4px solid #3b82f6; }
QFrame#card_axis1 { border-left: 4px solid #f59e0b; }
QFrame#card_axis2 { border-left: 4px solid #10b981; }
QFrame#card_axis3 { border-left: 4px solid #8b5cf6; }
QFrame#card_axis4 { border-left: 4px solid #ef4444; }
QFrame#card_axis5 { border-left: 4px solid #0ea5e9; }
QFrame#topBar { background: #ffffff; border: 1px solid #e2e8f0; border-radius: 10px; }
QWidget#systemPanel { background: transparent; }
QGroupBox#homeGroup { border-top: 3px solid #2563eb; }
QGroupBox#wdtGroup { border-top: 3px solid #0f766e; }
QGroupBox#firmwareGroup { border-top: 3px solid #d97706; }
QFrame#axisNav { background: #ffffff; border: 1px solid #dbe3ef; border-radius: 8px; }
QFrame#axisNav[selected="true"] { background: #eff6ff; border: 2px solid #3b82f6; }
QFrame#axisNav[available="false"] { background: #f8fafc; color: #94a3b8; border: 1px dashed #cbd5e1; }
QScrollArea#axisScroll { border: none; background: transparent; }
QScrollArea#axisScroll > QWidget > QWidget { background: transparent; }
QTableWidget { background: white; border: 1.5px solid #d1d5db; border-radius: 6px; gridline-color: #e5e7eb; alternate-background-color: #f9fafb; selection-background-color: #dbeafe; selection-color: #1f2937; }
QHeaderView::section { background: #f3f4f6; border: none; border-right: 1px solid #e5e7eb; border-bottom: 1.5px solid #d1d5db; padding: 6px; font-weight: 600; color: #4b5563; }
QPlainTextEdit { background: #1f2937; color: #d1d5db; border: 1.5px solid #374151; border-radius: 6px; font-family: 'Cascadia Code', 'Consolas', monospace; font-size: 12px; padding: 6px; }
QStatusBar { background: #f3f4f6; border-top: 1.5px solid #d1d5db; }
QTabWidget::pane { border: 1.5px solid #d1d5db; border-radius: 8px; background: #ffffff; }
QTabBar::tab { background: #e5e7eb; padding: 8px 18px; border: 1.5px solid #d1d5db; border-bottom: none; border-top-left-radius: 8px; border-top-right-radius: 8px; margin-right: 2px; color: #6b7280; }
QTabBar::tab:selected { background: white; color: #1d4ed8; font-weight: bold; border-bottom: 2px solid #3b82f6; }
QCheckBox::indicator { width: 18px; height: 18px; border: 1.5px solid #d1d5db; border-radius: 4px; background: white; }
QCheckBox::indicator:checked { background: #3b82f6; border-color: #3b82f6; }
"""


# ============================================================================
#  数据模型
# ============================================================================

class NoWheelSpinBox(QSpinBox):
    """只能键盘/按钮修改，忽略鼠标滚轮，避免误调参数。"""

    def wheelEvent(self, event):
        event.ignore()


class NoWheelComboBox(QComboBox):
    """下拉框忽略鼠标滚轮，避免焦点停留时误切换选项。"""

    def wheelEvent(self, event):
        event.ignore()

@dataclass
class TeachPoint:
    name: str
    positions: List[int]                  # 各轴目标位置
    speed: int = 1250                      # 移动速度
    note: str = ""

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> "TeachPoint":
        return cls(
            name=str(d.get("name", "")),
            positions=[int(x) for x in d.get("positions", [])],
            speed=int(d.get("speed", 1250)),
            note=str(d.get("note", "")),
        )


# ============================================================================
#  单轴控制卡
# ============================================================================

class AxisCard(QFrame):
    """单轴控制卡: 位置显示 + JOG + 定位 + 状态。"""

    name_changed = Signal(int, str)  # axis_idx, new_name

    def __init__(self, axis_idx: int, win: "MainWindow"):
        super().__init__()
        self.setObjectName(f"card_axis{axis_idx}")
        self.idx = axis_idx
        self.win = win
        self._axis_name = DEFAULT_AXIS_NAMES[axis_idx]
        self._build()
        self._update_status(AxisStatus())

    @property
    def axis_name(self) -> str:
        return self._axis_name

    def set_axis_name(self, name: str) -> None:
        self._axis_name = name
        self.lbl_name.setText(f"{AXIS_BADGES[self.idx]} {name}")

    def set_axis_available(self, available: bool) -> None:
        self.setVisible(available)
        self.setEnabled(available)

    # ---- UI 构建 ----

    def _build(self) -> None:
        outer = QVBoxLayout(self)
        outer.setSpacing(10)
        outer.setContentsMargins(10, 10, 10, 10)

        # 标题分两行，避免窄屏下名称和状态互相挤压。
        title_row = QHBoxLayout()
        title_row.setSpacing(8)

        # 轴名称 (可双击编辑)
        color = AXIS_COLORS[self.idx]
        self.lbl_name = QLabel(f"{AXIS_BADGES[self.idx]} {DEFAULT_AXIS_NAMES[self.idx]}")
        f = QFont(); f.setBold(True); f.setPointSize(13)
        self.lbl_name.setFont(f)
        self.lbl_name.setFixedHeight(28)
        self.lbl_name.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        self.lbl_name.setStyleSheet(f"color: {color};")
        self.lbl_name.setCursor(Qt.PointingHandCursor)
        self.lbl_name.setToolTip("双击修改轴名称")
        self.lbl_name.mouseDoubleClickEvent = self._on_edit_name
        title_row.addWidget(self.lbl_name)

        # 轴序号标签
        lbl_idx = QLabel(f"轴{self.idx}")
        lbl_idx.setFixedHeight(28)
        lbl_idx.setAlignment(Qt.AlignCenter)
        lbl_idx.setStyleSheet(
            f"background: {color}; color: white; padding: 0 10px; "
            f"border-radius: 10px; font-size: 11px; font-weight: bold;")
        title_row.addWidget(lbl_idx)

        title_row.addStretch()
        outer.addLayout(title_row)

        status_row = QHBoxLayout()
        status_row.setSpacing(8)
        status_row.addStretch()
        self.lbl_state = QLabel("空闲")
        self.lbl_state.setObjectName("lblState")
        self.lbl_state.setFixedHeight(28)
        self.lbl_state.setMinimumWidth(58)
        self.lbl_state.setAlignment(Qt.AlignCenter)
        self.lbl_state.setStyleSheet("background: #64748b;")
        status_row.addWidget(self.lbl_state)

        self.lbl_homed = QLabel("未回原")
        self.lbl_homed.setObjectName("lblState")
        self.lbl_homed.setFixedHeight(28)
        self.lbl_homed.setMinimumWidth(66)
        self.lbl_homed.setAlignment(Qt.AlignCenter)
        self.lbl_homed.setStyleSheet("background: #ca8a04;")
        status_row.addWidget(self.lbl_homed)
        outer.addLayout(status_row)

        # 位置显示
        self.lbl_pos = QLabel("0")
        self.lbl_pos.setObjectName("lblPos")
        self.lbl_pos.setAlignment(Qt.AlignCenter)
        outer.addWidget(self.lbl_pos)

        # ---- JOG 区 ----
        jog_grp = QGroupBox("点动 (JOG)")
        jog_lay = QVBoxLayout(jog_grp)
        jog_lay.setSpacing(10)

        # 速度与步进量同一行，标签对齐
        param_row = QHBoxLayout()
        param_row.setSpacing(8)
        sp_l = QVBoxLayout()
        sp_l.addWidget(QLabel("速度 (Hz)"))
        self.spn_speed = NoWheelSpinBox()
        self.spn_speed.setRange(100, 100000)
        self.spn_speed.setValue(1250)
        self.spn_speed.setSingleStep(100)
        self.spn_speed.setMinimumWidth(100)
        sp_l.addWidget(self.spn_speed)
        param_row.addLayout(sp_l)

        st_l = QVBoxLayout()
        st_l.addWidget(QLabel("步进量"))
        self.spn_step = NoWheelSpinBox()
        self.spn_step.setRange(1, 1000000)
        self.spn_step.setValue(100)
        self.spn_step.setSingleStep(50)
        self.spn_step.setMinimumWidth(100)
        st_l.addWidget(self.spn_step)
        param_row.addLayout(st_l)
        param_row.addStretch()
        jog_lay.addLayout(param_row)

        jog_hint = QLabel("连续点动（按住）")
        jog_hint.setStyleSheet("color: #64748b; font-size: 11px; font-weight: 600;")
        jog_lay.addWidget(jog_hint)

        # 连续 JOG (按住) — 三等分
        cont_row = QHBoxLayout()
        cont_row.setSpacing(8)
        self.btn_jog_neg = QPushButton("← 负向")
        self.btn_jog_neg.setObjectName("btnJogNeg")
        self.btn_jog_neg.pressed.connect(lambda: self.win.cmd_jog(self.idx, False))
        self.btn_jog_neg.released.connect(lambda: self.win.cmd_stop(self.idx))
        cont_row.addWidget(self.btn_jog_neg, 1)

        self.btn_stop = QPushButton("停止")
        self.btn_stop.setObjectName("btnDanger")
        self.btn_stop.clicked.connect(lambda: self.win.cmd_stop(self.idx))
        cont_row.addWidget(self.btn_stop, 1)

        self.btn_jog_pos = QPushButton("正向 →")
        self.btn_jog_pos.setObjectName("btnJogPos")
        self.btn_jog_pos.pressed.connect(lambda: self.win.cmd_jog(self.idx, True))
        self.btn_jog_pos.released.connect(lambda: self.win.cmd_stop(self.idx))
        cont_row.addWidget(self.btn_jog_pos, 1)
        jog_lay.addLayout(cont_row)

        step_hint = QLabel("单步点动（单击）")
        step_hint.setStyleSheet("color: #64748b; font-size: 11px; font-weight: 600;")
        jog_lay.addWidget(step_hint)

        step_row = QHBoxLayout()
        step_row.setSpacing(8)
        self.btn_step_neg = QPushButton("− 步进")
        self.btn_step_neg.setObjectName("btnStep")
        self.btn_step_neg.clicked.connect(
            lambda: self.win.cmd_move_rel(self.idx, -self.spn_step.value(),
                                          self.spn_speed.value()))
        step_row.addWidget(self.btn_step_neg, 1)
        self.btn_step_pos = QPushButton("步进 +")
        self.btn_step_pos.setObjectName("btnStep")
        self.btn_step_pos.clicked.connect(
            lambda: self.win.cmd_move_rel(self.idx, self.spn_step.value(),
                                          self.spn_speed.value()))
        step_row.addWidget(self.btn_step_pos, 1)
        jog_lay.addLayout(step_row)

        outer.addWidget(jog_grp)

        # ---- 定位区（纵向分组，避免窄屏挤压）----
        pos_grp = QGroupBox("定位 / 坐标")
        pos_lay = QVBoxLayout(pos_grp)
        pos_lay.setSpacing(8)

        tgt_lab = QLabel("目标位置")
        tgt_lab.setStyleSheet("font-weight: 600; color: #475569;")
        pos_lay.addWidget(tgt_lab)
        self.spn_target = NoWheelSpinBox()
        self.spn_target.setRange(-10000000, 10000000)
        self.spn_target.setValue(0)
        self.spn_target.setSingleStep(100)
        pos_lay.addWidget(self.spn_target)

        row_abs_rel = QHBoxLayout()
        row_abs_rel.setSpacing(8)
        self.btn_move_abs = QPushButton("绝对定位")
        self.btn_move_abs.setObjectName("btnPrimary")
        self.btn_move_abs.clicked.connect(
            lambda: self.win.cmd_move_abs(self.idx, self.spn_target.value(),
                                          self.spn_speed.value()))
        row_abs_rel.addWidget(self.btn_move_abs, 1)

        self.btn_move_rel = QPushButton("相对定位")
        self.btn_move_rel.clicked.connect(
            lambda: self.win.cmd_move_rel(self.idx, self.spn_target.value(),
                                          self.spn_speed.value()))
        row_abs_rel.addWidget(self.btn_move_rel, 1)
        pos_lay.addLayout(row_abs_rel)

        sep = QFrame()
        sep.setFixedHeight(1)
        sep.setStyleSheet("background: #e2e8f0; border: none;")
        pos_lay.addWidget(sep)

        set_lab = QLabel("写入当前位置（机械坐标）")
        set_lab.setStyleSheet("font-weight: 600; color: #475569;")
        pos_lay.addWidget(set_lab)
        self.spn_setpos = NoWheelSpinBox()
        self.spn_setpos.setRange(-10000000, 10000000)
        self.spn_setpos.setValue(0)
        pos_lay.addWidget(self.spn_setpos)

        row_set_zero = QHBoxLayout()
        row_set_zero.setSpacing(8)
        self.btn_set_pos = QPushButton("写入数值")
        self.btn_set_pos.clicked.connect(
            lambda: self.win.cmd_set_pos(self.idx, self.spn_setpos.value()))
        row_set_zero.addWidget(self.btn_set_pos, 1)

        self.btn_zero = QPushButton("当前位置置零")
        self.btn_zero.setObjectName("btnSuccess")
        self.btn_zero.clicked.connect(lambda: self.win.cmd_set_pos(self.idx, 0))
        row_set_zero.addWidget(self.btn_zero, 1)
        pos_lay.addLayout(row_set_zero)

        outer.addWidget(pos_grp)

        # ---- 工具区 ----
        tool_row = QHBoxLayout()
        tool_row.setSpacing(8)
        self.btn_home = QPushButton("回原点")
        self.btn_home.setObjectName("btnWarn")
        self.btn_home.clicked.connect(lambda: self.win.cmd_home(self.idx))
        tool_row.addWidget(self.btn_home, 1)

        self.btn_resume = QPushButton("继续剩余")
        self.btn_resume.clicked.connect(lambda: self.win.cmd_resume_remaining(self.idx))
        tool_row.addWidget(self.btn_resume, 1)
        outer.addLayout(tool_row)

    # ---- 状态刷新 ----

    def _update_status(self, st: AxisStatus) -> None:
        self.lbl_pos.setText(f"{st.position:+d}")

        # 状态色
        state_colors = {
            STATE_IDLE: "#6b7280",
            1: "#f59e0b",   # JOG
            2: "#3b82f6",   # MOVING
            3: "#8b5cf6",   # HOMING
        }
        color = state_colors.get(st.state, "#6b7280")
        self.lbl_state.setText(STATE_NAMES.get(st.state, f"状态{st.state}"))
        self.lbl_state.setStyleSheet(f"background: {color};")

        if st.homed:
            self.lbl_homed.setText("已回原")
            self.lbl_homed.setStyleSheet("background: #16a34a;")
        else:
            self.lbl_homed.setText("未回原")
            self.lbl_homed.setStyleSheet("background: #f59e0b;")

    def _on_edit_name(self, event=None) -> None:
        name, ok = QInputDialog.getText(
            self, "修改轴名称", f"轴{self.idx} 新名称:",
            text=self._axis_name)
        if ok and name.strip():
            self._axis_name = name.strip()
            self.lbl_name.setText(f"{AXIS_BADGES[self.idx]} {self._axis_name}")
            self.name_changed.emit(self.idx, self._axis_name)

    def get_position(self) -> int:
        # lbl 显示文本可能含正号, 解析回 int
        try:
            return int(self.lbl_pos.text().replace('+', ''))
        except ValueError:
            return 0

    def get_speed(self) -> int:
        return self.spn_speed.value()


class AxisNavCard(QFrame):
    """左侧轴选择卡，只显示固件实际注册轴。"""

    clicked = Signal(int)

    def __init__(self, axis_idx: int):
        super().__init__()
        self.idx = axis_idx
        self._selected = False
        self._available = True
        self.setObjectName("axisNav")
        self.setCursor(Qt.PointingHandCursor)
        self.setMinimumHeight(66)
        self.setMinimumWidth(92)
        self._build()
        self.set_selected(False)

    def _build(self) -> None:
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 8, 12, 8)
        lay.setSpacing(4)
        self.lbl_name = QLabel(f"{AXIS_BADGES[self.idx]} {DEFAULT_AXIS_NAMES[self.idx]}")
        f = QFont(); f.setBold(True); f.setPointSize(12)
        self.lbl_name.setFont(f)
        self.lbl_name.setStyleSheet(f"color: {AXIS_COLORS[self.idx]};")
        lay.addWidget(self.lbl_name)
        self.lbl_status = QLabel(f"轴{self.idx}  已注册")
        self.lbl_status.setStyleSheet("color: #64748b; font-size: 12px;")
        lay.addWidget(self.lbl_status)

    def mousePressEvent(self, event) -> None:
        if self._available and event.button() == Qt.LeftButton:
            self.clicked.emit(self.idx)
        super().mousePressEvent(event)

    def set_axis_name(self, name: str) -> None:
        self.lbl_name.setText(f"{AXIS_BADGES[self.idx]} {name}")

    def set_selected(self, selected: bool) -> None:
        self._selected = selected
        self.setProperty("selected", selected)
        self.style().unpolish(self)
        self.style().polish(self)

    def set_axis_available(self, available: bool) -> None:
        self._available = available
        self.setVisible(available)
        self.setEnabled(available)
        self.lbl_status.setText(f"轴{self.idx}  {'已注册' if available else '未注册'}")
        self.setProperty("available", available)
        self.style().unpolish(self)
        self.style().polish(self)


# ============================================================================
#  示教点面板
# ============================================================================

class TeachPanel(QWidget):
    def __init__(self, win: "MainWindow"):
        super().__init__()
        self.win = win
        self.points: List[TeachPoint] = []
        self._build()

    def _build(self) -> None:
        v = QVBoxLayout(self)
        v.setSpacing(8)
        v.setContentsMargins(8, 8, 8, 8)

        # 工具栏（两行：主要操作 / 编辑与文件）
        tb1 = QHBoxLayout()
        tb1.setSpacing(8)
        self.btn_record = QPushButton("记录当前位置")
        self.btn_record.setObjectName("btnSuccess")
        self.btn_record.clicked.connect(self._on_record)
        tb1.addWidget(self.btn_record)

        self.btn_goto = QPushButton("移动到选中点")
        self.btn_goto.setObjectName("btnPrimary")
        self.btn_goto.clicked.connect(self._on_goto)
        tb1.addWidget(self.btn_goto)
        tb1.addStretch()
        v.addLayout(tb1)

        tb2 = QHBoxLayout()
        tb2.setSpacing(8)
        self.btn_del = QPushButton("删除选中")
        self.btn_del.clicked.connect(self._on_delete)
        tb2.addWidget(self.btn_del)

        self.btn_clear = QPushButton("清空全部")
        self.btn_clear.clicked.connect(self._on_clear)
        tb2.addWidget(self.btn_clear)

        tb2.addStretch()

        self.btn_save = QPushButton("保存 JSON")
        self.btn_save.clicked.connect(self._on_save)
        tb2.addWidget(self.btn_save)

        self.btn_load = QPushButton("加载 JSON")
        self.btn_load.clicked.connect(self._on_load)
        tb2.addWidget(self.btn_load)

        self.btn_export = QPushButton("导出 CSV")
        self.btn_export.clicked.connect(self._on_export_csv)
        tb2.addWidget(self.btn_export)
        v.addLayout(tb2)

        # 表格
        self.tbl = QTableWidget()
        self._reset_headers()
        self.tbl.setAlternatingRowColors(True)
        self.tbl.setSelectionBehavior(QTableWidget.SelectRows)
        self.tbl.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.tbl.horizontalHeader().setStretchLastSection(True)
        self.tbl.setEditTriggers(QTableWidget.DoubleClicked | QTableWidget.EditKeyPressed)
        self.tbl.itemChanged.connect(self._on_item_changed)
        v.addWidget(self.tbl)

    # ---- 持久化 ----

    def serialize(self) -> List[dict]:
        return [p.to_dict() for p in self.points]

    def load_from(self, items: List[dict]) -> None:
        self.points = [TeachPoint.from_dict(d) for d in items]
        self._refresh_table()

    # ---- 操作 ----

    def _on_record(self) -> None:
        axis_count = self.win.active_axis_count()
        if axis_count == 0:
            self.win.log("当前没有已注册轴，无法记录当前位置")
            return
        positions = [self.win.axis_card(i).get_position() for i in range(axis_count)]
        speed = self.win.axis_card(0).get_speed()
        idx = len(self.points) + 1
        p = TeachPoint(name=f"P{idx:02d}", positions=positions, speed=speed)
        self.points.append(p)
        self._refresh_table()
        self.win.log(f"记录示教点 {p.name}: {positions} @ {speed}Hz")

    def set_axis_count(self, count: int) -> None:
        has_registered_axis = int(count) > 0
        self.btn_record.setEnabled(has_registered_axis)
        self.btn_goto.setEnabled(has_registered_axis)
        hint = "" if has_registered_axis else "当前没有已注册轴"
        self.btn_record.setToolTip(hint)
        self.btn_goto.setToolTip(hint)

    def _on_goto(self) -> None:
        if self.win.active_axis_count() == 0:
            self.win.log("当前没有已注册轴，无法执行示教点运动")
            return
        rows = sorted({i.row() for i in self.tbl.selectedItems()})
        if not rows:
            self.win.log("未选中任何示教点")
            return
        row = rows[0]
        if not (0 <= row < len(self.points)):
            return
        p = self.points[row]
        for i, target in enumerate(p.positions[:self.win.active_axis_count()]):
            if i < self.win.active_axis_count():
                self.win.cmd_move_abs(i, target, p.speed)
        self.win.log(f"移动到示教点 {p.name}: {p.positions}")

    def _on_delete(self) -> None:
        rows = sorted({i.row() for i in self.tbl.selectedItems()}, reverse=True)
        for r in rows:
            if 0 <= r < len(self.points):
                self.points.pop(r)
        self._refresh_table()

    def _on_clear(self) -> None:
        if not self.points:
            return
        ret = QMessageBox.question(self, "确认", "清空全部示教点?",
                                   QMessageBox.Yes | QMessageBox.No)
        if ret == QMessageBox.Yes:
            self.points.clear()
            self._refresh_table()

    def _on_save(self) -> None:
        fname, _ = QFileDialog.getSaveFileName(
            self, "保存示教点", app_data_path(TEACH_FILE_DEFAULT), "JSON (*.json)")
        if not fname:
            return
        with open(fname, 'w', encoding='utf-8') as f:
            json.dump(self.serialize(), f, ensure_ascii=False, indent=2)
        self.win.log(f"已保存 {len(self.points)} 个示教点到 {fname}")

    def _on_load(self) -> None:
        fname, _ = QFileDialog.getOpenFileName(
            self, "加载示教点", "", "JSON (*.json)")
        if not fname:
            return
        try:
            with open(fname, 'r', encoding='utf-8') as f:
                data = json.load(f)
            self.load_from(data)
            self.win.log(f"从 {fname} 加载 {len(self.points)} 个示教点")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"加载失败: {e}")

    def _on_export_csv(self) -> None:
        if not self.points:
            QMessageBox.information(self, "提示", "无示教点可导出")
            return
        fname, _ = QFileDialog.getSaveFileName(
            self, "导出 CSV", "teach_points.csv", "CSV (*.csv)")
        if not fname:
            return
        import csv
        with open(fname, 'w', encoding='utf-8-sig', newline='') as f:
            w = csv.writer(f)
            axis_count = self.win.active_axis_count()
            names = [self.win.axis_card(i).axis_name for i in range(axis_count)]
            header = ["序号", "名称"] + [f"{names[i]}位置" for i in range(axis_count)] + ["速度", "备注"]
            w.writerow(header)
            for i, p in enumerate(self.points, 1):
                row = [i, p.name] + list(p.positions[:axis_count]) + [p.speed, p.note]
                w.writerow(row)
        self.win.log(f"已导出 CSV: {fname}")

    # ---- 表格 ----

    def _refresh_table(self) -> None:
        self._reset_headers()
        axis_count = self.win.active_axis_count()
        self.tbl.blockSignals(True)
        self.tbl.setRowCount(len(self.points))
        for r, p in enumerate(self.points):
            self.tbl.setItem(r, 0, _ro_item(str(r + 1)))
            self.tbl.setItem(r, 1, QTableWidgetItem(p.name))
            for i in range(axis_count):
                pos = p.positions[i] if i < len(p.positions) else 0
                self.tbl.setItem(r, 2 + i, _ro_item(f"{pos:+d}"))
            self.tbl.setItem(r, 2 + axis_count, QTableWidgetItem(str(p.speed)))
            self.tbl.setItem(r, 3 + axis_count, QTableWidgetItem(p.note))
        self.tbl.blockSignals(False)

    def _reset_headers(self) -> None:
        axis_count = self.win.active_axis_count()
        names = [self.win.axis_card(i).axis_name for i in range(axis_count)]
        cols = ["#", "名称"] + [f"{names[i]} 位置" for i in range(axis_count)] + ["速度", "备注"]
        self.tbl.setColumnCount(len(cols))
        self.tbl.setHorizontalHeaderLabels(cols)

    def _on_item_changed(self, item: QTableWidgetItem) -> None:
        r, c = item.row(), item.column()
        if not (0 <= r < len(self.points)):
            return
        p = self.points[r]
        text = item.text()
        axis_count = self.win.active_axis_count()
        try:
            if c == 1:
                p.name = text
            elif c == 2 + axis_count:         # speed
                p.speed = max(100, int(text))
                item.setText(str(p.speed))
            elif c == 3 + axis_count:         # note
                p.note = text
        except ValueError:
            self._refresh_table()


def _ro_item(text: str) -> QTableWidgetItem:
    it = QTableWidgetItem(text)
    it.setFlags(it.flags() & ~Qt.ItemIsEditable)
    return it


# ============================================================================
#  I/O 监控面板
# ============================================================================

class IOPanel(QWidget):
    def __init__(self, win: "MainWindow", compact: bool = False):
        super().__init__()
        self.win = win
        self._compact = compact
        self._reserved_outputs = set()
        self._op_mode = MODE_NORMAL
        self._build()

    def _build(self) -> None:
        outer = QVBoxLayout(self)
        outer.setSpacing(8 if self._compact else 10)
        outer.setContentsMargins(0, 0, 0, 0)

        # X 输入
        x_grp = QGroupBox("X 输入 (DI)" if self._compact else "X 输入 (DI, FC02 / 寄存器 300)")
        xg = QGridLayout(x_grp)
        xg.setSpacing(6)
        self.di_lbls: List[QLabel] = []
        x_names = [f"X{i//8}{i%8}" if i >= 8 else f"X{i}" for i in range(NUM_DI)]
        x_cols = 4 if self._compact else 8
        for i in range(NUM_DI):
            lbl = QLabel(f"{x_names[i]}: 0")
            lbl.setMinimumHeight(26)
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(_di_off_style())
            xg.addWidget(lbl, i // x_cols, i % x_cols)
            self.di_lbls.append(lbl)
        outer.addWidget(x_grp)

        # Y 输出
        y_grp = QGroupBox("Y 输出 (DO)" if self._compact else "Y 输出 (DO, FC01 / 寄存器 302)  — 双击切换")
        yg = QGridLayout(y_grp)
        yg.setSpacing(6)
        self.do_btns: List[QPushButton] = []
        y_names = [f"Y{i}" if i < 8 else f"Y1{i-8}" for i in range(NUM_DO)]
        y_cols = 3 if self._compact else 6
        for i in range(NUM_DO):
            btn = QPushButton(f"{y_names[i]}: OFF")
            btn.setCheckable(True)
            btn.setMinimumHeight(30)
            btn.setStyleSheet(_do_off_style())
            btn.clicked.connect(lambda checked, idx=i: self.win.cmd_write_coil(idx, checked))
            yg.addWidget(btn, i // y_cols, i % y_cols)
            self.do_btns.append(btn)
        outer.addWidget(y_grp)
        outer.addStretch()

    def set_reserved_outputs(self, axes: List[dict]) -> None:
        """记录轴复用输出；调试模式下允许在对应轴空闲时测试。"""
        reserved = set()
        for axis in axes:
            direction = axis.get('dir')
            if direction is not None:
                reserved.add(int(direction))
            if int(axis.get('pulse_mode', 0)) == 1:
                pulse_y = SOFT_PWM_TO_Y.get(axis.get('soft'))
            else:
                pulse_y = PWM_TO_Y.get(axis.get('pwm'))
            if pulse_y is not None:
                reserved.add(int(pulse_y))

        self._reserved_outputs = {
            index for index in reserved if 0 <= index < len(self.do_btns)
        }
        self._refresh_output_access()

    def set_op_mode(self, mode: int) -> None:
        self._op_mode = mode
        self._refresh_output_access()

    def _refresh_output_access(self) -> None:
        for index, btn in enumerate(self.do_btns):
            is_reserved = index in self._reserved_outputs
            btn.setEnabled(True)
            btn.setToolTip(
                "该输出被运动轴占用：仅调试模式且对应轴空闲时可手动切换；"
                "正常模式或轴运动期间固件会拒绝" if is_reserved else "")

    def update_io(self, di: List[bool], do: List[bool]) -> None:
        for i, on in enumerate(di):
            if i >= len(self.di_lbls):
                break
            base = self.di_lbls[i].text().split(":")[0]
            self.di_lbls[i].setText(f"{base}: {'1' if on else '0'}")
            self.di_lbls[i].setStyleSheet(_di_on_style() if on else _di_off_style())
        for i, on in enumerate(do):
            if i >= len(self.do_btns):
                break
            base = self.do_btns[i].text().split(":")[0]
            self.do_btns[i].blockSignals(True)
            self.do_btns[i].setChecked(on)
            self.do_btns[i].setText(f"{base}: {'ON' if on else 'OFF'}")
            self.do_btns[i].setStyleSheet(_do_on_style() if on else _do_off_style())
            self.do_btns[i].blockSignals(False)


def _di_off_style():
    return ("background: #f3f4f6; color: #6b7280; padding: 4px 6px;"
            "border: 1.5px solid #d1d5db; border-radius: 6px; font-weight: 500;")

def _di_on_style():
    return ("background: #dcfce7; color: #166534; padding: 4px 6px;"
            "border: 1.5px solid #16a34a; border-radius: 6px; font-weight: bold;")

def _do_off_style():
    return ("background: #f3f4f6; color: #6b7280; padding: 5px 8px;"
            "border: 1.5px solid #d1d5db; border-radius: 6px; font-weight: 600;")

def _do_on_style():
    return ("background: #fef3c7; color: #92400e; padding: 5px 8px;"
            "border: 2px solid #f59e0b; border-radius: 6px; font-weight: bold;")


# ============================================================================
#  轴配置面板 — 固件固定轴绑定说明与本地轴名称
# ============================================================================

class AxisConfigPanel(QWidget):
    """轴绑定由固件固定维护；上位机只维护本地轴名称。"""

    def __init__(self, win: "MainWindow"):
        super().__init__()
        self.win = win
        self._ax_widgets: list = []
        self._build()

    def _build(self) -> None:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        inner = QWidget()
        main_lay = QVBoxLayout(inner)
        main_lay.setSpacing(10)
        main_lay.setContentsMargins(8, 8, 8, 8)

        note = QLabel("轴绑定和传感器绑定由固件业务代码固定维护；本页从底层只读读取并显示，只有轴名称可本地编辑。")
        note.setStyleSheet("color: #1e40af; background: #eff6ff; border: 1px solid #bfdbfe; border-radius: 8px; padding: 10px 12px; font-weight: 600;")
        main_lay.addWidget(note)

        # ---- 每个轴的引脚配置 ----
        for i in range(NUM_AXES):
            color = AXIS_COLORS[i]
            g = QGroupBox(f"{AXIS_BADGES[i]} {DEFAULT_AXIS_NAMES[i]} 绑定配置")
            g.setStyleSheet(
                f"QGroupBox {{ border: 1.5px solid {color}; border-radius: 8px; "
                f"margin-top: 12px; padding-top: 12px; padding-bottom: 10px; "
                f"background: #ffffff; font-weight: 600; }}"
                f"QGroupBox::title {{ color: {color}; }}")
            grid = QGridLayout(g)
            grid.setSpacing(8)
            w = {}

            grid.addWidget(QLabel("轴名称:"), 0, 0)
            edt_name = QLineEdit()
            edt_name.setText(DEFAULT_AXIS_NAMES[i])
            edt_name.setFixedWidth(160)
            edt_name.setPlaceholderText("自定义轴名称")
            edt_name.textChanged.connect(
                lambda txt, idx=i: self._on_name_edited(idx, txt))
            grid.addWidget(edt_name, 0, 1)
            w['name'] = edt_name
            w['group'] = g

            grid.addWidget(QLabel("PWM通道:"), 0, 2)
            w['pwm'] = self._cfg_label("未读取", muted=True)
            grid.addWidget(w['pwm'], 0, 3)

            grid.addWidget(QLabel("方向输出:"), 1, 0)
            w['dir'] = self._cfg_label("未读取", muted=True)
            grid.addWidget(w['dir'], 1, 1)

            grid.addWidget(QLabel("原点传感器:"), 1, 2)
            w['home'] = self._cfg_label("无 (不使用)", muted=True)
            grid.addWidget(w['home'], 1, 3)

            grid.addWidget(QLabel("正极限传感器:"), 2, 0)
            w['limp'] = self._cfg_label("无 (不使用)", muted=True)
            grid.addWidget(w['limp'], 2, 1)

            grid.addWidget(QLabel("负极限传感器:"), 2, 2)
            w['limn'] = self._cfg_label("无 (不使用)", muted=True)
            grid.addWidget(w['limn'], 2, 3)

            grid.addWidget(QLabel("方向反转:"), 3, 0)
            w['inv'] = self._cfg_label("否", muted=True)
            grid.addWidget(w['inv'], 3, 1)

            grid.addWidget(QLabel("回原方向:"), 3, 2)
            w['home_dir'] = self._cfg_label("未读取", muted=True)
            grid.addWidget(w['home_dir'], 3, 3)

            grid.addWidget(QLabel("软件脉冲:"), 4, 0)
            w['soft'] = self._cfg_label("None", muted=True)
            grid.addWidget(w['soft'], 4, 1)

            grid.addWidget(QLabel("脉冲模式:"), 4, 2)
            w['pulse_mode'] = self._cfg_label("None", muted=True)
            grid.addWidget(w['pulse_mode'], 4, 3)

            # ---- 软限位 (可选行程保护, 掉电保持) ----
            grid.addWidget(QLabel("软限位:"), 5, 0)
            sl_row = QWidget()
            sl_lay = QHBoxLayout(sl_row)
            sl_lay.setContentsMargins(0, 0, 0, 0)
            sl_lay.setSpacing(6)
            sl_en = QCheckBox("启用")
            sl_en.setChecked(False)
            sl_en.setToolTip("默认关闭; 启用后超出区间的位置命令和 JOG 会被固件拒绝")
            sl_lay.addWidget(sl_en)
            sl_neg = NoWheelSpinBox()
            sl_neg.setRange(-2147483648, 2147483647)
            sl_neg.setValue(-100000)
            sl_neg.setSuffix(" 脉冲")
            sl_neg.setMinimumWidth(128)
            sl_neg.setToolTip("负向软限位 (脉冲)")
            sl_lay.addWidget(sl_neg)
            sl_pos = NoWheelSpinBox()
            sl_pos.setRange(-2147483648, 2147483647)
            sl_pos.setValue(100000)
            sl_pos.setSuffix(" 脉冲")
            sl_pos.setMinimumWidth(128)
            sl_pos.setToolTip("正向软限位 (脉冲)")
            sl_lay.addWidget(sl_pos)
            sl_btn_read = QPushButton("读取")
            sl_btn_read.clicked.connect(
                lambda _checked=False, idx=i: self._read_soft_limit(idx))
            sl_lay.addWidget(sl_btn_read)
            sl_btn_write = QPushButton("写入并保存")
            sl_btn_write.setObjectName("btnWarn")
            sl_btn_write.setToolTip("写入后立即保存到内部 Flash, 掉电保持")
            sl_btn_write.clicked.connect(
                lambda _checked=False, idx=i: self._write_soft_limit(idx))
            sl_lay.addWidget(sl_btn_write)
            sl_lbl = QLabel("未读取")
            sl_lbl.setStyleSheet("color: #667085; font-weight: 600;")
            sl_lay.addWidget(sl_lbl)
            sl_lay.addStretch()
            grid.addWidget(sl_row, 5, 1, 1, 3)
            w['sl_en'] = sl_en
            w['sl_neg'] = sl_neg
            w['sl_pos'] = sl_pos
            w['sl_status'] = sl_lbl

            self._ax_widgets.append(w)
            main_lay.addWidget(g)

        # ---- 按钮栏 ----
        bl = QHBoxLayout()
        bl.setSpacing(8)
        btn_read = QPushButton("读取底层配置")
        btn_read.setObjectName("btnPrimary")
        btn_read.clicked.connect(self._read_from_device)
        bl.addWidget(btn_read)

        btn_refresh_names = QPushButton("保存轴名称")
        btn_refresh_names.clicked.connect(self._save_local)
        bl.addWidget(btn_refresh_names)
        bl.addStretch()

        btn_load = QPushButton("加载轴名称")
        btn_load.clicked.connect(self._load_local)
        bl.addWidget(btn_load)

        main_lay.addLayout(bl)
        main_lay.addStretch()

        scroll.setWidget(inner)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

    def _on_name_edited(self, idx: int, name: str) -> None:
        """轴名称编辑时同步更新轴卡片。"""
        if idx < len(self._ax_widgets):
            self._ax_widgets[idx]['group'].setTitle(
                f"{AXIS_BADGES[idx]} {name or DEFAULT_AXIS_NAMES[idx]} 绑定配置")
        if idx in self.win._axis_cards_map:
            self.win._axis_cards_map[idx].set_axis_name(name or DEFAULT_AXIS_NAMES[idx])
            self.win._axis_cards_map[idx].name_changed.emit(
                idx, name or DEFAULT_AXIS_NAMES[idx])

    # ---- 软限位 (可选行程保护, 掉电保持) ----

    def _read_soft_limit(self, idx: int) -> None:
        if not self.win._worker:
            QMessageBox.warning(self, "未连接", "请先连接设备。")
            return
        if idx >= self.win.active_axis_count():
            self.win.log(f"轴{idx} 未注册, 无法读取软限位")
            return
        self._ax_widgets[idx]['sl_status'].setText("读取中...")
        self.win._enqueue(self.win._mb.read_soft_limit, idx,
                          tag=f"slread{idx}")

    def _write_soft_limit(self, idx: int) -> None:
        if not self.win._worker:
            QMessageBox.warning(self, "未连接", "请先连接设备。")
            return
        if idx >= self.win.active_axis_count():
            self.win.log(f"轴{idx} 未注册, 无法写入软限位")
            return
        if self.win._current_mode != MODE_DEBUG:
            QMessageBox.warning(
                self, "需要调试模式",
                "固件只在调试模式下允许写软限位。\n请先在设备侧切到调试模式。")
            return
        w = self._ax_widgets[idx]
        enable = w['sl_en'].isChecked()
        neg = int(w['sl_neg'].value())
        pos = int(w['sl_pos'].value())
        if enable and neg >= pos:
            QMessageBox.warning(
                self, "参数错误", "启用软限位时要求 负向限位 < 正向限位。")
            return
        ret = QMessageBox.question(
            self, "确认写入软限位",
            f"轴{idx}: 启用={int(enable)} 负向={neg} 正向={pos}\n"
            "写入后将保存到内部 Flash (掉电保持)，是否继续？",
            QMessageBox.Yes | QMessageBox.No)
        if ret != QMessageBox.Yes:
            return
        w['sl_status'].setText("写入中...")
        self.win._enqueue(self.win._mb.write_soft_limit, idx, enable, neg, pos,
                          tag=f"slwrite{idx}")

    def apply_soft_limit(self, idx: int, data: dict) -> None:
        if not (0 <= idx < len(self._ax_widgets)):
            return
        w = self._ax_widgets[idx]
        w['sl_en'].setChecked(bool(data.get('enable', False)))
        w['sl_neg'].setValue(int(data.get('min', 0)))
        w['sl_pos'].setValue(int(data.get('max', 0)))
        if data.get('valid'):
            w['sl_status'].setText("已启用且有效")
            w['sl_status'].setStyleSheet("color: #16a34a; font-weight: 600;")
        elif data.get('enable'):
            w['sl_status'].setText("已启用但范围无效")
            w['sl_status'].setStyleSheet("color: #dc2626; font-weight: 600;")
        else:
            w['sl_status'].setText("未启用")
            w['sl_status'].setStyleSheet("color: #667085; font-weight: 600;")

    def handle_command_result(self, tag: str, result) -> bool:
        if tag.startswith("slread"):
            idx = int(tag[len("slread"):])
            try:
                self.apply_soft_limit(idx, result)
            except Exception as exc:  # noqa: BLE001
                self._ax_widgets[idx]['sl_status'].setText(f"读取失败: {exc}")
            return True
        if tag.startswith("slwrite"):
            idx = int(tag[len("slwrite"):])
            w = self._ax_widgets[idx]
            w['sl_status'].setText("已写入并保存到 Flash")
            w['sl_status'].setStyleSheet("color: #16a34a; font-weight: 600;")
            return True
        return False

    def handle_command_finished(self, tag: str, ok: bool, message: str) -> bool:
        if not (tag.startswith("slread") or tag.startswith("slwrite")):
            return False
        if not ok:
            idx = int(tag[len("slread"):] if tag.startswith("slread")
                      else tag[len("slwrite"):])
            w = self._ax_widgets[idx]
            w['sl_status'].setText(f"失败: {message}")
            w['sl_status'].setStyleSheet("color: #dc2626; font-weight: 600;")
        return True

    def _write_config(self) -> None:
        """轴绑定配置只读，不允许从上位机写入。"""
        QMessageBox.information(self, "只读配置", "轴绑定由固件业务代码维护，上位机不写入。")

    def _read_from_device(self) -> None:
        """从底层读取只读轴绑定配置并刷新显示。"""
        if not self.win._worker:
            QMessageBox.warning(self, "未连接", "请先连接设备。")
            return
        self.win._enqueue(self.win._mb.read_all_config, tag="read_config")
        # 软限位与轴绑定配置一起刷新
        for idx in range(self.win.active_axis_count()):
            self.win._enqueue(self.win._mb.read_soft_limit, idx, tag=f"slread{idx}")
        self.win.log("[配置] 正在读取底层只读轴绑定配置和软限位...")

    def apply_device_config(self, cfg: dict) -> None:
        """应用通信线程读回的只读轴绑定配置。"""
        axis_count = cfg.get('system', {}).get('axis_count', NUM_AXES)
        self.set_axis_count(axis_count)
        self._set_axis_configs(cfg.get('axes', []))
        self.win.log("[配置] 已读取底层只读轴绑定配置")

    def set_axis_count(self, count: int) -> None:
        count = max(1, min(int(count), NUM_AXES))
        for i, w in enumerate(self._ax_widgets):
            w['group'].setVisible(True)
            w['group'].setProperty("registered", i < count)
            w['group'].setEnabled(i < count)

    def _get_config_dict(self) -> dict:
        """从 UI 收集配置字典。"""
        cfg = {'axes': []}
        for i in range(NUM_AXES):
            w = self._ax_widgets[i]
            ax = {
                'name': w['name'].text().strip() or DEFAULT_AXIS_NAMES[i],
                'pulse_mode': w['pulse_mode'].property('raw_value'),
                'pwm': w['pwm'].property('raw_value'),
                'soft': w['soft'].property('raw_value'),
                'dir': w['dir'].property('raw_value'),
                'home': w['home'].property('raw_value'),
                'limp': w['limp'].property('raw_value'),
                'limn': w['limn'].property('raw_value'),
                'inv': bool(w['inv'].property('raw_value')),
                'home_dir': bool(w['home_dir'].property('raw_value')),
            }
            cfg['axes'].append(ax)
        return cfg

    @staticmethod
    def _cfg_label(text: str, muted: bool = False) -> QLabel:
        label = QLabel(text)
        label.setObjectName("cfgMuted" if muted else "cfgValue")
        label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        label.setProperty('raw_value', None)
        return label

    def _set_axis_configs(self, axes: list) -> None:
        for i, ax in enumerate(axes[:NUM_AXES]):
            w = self._ax_widgets[i]
            pulse_mode = int(ax.get('pulse_mode', 0))
            pwm = ax.get('pwm')
            soft = ax.get('soft')
            direction = ax.get('dir')
            home = ax.get('home')
            limp = ax.get('limp')
            limn = ax.get('limn')
            inv = bool(ax.get('inv', False))
            home_dir = bool(ax.get('home_dir', False))

            self._set_cfg_value(w['pulse_mode'], pulse_mode,
                                self._pulse_mode_text(pulse_mode), False)
            self._set_cfg_value(w['pwm'], pwm, self._pwm_text(pwm), pwm is None)
            self._set_cfg_value(w['soft'], soft, self._soft_pwm_text(soft), soft is None)
            self._set_cfg_value(w['dir'], direction, self._dir_text(direction), direction is None)
            self._set_cfg_value(w['home'], home, self._sensor_text(home), home is None)
            self._set_cfg_value(w['limp'], limp, self._sensor_text(limp), limp is None)
            self._set_cfg_value(w['limn'], limn, self._sensor_text(limn), limn is None)
            self._set_cfg_value(w['inv'], inv, "是" if inv else "否", not inv)
            self._set_cfg_value(w['home_dir'], home_dir,
                                "正向搜索" if home_dir else "负向搜索", False)

    @staticmethod
    def _set_cfg_value(label: QLabel, raw_value, text: str, muted: bool = False) -> None:
        label.setProperty('raw_value', raw_value)
        label.setText(text)
        label.setObjectName("cfgMuted" if muted else "cfgValue")
        label.style().unpolish(label)
        label.style().polish(label)

    @staticmethod
    def _pwm_text(value: int) -> str:
        if value is None:
            return "无 (不使用)"
        value = int(value)
        return PWM_CH_NAMES[value] if 0 <= value < len(PWM_CH_NAMES) else f"未知({value})"

    @staticmethod
    def _soft_pwm_text(value: int) -> str:
        if value is None:
            return "无 (不使用)"
        value = int(value)
        return (SOFT_PWM_CH_NAMES[value]
                if 0 <= value < len(SOFT_PWM_CH_NAMES) else f"未知({value})")

    @staticmethod
    def _pulse_mode_text(value: int) -> str:
        value = int(value)
        if value == 0:
            return "硬件 PWM"
        if value == 1:
            return "软件脉冲"
        return f"未知({value})"

    @staticmethod
    def _dir_text(value: int) -> str:
        if value is None:
            return "无 (不使用)"
        value = int(value)
        return Y_NAMES[value] if 0 <= value < len(Y_NAMES) else f"未知({value})"

    @staticmethod
    def _sensor_text(value: int) -> str:
        if value is None:
            return "无 (不使用)"
        value = int(value)
        return X_NAMES[value] if 0 <= value < len(X_NAMES) else "无 (不使用)"

    def _apply_names_to_cards(self) -> None:
        """将配置面板中的轴名称同步到轴卡片。"""
        for i in range(NUM_AXES):
            name = self._ax_widgets[i]['name'].text().strip() or DEFAULT_AXIS_NAMES[i]
            if i in self.win._axis_cards_map:
                self.win._axis_cards_map[i].set_axis_name(name)
                self.win._axis_cards_map[i].name_changed.emit(i, name)

    def sync_name_from_card(self, idx: int, name: str) -> None:
        """从轴卡片同步名称到配置面板。"""
        if idx < len(self._ax_widgets):
            self._ax_widgets[idx]['name'].blockSignals(True)
            self._ax_widgets[idx]['name'].setText(name)
            self._ax_widgets[idx]['name'].blockSignals(False)
            self._ax_widgets[idx]['group'].setTitle(
                f"{AXIS_BADGES[idx]} {name} 绑定配置")

    def _auto_save(self) -> None:
        """自动保存本地轴名称到 last_config.json。"""
        try:
            cfg = {'axis_names': [
                self._ax_widgets[i]['name'].text().strip() or DEFAULT_AXIS_NAMES[i]
                for i in range(NUM_AXES)
            ]}
            cfg['point_names'] = self.win._points_editor.get_point_names()
            path = app_data_path(CONFIG_FILE)
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(cfg, f, ensure_ascii=False, indent=2)
        except Exception:
            pass

    def auto_load(self) -> None:
        """启动时自动加载本地轴名称。"""
        path = app_data_path(CONFIG_FILE)
        if os.path.isfile(path):
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    cfg = json.load(f)
                self._set_names_from_local(cfg)
                self.win._points_editor.set_point_names(cfg.get('point_names'))
                self._apply_names_to_cards()
                self.win.log("[配置] 自动加载上次轴名称和点位名称")
            except Exception:
                pass

    def _save_local(self) -> None:
        cfg = {'axis_names': [
            self._ax_widgets[i]['name'].text().strip() or DEFAULT_AXIS_NAMES[i]
            for i in range(NUM_AXES)
        ], 'point_names': self.win._points_editor.get_point_names()}
        p, _ = QFileDialog.getSaveFileName(
            self, "保存轴名称", "axis_names.json", "JSON (*.json)")
        if p:
            with open(p, 'w', encoding='utf-8') as f:
                json.dump(cfg, f, ensure_ascii=False, indent=2)
            self.win.log(f"[配置] 轴名称已保存到 {p}")

    def _load_local(self) -> None:
        p, _ = QFileDialog.getOpenFileName(
            self, "加载配置", "", "JSON (*.json)")
        if not p:
            return
        with open(p, 'r', encoding='utf-8') as f:
            cfg = json.load(f)
        self._set_names_from_local(cfg)
        self.win._points_editor.set_point_names(cfg.get('point_names'))
        self._apply_names_to_cards()
        self.win.log(f"[配置] 轴名称和点位名称已从 {p} 加载")

    def _set_names_from_local(self, cfg: dict) -> None:
        names = cfg.get('axis_names')
        if names is None:
            names = [ax.get('name', DEFAULT_AXIS_NAMES[i])
                     for i, ax in enumerate(cfg.get('axes', []))]
        for i, name in enumerate(names[:NUM_AXES]):
            self._ax_widgets[i]['name'].setText(name or DEFAULT_AXIS_NAMES[i])


# ============================================================================
#  定位点面板 (PositionPointPanel)
# ============================================================================

class PositionPointPanel(QWidget):
    """每轴独立 8 个定位点名称和坐标。"""

    def __init__(self, win: "MainWindow"):
        super().__init__()
        self.win = win
        self._tables: List[QTableWidget] = []
        self._groups: List[QGroupBox] = []
        self._spinboxes: dict = {}
        self._speed_spinboxes: dict = {}
        self._point_names: List[List[str]] = build_default_axis_point_names()
        self._current_axis = 0
        self._axis_count = 0
        self._build()

    def _build(self) -> None:
        outer = QVBoxLayout(self)
        outer.setSpacing(10)
        outer.setContentsMargins(0, 0, 0, 0)

        self.lbl_axis_title = QLabel(f"{AXIS_BADGES[0]} {DEFAULT_AXIS_NAMES[0]} 定位点")
        title_font = QFont(); title_font.setBold(True); title_font.setPointSize(14)
        self.lbl_axis_title.setFont(title_font)
        self.lbl_axis_title.setStyleSheet(
            "background: #ffffff; border: 1px solid #dbe3ef; border-radius: 8px; "
            "padding: 10px 12px; color: #0f172a;")
        outer.addWidget(self.lbl_axis_title)

        tb = QHBoxLayout()
        tb.setSpacing(8)
        self.btn_read = QPushButton("从设备读取")
        self.btn_read.setObjectName("btnPrimary")
        self.btn_read.clicked.connect(self._read_from_device)
        tb.addWidget(self.btn_read)
        self.btn_teach = QPushButton("当前位置示教")
        self.btn_teach.setObjectName("btnSuccess")
        self.btn_teach.clicked.connect(self._teach_current)
        tb.addWidget(self.btn_teach)
        self.btn_write = QPushButton("写入到设备")
        self.btn_write.clicked.connect(self._write_to_device)
        tb.addWidget(self.btn_write)
        self.btn_save = QPushButton("保存到 Flash")
        self.btn_save.setObjectName("btnWarn")
        self.btn_save.clicked.connect(self._write_to_flash)
        tb.addWidget(self.btn_save)
        tb.addStretch()
        self.lbl_status = QLabel("未读取")
        self.lbl_status.setStyleSheet("color: #667085; font-weight: 600; padding: 2px 8px;")
        tb.addWidget(self.lbl_status)
        outer.addLayout(tb)

        self._stack = QStackedWidget()

        for axis in range(NUM_AXES):
            group = QGroupBox("点位列表")
            grid = QVBoxLayout(group)
            tbl = QTableWidget()
            tbl.setColumnCount(4)
            tbl.setHorizontalHeaderLabels(["#", "点位名称", "坐标", "速度 (Hz)"])
            tbl.setRowCount(8)
            tbl.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
            tbl.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
            tbl.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
            tbl.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
            tbl.verticalHeader().setVisible(False)
            tbl.setAlternatingRowColors(True)
            tbl.setSelectionBehavior(QTableWidget.SelectItems)
            tbl.setSelectionMode(QTableWidget.SingleSelection)
            tbl.setMinimumHeight(360)
            for row in range(8):
                tbl.setRowHeight(row, 40)
                tbl.setItem(row, 0, _ro_item(str(row + 1)))
                name_item = QTableWidgetItem(self._point_names[axis][row])
                name_item.setFlags(name_item.flags() | Qt.ItemIsEditable)
                tbl.setItem(row, 1, name_item)
                spn = NoWheelSpinBox()
                spn.setRange(-10000000, 10000000)
                spn.setValue(0)
                spn.setMinimumHeight(30)
                spn.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                spn.setProperty("axis", axis)
                spn.setProperty("point_row", row)
                spn.installEventFilter(self)
                tbl.setCellWidget(row, 2, spn)
                self._spinboxes[(axis, row)] = spn

                spd = NoWheelSpinBox()
                spd.setRange(POINT_SPEED_MIN, POINT_SPEED_MAX)
                spd.setSingleStep(100)
                spd.setValue(POINT_SPEED_DEFAULT)
                spd.setSuffix(" Hz")
                spd.setMinimumHeight(30)
                spd.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                spd.setProperty("axis", axis)
                spd.setProperty("point_row", row)
                spd.installEventFilter(self)
                tbl.setCellWidget(row, 3, spd)
                self._speed_spinboxes[(axis, row)] = spd
            tbl.itemChanged.connect(self._on_name_edited)
            grid.addWidget(tbl)
            self._groups.append(group)
            self._tables.append(tbl)
            self._stack.addWidget(group)
        outer.addWidget(self._stack, 1)

    def _on_name_edited(self, item: QTableWidgetItem) -> None:
        if item.column() != 1:
            return
        for axis, tbl in enumerate(self._tables):
            if item.tableWidget() is tbl:
                row = item.row()
                self._point_names[axis][row] = item.text().strip() or DEFAULT_POINT_NAMES[row]
                self.win._axis_cfg._auto_save()
                return

    def set_axis_count(self, count: int) -> None:
        self._axis_count = max(1, min(int(count), NUM_AXES))
        for axis, group in enumerate(self._groups):
            group.setEnabled(axis < self._axis_count)
        if self._current_axis >= self._axis_count:
            self.set_current_axis(0)
        self._update_motion_controls()

    def set_current_axis(self, axis: int) -> None:
        if not (0 <= axis < NUM_AXES):
            return
        self._current_axis = axis
        self._stack.setCurrentIndex(axis)
        name = self.win.axis_card(axis).axis_name
        self.lbl_axis_title.setText(f"{AXIS_BADGES[axis]} {name} 定位点")
        self._update_motion_controls()

    def _update_motion_controls(self) -> None:
        registered = self._current_axis < self._axis_count
        self.btn_teach.setEnabled(registered)
        self.btn_teach.setToolTip("" if registered else "该轴未注册，不能读取当前位置")

    def get_point_names(self) -> List[List[str]]:
        return [list(names) for names in self._point_names]

    def set_point_names(self, names) -> None:
        if not names:
            return
        if names and all(isinstance(x, str) for x in names):
            names = [list(names) for _ in range(NUM_AXES)]
        for axis, axis_names in enumerate(list(names)[:NUM_AXES]):
            for row, name in enumerate(list(axis_names)[:8]):
                text = str(name).strip() or DEFAULT_POINT_NAMES[row]
                self._point_names[axis][row] = text
                item = self._tables[axis].item(row, 1)
                if item:
                    item.setText(text)

    def _read_from_device(self) -> None:
        if not self.win._worker:
            QMessageBox.warning(self, "未连接", "请先连接设备。")
            return
        self.btn_read.setEnabled(False)
        self.lbl_status.setText("读取中...")
        i = self._current_axis
        self.win._enqueue(
            self.win._mb.read_position_point_data, i, tag=f"read_points:{i}")

    def _apply_read_data(self, axis: int, data: dict) -> None:
        pts = data.get('positions', [])
        names = data.get('names', [])
        speeds = data.get('speeds', [])
        self._tables[axis].blockSignals(True)
        try:
            for j in range(min(8, len(pts))):
                self._spinboxes[(axis, j)].setValue(pts[j])
            for j in range(min(8, len(speeds))):
                spd = int(speeds[j])
                if not (POINT_SPEED_MIN <= spd <= POINT_SPEED_MAX):
                    spd = POINT_SPEED_DEFAULT
                self._speed_spinboxes[(axis, j)].setValue(spd)
            for j in range(min(8, len(names))):
                name = str(names[j]).strip() or DEFAULT_POINT_NAMES[j]
                self._point_names[axis][j] = name
                self._tables[axis].item(j, 1).setText(name)
        finally:
            self._tables[axis].blockSignals(False)

    def _write_to_device(self) -> None:
        if not self.win._worker:
            QMessageBox.warning(self, "未连接", "请先连接设备。")
            return
        i = self._current_axis
        positions = [self._spinboxes[(i, j)].value() for j in range(8)]
        names = [self._point_names[i][j] for j in range(8)]
        speeds = [self._speed_spinboxes[(i, j)].value() for j in range(8)]
        self.lbl_status.setText("写入中...")
        self.win._enqueue(
            self.win._mb.write_position_point_data, i, positions, names, speeds,
            tag=f"write_ram:{i}")

    def _write_to_flash(self) -> None:
        if not self.win._worker:
            QMessageBox.warning(self, "未连接", "请先连接设备。")
            return
        ret = QMessageBox.question(self, "确认保存到 Flash", "将定位点数据保存到内部 Flash？",
                                   QMessageBox.Yes | QMessageBox.No)
        if ret != QMessageBox.Yes:
            return
        self.lbl_status.setText("烧写 Flash...")
        self.win._enqueue(self.win._mb.save_points_to_flash, tag="save_flash")

    def handle_command_result(self, tag: str, result) -> None:
        if tag.startswith("read_points:"):
            axis = int(tag.split(":", 1)[1])
            self._apply_read_data(axis, result)
            self.btn_read.setEnabled(True)
            self.lbl_status.setText("坐标、名称和速度已从设备读取")
        elif tag.startswith("write_ram:"):
            self.lbl_status.setText("当前轴已写入 MCU RAM")
        elif tag == "save_flash":
            self.lbl_status.setText("已保存到 Flash")
        else:
            return
        self.lbl_status.setStyleSheet(
            "color: #16a34a; font-weight: 600; padding: 2px 8px;")

    def handle_command_finished(self, tag: str, ok: bool, message: str) -> None:
        if not (tag.startswith("read_points:") or
                tag.startswith("write_ram:") or tag == "save_flash"):
            return
        if tag.startswith("read_points:"):
            self.btn_read.setEnabled(True)
        if not ok:
            self.lbl_status.setText(f"操作失败: {message}")
            self.lbl_status.setStyleSheet(
                "color: #dc2626; font-weight: 600; padding: 2px 8px;")

    def _teach_current(self) -> None:
        axis = self._current_axis
        if axis >= self._axis_count:
            self.win.log(f"轴{axis} 未注册，无法示教当前位置")
            return
        tbl = self._tables[axis]
        row = tbl.currentRow()
        if row < 0:
            QMessageBox.information(self, "提示", "请先点选当前轴表格中的点位行。")
            return
        pos = self.win.axis_card(axis).get_position()
        self._spinboxes[(axis, row)].setValue(pos)
        name = self._point_names[axis][row]
        self.win.log(f"[示教] {self.win.axis_card(axis).axis_name} {name} <- {pos:+d}")

    def eventFilter(self, watched, event):
        if isinstance(watched, QSpinBox) and event.type() in (QEvent.FocusIn, QEvent.MouseButtonPress):
            axis = watched.property("axis")
            row = watched.property("point_row")
            if axis is not None and row is not None:
                col = 3 if watched in self._speed_spinboxes.values() else 2
                self._tables[int(axis)].setCurrentCell(int(row), col)
        return super().eventFilter(watched, event)

    def update_axis_names(self, names: List[str]) -> None:
        if 0 <= self._current_axis < len(names):
            self.lbl_axis_title.setText(
                f"{AXIS_BADGES[self._current_axis]} {names[self._current_axis]} 定位点")


# ============================================================================
#  主窗口
# ============================================================================

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.resize(1440, 900)
        self.setMinimumSize(1180, 760)

        self._mb = ModbusMaster()
        self._ser: Optional[serial.Serial] = None
        self._worker: Optional[CommWorker] = None
        self._thread = None
        self._disconnecting = False
        self._active_axis_count = NUM_AXES
        self._build()
        self._refresh_ports()
        self._current_mode = MODE_NORMAL

    # ----------------------------------------------------------------------
    # UI
    # ----------------------------------------------------------------------

    def _build(self) -> None:
        cw = QWidget()
        self.setCentralWidget(cw)
        outer = QVBoxLayout(cw)
        outer.setSpacing(8)
        outer.setContentsMargins(10, 10, 10, 10)

        outer.addWidget(self._build_top_bar())

        # 中间分两侧: 左 = 轴卡片栈, 右 = 标签页 (示教 / IO / 系统)
        splitter = QSplitter(Qt.Horizontal)

        # 左侧：轴选择器 + 当前轴详情
        left_panel = QWidget()
        left_lay = QVBoxLayout(left_panel)
        left_lay.setContentsMargins(0, 0, 8, 0)
        left_lay.setSpacing(8)

        nav_panel = QWidget()
        nav_lay = QHBoxLayout(nav_panel)
        nav_lay.setContentsMargins(0, 0, 0, 0)
        nav_lay.setSpacing(8)
        self._axis_cards: List[AxisCard] = []
        self._axis_navs: List[AxisNavCard] = []
        self._axis_cards_map = {}
        self._axis_stack = QStackedWidget()
        for i in range(NUM_AXES):
            nav = AxisNavCard(i)
            nav.clicked.connect(self._select_axis)
            self._axis_navs.append(nav)
            nav_lay.addWidget(nav)
            card = AxisCard(i, self)
            self._axis_cards.append(card)
            self._axis_cards_map[i] = card
            self._axis_stack.addWidget(card)
        nav_lay.addStretch()
        nav_scroll = QScrollArea()
        nav_scroll.setFrameShape(QFrame.NoFrame)
        nav_scroll.setWidgetResizable(True)
        nav_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        nav_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        nav_scroll.setFixedHeight(78)
        nav_scroll.setWidget(nav_panel)
        left_lay.addWidget(nav_scroll)

        axis_scroll = QScrollArea()
        axis_scroll.setObjectName("axisScroll")
        axis_scroll.setFrameShape(QFrame.NoFrame)
        axis_scroll.setWidgetResizable(True)
        axis_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        axis_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        axis_scroll.setWidget(self._axis_stack)
        left_lay.addWidget(axis_scroll, 1)
        left_panel.setMinimumWidth(300)
        splitter.addWidget(left_panel)

        # 右侧
        self._tabs = QTabWidget()
        self._teach = TeachPanel(self)
        self._tabs.addTab(self._teach, "示教点 (Teach)")

        self._axis_cfg = AxisConfigPanel(self)
        self._tabs.addTab(self._axis_cfg, "轴配置")

        io_points = QWidget()
        io_points_lay = QVBoxLayout(io_points)
        io_points_lay.setContentsMargins(8, 8, 8, 8)
        io_points_lay.setSpacing(12)

        io_points_top = QSplitter(Qt.Horizontal)
        io_points_top.setChildrenCollapsible(False)
        self._points_editor = PositionPointPanel(self)
        self._io = IOPanel(self, compact=True)

        io_scroll = QScrollArea()
        io_scroll.setObjectName("axisScroll")
        io_scroll.setFrameShape(QFrame.NoFrame)
        io_scroll.setWidgetResizable(True)
        io_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        io_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        io_scroll.setMinimumWidth(270)
        io_scroll.setMaximumWidth(360)
        io_scroll.setWidget(self._io)

        io_points_top.addWidget(self._points_editor)
        io_points_top.addWidget(io_scroll)
        io_points_top.setStretchFactor(0, 1)
        io_points_top.setStretchFactor(1, 0)
        io_points_top.setSizes([760, 320])
        io_points_lay.addWidget(io_points_top, 1)

        self._sys_panel = self._build_sys_tab()
        self._sys_panel.setMinimumHeight(154)
        self._sys_panel.setMaximumHeight(178)
        io_points_lay.addWidget(self._sys_panel)
        self._tabs.addTab(io_points, "定位点 / I/O")
        splitter.addWidget(self._tabs)

        # 连接轴名称变更信号
        for i in range(NUM_AXES):
            card = self._axis_cards[i]
            card.name_changed.connect(self._on_axis_name_changed)

        splitter.setStretchFactor(0, 42)
        splitter.setStretchFactor(1, 58)
        splitter.setSizes([500, 700])
        splitter.setChildrenCollapsible(False)
        outer.addWidget(splitter, 1)

        # 日志
        self._log = QPlainTextEdit()
        self._log.setReadOnly(True)
        self._log.setMaximumBlockCount(800)
        self._log.setMinimumHeight(100)
        self._log.setMaximumHeight(160)
        outer.addWidget(self._log)

        # 状态栏
        sb = QStatusBar()
        self.setStatusBar(sb)
        self.lbl_status_conn = QLabel("● 未连接")
        self.lbl_status_conn.setStyleSheet("color: #dc2626; font-weight: bold;")
        sb.addWidget(self.lbl_status_conn)

        self.lbl_status_diag = QLabel("FrameOK=0  Err=0")
        sb.addPermanentWidget(self.lbl_status_diag)

        self.setStyleSheet(QSS)

        # 启动时自动加载上次配置
        QTimer.singleShot(100, self._axis_cfg.auto_load)
        self._apply_axis_count(NUM_AXES)

    # ---- 顶部连接栏 ----

    def _build_top_bar(self) -> QWidget:
        bar = QFrame()
        bar.setObjectName("topBar")
        outer = QVBoxLayout(bar)
        outer.setContentsMargins(14, 8, 14, 8)
        outer.setSpacing(6)
        h = QHBoxLayout()
        h.setSpacing(10)

        h.addWidget(QLabel("端口:"))
        self.cmb_port = NoWheelComboBox()
        self.cmb_port.setMinimumWidth(160)
        h.addWidget(self.cmb_port)

        self.btn_refresh = QPushButton("刷新")
        self.btn_refresh.clicked.connect(self._refresh_ports)
        h.addWidget(self.btn_refresh)

        h.addWidget(QLabel("波特率:"))
        self.cmb_baud = NoWheelComboBox()
        self.cmb_baud.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.cmb_baud.setCurrentText("115200")  # 与固件 app_config.h 一致
        h.addWidget(self.cmb_baud)

        h.addWidget(QLabel("从站地址:"))
        self.spn_addr = NoWheelSpinBox()
        self.spn_addr.setRange(1, 247)
        self.spn_addr.setValue(1)
        h.addWidget(self.spn_addr)

        self.btn_conn = QPushButton("连接")
        self.btn_conn.setObjectName("btnPrimary")
        self.btn_conn.clicked.connect(self._toggle_connect)
        h.addWidget(self.btn_conn)
        h.addStretch()
        outer.addLayout(h)

        status_row = QHBoxLayout()
        status_row.setSpacing(10)

        # 运行模式显示 (只读, 由固件当前状态决定)
        self.lbl_op_mode = QLabel("正常运行")
        self.lbl_op_mode.setStyleSheet(
            "background: #dbeafe; color: #1e40af; padding: 4px 12px; "
            "border-radius: 12px; font-weight: bold; font-size: 13px;")
        status_row.addWidget(self.lbl_op_mode)
        status_row.addStretch()

        self.lbl_conn = QLabel("● 未连接")
        self.lbl_conn.setObjectName("lblConn")
        self.lbl_conn.setStyleSheet("background: #fee2e2; color: #991b1b; padding: 4px 12px; border-radius: 12px;")
        status_row.addWidget(self.lbl_conn)

        # 大急停按钮
        self.btn_estop = QPushButton("⏹ 急  停")
        self.btn_estop.setObjectName("btnEStop")
        self.btn_estop.clicked.connect(self._on_estop)
        status_row.addWidget(self.btn_estop)
        outer.addLayout(status_row)

        return bar

    # ---- 系统标签页 ----

    def _build_sys_tab(self) -> QWidget:
        w = QWidget()
        w.setObjectName("systemPanel")
        grid = QGridLayout(w)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(0)

        # 回原参数
        home_grp = QGroupBox("回原参数")
        home_grp.setObjectName("homeGroup")
        home_grp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        hg = QGridLayout(home_grp)
        hg.setContentsMargins(12, 16, 12, 10)
        hg.setHorizontalSpacing(8)
        hg.setVerticalSpacing(4)
        hg.addWidget(QLabel("快速 Hz"), 0, 0)
        self.spn_home_fast = NoWheelSpinBox(); self.spn_home_fast.setRange(100, 100000); self.spn_home_fast.setValue(1250)
        self.spn_home_fast.setFixedWidth(76)
        hg.addWidget(self.spn_home_fast, 1, 0)
        hg.addWidget(QLabel("慢速 Hz"), 0, 1)
        self.spn_home_slow = NoWheelSpinBox(); self.spn_home_slow.setRange(100, 100000); self.spn_home_slow.setValue(300)
        self.spn_home_slow.setFixedWidth(76)
        hg.addWidget(self.spn_home_slow, 1, 1)
        hg.addWidget(QLabel("回退 Hz"), 0, 2)
        self.spn_home_back = NoWheelSpinBox(); self.spn_home_back.setRange(100, 100000); self.spn_home_back.setValue(300)
        self.spn_home_back.setFixedWidth(80)
        hg.addWidget(self.spn_home_back, 1, 2)
        hg.addWidget(QLabel("回原模式"), 0, 3)
        self.cmb_home_mode = NoWheelComboBox()
        self.cmb_home_mode.addItem("精回原 (退到释放+慢找)", 0)
        self.cmb_home_mode.addItem("退到释放即回原", 1)
        self.cmb_home_mode.setFixedWidth(160)
        hg.addWidget(self.cmb_home_mode, 1, 3)
        self.btn_apply_home = QPushButton("应用")
        self.btn_apply_home.setObjectName("btnPrimary")
        self.btn_apply_home.clicked.connect(self._on_apply_home)
        self.btn_apply_home.setFixedWidth(60)
        hg.addWidget(self.btn_apply_home, 1, 4)
        hg.setColumnStretch(3, 1)
        grid.addWidget(home_grp, 0, 0)

        # 通信看门狗
        wdt_grp = QGroupBox("通信看门狗")
        wdt_grp.setObjectName("wdtGroup")
        wg = QGridLayout(wdt_grp)
        wg.setContentsMargins(12, 16, 12, 10)
        wg.setHorizontalSpacing(8)
        wg.setVerticalSpacing(8)
        wg.addWidget(QLabel("超时 ms"), 0, 0)
        self.spn_wdt = NoWheelSpinBox()
        self.spn_wdt.setRange(0, 60000)
        self.spn_wdt.setValue(2000)
        self.spn_wdt.setSpecialValueText("禁用")
        self.spn_wdt.setFixedWidth(82)
        wg.addWidget(self.spn_wdt, 0, 1)
        self.btn_wdt = QPushButton("写入")
        self.btn_wdt.setFixedWidth(54)
        self.btn_wdt.clicked.connect(self._on_apply_wdt)
        wg.addWidget(self.btn_wdt, 0, 2)
        self.lbl_diag = QLabel("FrameOK=0  Err=0  WDT_ST=0")
        self.lbl_diag.setWordWrap(True)
        self.lbl_diag.setMinimumWidth(0)
        self.lbl_diag.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self.lbl_diag.setStyleSheet(
            "color: #475569; font-family: 'Cascadia Code', 'Consolas', monospace; "
            "background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 5px; padding: 6px;")
        wg.addWidget(self.lbl_diag, 1, 0, 1, 3)
        wg.setColumnStretch(1, 1)
        grid.addWidget(wdt_grp, 0, 1)

        # 固件信息
        info_grp = QGroupBox("固件信息")
        info_grp.setObjectName("firmwareGroup")
        ig = QGridLayout(info_grp)
        ig.setContentsMargins(12, 16, 12, 10)
        ig.setVerticalSpacing(8)
        self.lbl_version = QLabel("—")
        self.lbl_version.setStyleSheet(
            "font-family: 'Cascadia Code', 'Consolas', monospace; font-size: 16px; "
            "font-weight: 700; color: #0f172a;")
        ig.addWidget(QLabel("版本:"), 0, 0)
        ig.addWidget(self.lbl_version, 0, 1)
        self.btn_read_ver = QPushButton("读取")
        self.btn_read_ver.clicked.connect(self._on_read_version)
        ig.addWidget(self.btn_read_ver, 1, 0, 1, 2)
        ig.setColumnStretch(1, 1)
        grid.addWidget(info_grp, 0, 2)

        grid.setColumnStretch(0, 5)
        grid.setColumnStretch(1, 3)
        grid.setColumnStretch(2, 2)
        return w

    # ----------------------------------------------------------------------
    # 端口
    # ----------------------------------------------------------------------

    def _refresh_ports(self) -> None:
        ports = serial.tools.list_ports.comports()
        cur = self.cmb_port.currentText()
        self.cmb_port.clear()
        for p in ports:
            self.cmb_port.addItem(f"{p.device} - {p.description}", userData=p.device)
        if cur:
            for i in range(self.cmb_port.count()):
                if self.cmb_port.itemData(i) == cur:
                    self.cmb_port.setCurrentIndex(i)
                    break

    def axis_card(self, idx: int) -> AxisCard:
        return self._axis_cards[idx]

    def active_axis_count(self) -> int:
        return max(1, min(int(self._active_axis_count), NUM_AXES))

    def _select_axis(self, idx: int) -> None:
        if not (0 <= idx < self.active_axis_count()):
            return
        self._axis_stack.setCurrentIndex(idx)
        self._points_editor.set_current_axis(idx)
        for i, nav in enumerate(self._axis_navs):
            nav.set_selected(i == idx)

    def _apply_axis_count(self, count: int) -> None:
        count = max(1, min(int(count), NUM_AXES))
        self._active_axis_count = count
        for i in range(NUM_AXES):
            available = i < count
            self._axis_cards[i].set_axis_available(available)
            self._axis_navs[i].set_axis_available(available)
        if self._axis_stack.currentIndex() >= count:
            self._select_axis(0)
        else:
            self._select_axis(max(0, self._axis_stack.currentIndex()))
        self._axis_cfg.set_axis_count(count)
        self._points_editor.set_axis_count(count)
        self._teach.set_axis_count(count)
        self._update_teach_headers()

    def _on_axis_name_changed(self, idx: int, name: str) -> None:
        """轴名称变更时同步到配置面板和示教表头。"""
        self._axis_cfg.sync_name_from_card(idx, name)
        self._axis_navs[idx].set_axis_name(name)
        # 更新示教点表格表头
        self._update_teach_headers()

    def _update_teach_headers(self) -> None:
        """更新示教点表格表头中的轴名称。"""
        self._teach._reset_headers()
        names = [self._axis_cards[i].axis_name for i in range(NUM_AXES)]
        self._points_editor.update_axis_names(names)

    # ----------------------------------------------------------------------
    # 连接 / 断开
    # ----------------------------------------------------------------------

    def _toggle_connect(self) -> None:
        if self._mb.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self) -> None:
        if self._thread is not None and self._thread.isRunning():
            QMessageBox.warning(self, "通信线程忙", "上一次通信线程仍在退出，请稍后重试。")
            return
        device = self.cmb_port.currentData()
        if not device:
            QMessageBox.warning(self, "提示", "请选择串口")
            return
        baud = int(self.cmb_baud.currentText())
        addr = self.spn_addr.value()

        try:
            ser = serial.Serial(device, baudrate=baud, bytesize=8,
                                parity='N', stopbits=1,
                                timeout=mm.DEFAULT_TIMEOUT_S, write_timeout=0.5)
        except Exception as e:
            QMessageBox.critical(self, "串口打开失败", str(e))
            return

        self._mb.set_serial(ser)
        self._mb.slave_addr = addr
        self._mb.reset_stats()

        try:
            ver = self._mb.get_version()
        except Exception as e:
            self._mb.set_serial(None)
            ser.close()
            QMessageBox.critical(self, "设备握手失败", f"串口已打开，但未读到 Modbus 设备响应：\n{e}")
            return
        if ver != EXPECTED_FW_VERSION:
            self._mb.set_serial(None)
            ser.close()
            QMessageBox.critical(self, "固件不匹配",
                                 f"读到固件版本 0x{ver:04X}，期望 0x{EXPECTED_FW_VERSION:04X}。")
            return

        try:
            axis_count = self._mb.read_axis_count()
        except Exception as e:
            self._mb.set_serial(None)
            ser.close()
            QMessageBox.critical(self, "轴配置读取失败", f"设备已响应，但无法读取实际轴数：\n{e}")
            return
        self._apply_axis_count(axis_count)

        # 启动通信线程
        self._worker, self._thread = start_worker(self._mb, WorkerConfig(axis_count=axis_count))
        self._worker.axis_status.connect(self._on_axis_status)
        self._worker.io_status.connect(self._on_io_status)
        self._worker.comm_diag.connect(self._on_comm_diag)
        self._worker.op_mode_changed.connect(self._on_op_mode_changed)
        self._worker.config_read.connect(self._on_config_read)
        self._worker.command_result.connect(self._on_command_result)
        self._worker.cmd_finished.connect(self._on_cmd_finished)
        self._worker.error.connect(self._on_worker_error)
        self._worker.info.connect(self.log)
        self._worker.disconnected.connect(self._on_worker_disconnected)

        # UI 状态切换
        self.btn_conn.setText("断开")
        self.lbl_conn.setText("● 已连接")
        self.lbl_conn.setStyleSheet(
            "background: #dcfce7; color: #166534; padding: 4px 12px; border-radius: 12px; font-weight: bold;")
        self.lbl_status_conn.setText(f"● 已连接  {device} @ {baud} (Slave={addr})")
        self.lbl_status_conn.setStyleSheet("color: #16a34a; font-weight: bold;")
        self.cmb_port.setEnabled(False)
        self.cmb_baud.setEnabled(False)
        self.spn_addr.setEnabled(False)
        self.lbl_version.setText(f"0x{ver:04X}")
        self.log(f"连接 {device} @ {baud} 8N1, 从站地址={addr}, 固件版本=0x{ver:04X}, 实际轴数={axis_count}")
        self._worker.enqueue(self._mb.read_all_config, tag="read_config")

    def _disconnect(self) -> None:
        if self._disconnecting:
            return
        self._disconnecting = True

        worker = self._worker
        thread = self._thread
        ser = self._mb.serial

        if worker:
            worker.stop()
        stopped = True
        if thread:
            thread.quit()
            stopped = thread.wait(3000)
        if not stopped and ser:
            try:
                ser.close()
            except Exception:
                pass
            stopped = thread.wait(1500)

        self._mb.set_serial(None)
        if ser:
            try:
                ser.close()
            except Exception:
                pass
        if stopped:
            self._worker = None
            self._thread = None
        else:
            self.log("通信线程未能在超时时间内退出；串口已关闭，等待线程自行结束")

        self.btn_conn.setText("连接")
        self.lbl_conn.setText("● 未连接")
        self.lbl_conn.setStyleSheet(
            "background: #fee2e2; color: #991b1b; padding: 4px 12px; border-radius: 12px; font-weight: bold;")
        self.lbl_status_conn.setText("● 未连接")
        self.lbl_status_conn.setStyleSheet("color: #dc2626; font-weight: bold;")
        self.cmb_port.setEnabled(True)
        self.cmb_baud.setEnabled(True)
        self.spn_addr.setEnabled(True)
        self._apply_axis_count(NUM_AXES)
        self.log("已断开")
        self._disconnecting = False

    def _on_worker_disconnected(self) -> None:
        # worker 主动结束 (例如连续超时)
        if not self._disconnecting and self._mb.is_open:
            self._disconnect()

    # ----------------------------------------------------------------------
    # Worker -> UI
    # ----------------------------------------------------------------------

    def _on_axis_status(self, statuses: list) -> None:
        for i, st in enumerate(statuses):
            if i < self.active_axis_count() and i < len(self._axis_cards):
                self._axis_cards[i]._update_status(st)

    def _on_io_status(self, di: list, do: list) -> None:
        self._io.update_io(di, do)

    def _on_comm_diag(self, diag: CommDiag) -> None:
        ok, err = self._mb.stats
        self.lbl_status_diag.setText(f"FrameOK={ok}  Err={err}  WDT_ST={diag.wdt_triggered}")
        self.lbl_diag.setText(
            f"固件帧计数={diag.frame_count}  错误计数={diag.err_count}  "
            f"WDT_ST={diag.wdt_triggered}  WDT_MS={diag.wdt_ms}")
        if diag.wdt_triggered:
            self.lbl_status_diag.setStyleSheet("color: #dc2626; font-weight: bold;")
        else:
            self.lbl_status_diag.setStyleSheet("")

    def _on_config_read(self, cfg: dict) -> None:
        system = cfg.get('system', {})
        self._apply_axis_count(int(system.get('axis_count', self.active_axis_count())))
        self._axis_cfg.apply_device_config(cfg)
        self._io.set_reserved_outputs(
            cfg.get('axes', [])[:self.active_axis_count()])
        self.spn_home_fast.setValue(int(system.get('home_fast', self.spn_home_fast.value())))
        self.spn_home_slow.setValue(int(system.get('home_slow', self.spn_home_slow.value())))
        # 回退速度: 新固件读 221; 旧固件无此值时保持当前值 (默认 300)
        backoff_hz = int(system.get('home_backoff_hz', 0))
        if backoff_hz >= 100:
            self.spn_home_back.setValue(backoff_hz)
        home_mode = int(system.get('home_mode', 0))
        idx = self.cmb_home_mode.findData(1 if home_mode == 1 else 0)
        if idx >= 0:
            self.cmb_home_mode.setCurrentIndex(idx)
        self.spn_wdt.setValue(int(system.get('comm_wdt_ms', self.spn_wdt.value())))

    def _on_worker_error(self, msg: str) -> None:
        self.log(f"[ERR] {msg}")

    def _on_command_result(self, tag: str, result) -> None:
        self._points_editor.handle_command_result(tag, result)
        self._axis_cfg.handle_command_result(tag, result)
        if tag == "version":
            ver = int(result)
            self.lbl_version.setText(f"0x{ver:04X}")
            self.log(f"固件版本: 0x{ver:04X}")

    def _on_cmd_finished(self, tag: str, ok: bool, message: str) -> None:
        self._points_editor.handle_command_finished(tag, ok, message)
        self._axis_cfg.handle_command_finished(tag, ok, message)

    def _on_op_mode_changed(self, mode: int) -> None:
        changed = mode != self._current_mode
        self._current_mode = mode
        self._update_mode_label(mode)
        self._io.set_op_mode(mode)
        if changed:
            self.log(f"运行模式: {MODE_NAMES.get(mode, mode)}")

    def _update_mode_label(self, mode: int) -> None:
        name = MODE_NAMES.get(mode, f"未知({mode})")
        if mode == MODE_DEBUG:
            self.lbl_op_mode.setText(name)
            self.lbl_op_mode.setStyleSheet(
                "background: #fef3c7; color: #92400e; padding: 4px 12px; "
                "border-radius: 12px; font-weight: bold; font-size: 13px;")
        else:
            self.lbl_op_mode.setText(name)
            self.lbl_op_mode.setStyleSheet(
                "background: #dbeafe; color: #1e40af; padding: 4px 12px; "
                "border-radius: 12px; font-weight: bold; font-size: 13px;")

    # ----------------------------------------------------------------------
    # UI -> Worker (命令快捷封装)
    # ----------------------------------------------------------------------

    def _enqueue(self, fn, *args, tag: str = "") -> None:
        if not self._worker:
            self.log("未连接, 命令忽略")
            return
        self._worker.enqueue(fn, *args, tag=tag)

    def _axis_enabled(self, axis: int) -> bool:
        if 0 <= axis < self.active_axis_count():
            return True
        self.log(f"轴{axis} 未注册, 命令忽略")
        return False

    def cmd_jog(self, axis: int, positive: bool) -> None:
        if not self._axis_enabled(axis):
            return
        if self._current_mode == MODE_NORMAL:
            self.log("正常运行模式, JOG 禁用")
            return
        speed = self._axis_cards[axis].get_speed()
        self._enqueue(self._mb.axis_jog, axis, speed, positive,
                      tag=f"jog{axis}{'+' if positive else '-'}")

    def cmd_stop(self, axis: int) -> None:
        if not self._axis_enabled(axis):
            return
        self._enqueue(self._mb.axis_stop, axis, tag=f"stop{axis}")

    def cmd_move_rel(self, axis: int, steps: int, speed: int) -> None:
        if not self._axis_enabled(axis):
            return
        if steps == 0:
            return
        if self._current_mode == MODE_NORMAL:
            self.log("正常运行模式, 定位禁用")
            return
        self._enqueue(self._mb.axis_move_rel, axis, speed, steps,
                      tag=f"rel{axis}={steps:+d}")

    def cmd_move_abs(self, axis: int, target: int, speed: int) -> None:
        if not self._axis_enabled(axis):
            return
        if self._current_mode == MODE_NORMAL:
            self.log("正常运行模式, 定位禁用")
            return
        self._enqueue(self._mb.axis_move_abs, axis, speed, target,
                      tag=f"abs{axis}={target}")

    def cmd_home(self, axis: int) -> None:
        if not self._axis_enabled(axis):
            return
        if self._current_mode == MODE_NORMAL:
            self.log("正常运行模式, 回原禁用")
            return
        self._enqueue(self._mb.axis_home, axis, tag=f"home{axis}")

    def cmd_set_pos(self, axis: int, pos: int) -> None:
        if not self._axis_enabled(axis):
            return
        self._enqueue(self._mb.axis_set_position, axis, pos, tag=f"setpos{axis}={pos}")

    def cmd_resume_remaining(self, axis: int) -> None:
        if not self._axis_enabled(axis):
            return
        if self._current_mode == MODE_NORMAL:
            self.log("正常运行模式, 继续剩余禁用")
            return
        self._enqueue(self._mb.axis_resume_remaining, axis, tag=f"resume{axis}")

    def cmd_write_coil(self, addr: int, value: bool) -> None:
        if addr in self._io._reserved_outputs:
            if self._current_mode == MODE_NORMAL:
                self.log(f"Y 输出 {addr} 被运动轴占用；请先切换到调试模式，"
                         "并确保对应轴空闲")
                return
            self.log(f"Y 输出 {addr} 被运动轴占用；轴运动中固件会拒绝写入")
        self._enqueue(self._mb.write_coil, addr, value, tag=f"coil{addr}={int(value)}")

    def _on_estop(self) -> None:
        if not self._mb.is_open:
            self.log("未连接, 急停忽略")
            return
        if self._worker:
            self._worker.emergency_stop()
        else:
            self._mb.emergency_stop()
        self.log("⏹ 已发送急停命令")

    def _on_apply_home(self) -> None:
        # 第 3 个参数现在是回退速度 (Hz), 写入 221 号寄存器
        self._enqueue(self._mb.set_home_params,
                      self.spn_home_fast.value(),
                      self.spn_home_slow.value(),
                      self.spn_home_back.value(),
                      self.cmb_home_mode.currentData(),
                      tag="home_params")

    def _on_apply_wdt(self) -> None:
        self._enqueue(self._mb.set_comm_wdt_ms, self.spn_wdt.value(), tag="wdt")

    def _on_read_version(self) -> None:
        if not self._worker:
            return
        self._worker.enqueue(self._mb.get_version, tag="version")

    # ----------------------------------------------------------------------
    # 日志
    # ----------------------------------------------------------------------

    def log(self, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self._log.appendPlainText(f"[{ts}] {msg}")

    # ---- 关闭 ----

    def closeEvent(self, event) -> None:
        try:
            self._axis_cfg._auto_save()
            if self._mb.is_open:
                # 关闭前尽量发一次急停, 防止电机继续运行
                try:
                    self._mb.emergency_stop()
                except Exception:
                    pass
                self._disconnect()
        finally:
            event.accept()


# ============================================================================
#  入口
# ============================================================================

def main() -> int:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # 强制应用 QSS
    win = MainWindow()
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
