#!/usr/bin/env python3
"""hand_arm_monitor.py - 用 matplotlib 实时可视化 hand / arm 的目标关节值。

订阅:
    /hand_joint_targets   std_msgs/Float64MultiArray  (6 个手关节角, rad)
    /arm_joint_targets    std_msgs/Float64MultiArray  (6 个臂关节角, rad)

界面布局:
    ┌──────────────────────────────────────────────────────────────┐
    │  ARM (UR5) 侧视示意图      │ 底盘俯视(看 pan)  │ 6 关节角度条 │
    ├────────────────────── 虚线 ─────────────────────────────────┤
    │  HAND 五指示意(看弯曲度)   │                   6 关节角度条  │
    └──────────────────────────────────────────────────────────────┘

用法:
    ros2 run ur_inspire_hand hand_arm_monitor.py
    ros2 launch ur_inspire_hand hand_arm_monitor.launch.py
    # 可选参数（调试用）:
    ros2 run ur_inspire_hand hand_arm_monitor.py --snapshot /tmp/hand_arm.png
    ros2 run ur_inspire_hand hand_arm_monitor.py --duration 10

说明: 图内文字用英文，避免没装中文字体时显示成方块；终端日志仍是中文。
"""

import math
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, FancyBboxPatch, Polygon

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

HAND_TOPIC = "/hand_joint_targets"
ARM_TOPIC = "/arm_joint_targets"

DEG = 180.0 / math.pi

# ---- 配色（浅色主题） -----------------------------------------------------
BG = "#f6f8fb"
PANEL = "#ffffff"
FG = "#1f2933"
MUTED = "#7b8794"
GRID = "#dde3ea"
ARM_ACCENT = "#20507e"
HAND_ACCENT = "#b05a2a"
# 与 Gazebo 里 6 段连杆颜色一致：红 橙 黄 绿 蓝 紫
LINK_COLORS = ["#d64545", "#e07a2b", "#d9b41c", "#3f9a4a", "#2f6fb5", "#7c4fc4"]

ARM_LABELS = ["shoulder_pan", "shoulder_lift", "elbow",
              "wrist_1", "wrist_2", "wrist_3"]
HAND_LABELS = ["thumb_1", "thumb_2", "index", "middle", "ring", "little"]

# 与 hand_preset.py 保持一致的关节顺序
ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
HAND_JOINTS = [
    "right_thumb_1_joint",
    "right_thumb_2_joint",
    "right_index_1_joint",
    "right_middle_1_joint",
    "right_ring_1_joint",
    "right_little_1_joint",
]

# UR5 标准 DH 参数: (d, a, alpha)
UR5_DH = [
    (0.089159, 0.0, math.pi / 2),
    (0.0, -0.425, 0.0),
    (0.0, -0.39225, 0.0),
    (0.10915, 0.0, math.pi / 2),
    (0.09465, 0.0, -math.pi / 2),
    (0.0823, 0.0, 0.0),
]

# 食指等四指的从动关节系数（mimic multiplier，来自 URDF）
MIMIC = 1.1169


def fk_ur5(q):
    """正运动学：返回 base -> 6 个关节原点 -> 末端 共 7 个 3D 点。"""
    t = np.eye(4)
    pts = [t[:3, 3].copy()]
    for qi, (d, a, alpha) in zip(q, UR5_DH):
        ct, st = math.cos(qi), math.sin(qi)
        ca, sa = math.cos(alpha), math.sin(alpha)
        t = t @ np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ])
        pts.append(t[:3, 3].copy())
    return np.array(pts)


class Monitor(Node):
    def __init__(self):
        super().__init__("hand_arm_monitor")
        # 初始值：手全开、臂 home（和仿真启动位一致）
        self.hand = [0.0] * 6
        self.arm = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]
        self.labels = {"hand": "-", "arm": "-"}
        self.stamps = {"hand": "-", "arm": "-"}
        self.got_cmd = {"hand": False, "arm": False}
        self.dirty = True

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Float64MultiArray, HAND_TOPIC,
                                 lambda m: self._on_msg("hand", m), qos)
        self.create_subscription(Float64MultiArray, ARM_TOPIC,
                                 lambda m: self._on_msg("arm", m), qos)
        # 收到命令之前，先用 /joint_states 的真实值填充，界面一开就有内容
        self.create_subscription(JointState, "/joint_states",
                                 self._on_joint_states, 10)
        self.get_logger().info("订阅: %s | %s | /joint_states"
                               % (HAND_TOPIC, ARM_TOPIC))

    def _on_joint_states(self, msg):
        idx = {}
        for name, pos in zip(msg.name, msg.position):
            idx[name] = float(pos)

        def near_same(a, b):
            return all(abs(x - y) < 0.004 for x, y in zip(a, b))

        changed = False
        if not self.got_cmd["arm"] and set(ARM_JOINTS) & set(idx):
            values = [idx.get(n, 0.0) for n in ARM_JOINTS]
            if not near_same(values, self.arm):
                self.arm = values
                changed = True
            self.labels["arm"] = "actual"
        if not self.got_cmd["hand"] and set(HAND_JOINTS) & set(idx):
            values = [idx.get(n, 0.0) for n in HAND_JOINTS]
            if not near_same(values, self.hand):
                self.hand = values
                changed = True
            self.labels["hand"] = "actual"
        if changed:
            self.dirty = True

    def _on_msg(self, key, msg):
        values = [float(v) for v in msg.data]
        if len(values) != 6:
            self.get_logger().warn(
                "%s 收到 %d 个值（应为 6），已忽略" % (key, len(values)))
            return
        setattr(self, key, values)
        self.got_cmd[key] = True
        label = msg.layout.dim[0].label if msg.layout.dim else ""
        self.labels[key] = label or key
        self.stamps[key] = time.strftime("%H:%M:%S")
        self.dirty = True
        self.get_logger().info(
            "[%s] %s  <- %s" % (key, ["%.3f" % v for v in values], self.labels[key]))


class Viewer:
    def __init__(self, node):
        self.node = node
        self.fig = plt.figure(figsize=(12.6, 8.6), facecolor=BG)
        self.fig.canvas.manager.set_window_title("UR + Inspire Hand Monitor")

        gs = self.fig.add_gridspec(
            2, 2,
            left=0.055, right=0.975, top=0.915, bottom=0.055,
            width_ratios=[1.45, 1.0], height_ratios=[1.0, 1.0],
            hspace=0.45, wspace=0.14,
        )
        top = gs[0, 0].subgridspec(1, 2, width_ratios=[3.1, 1.0], wspace=0.12)
        self.ax_arm = self.fig.add_subplot(top[0, 0])
        self.ax_top = self.fig.add_subplot(top[0, 1])
        self.ax_hand = self.fig.add_subplot(gs[1, 0])
        self.ax_arm_bar = self.fig.add_subplot(gs[0, 1])
        self.ax_hand_bar = self.fig.add_subplot(gs[1, 1])

        for ax in (self.ax_arm, self.ax_hand, self.ax_arm_bar, self.ax_hand_bar):
            ax.set_facecolor(PANEL)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            for side in ("left", "bottom"):
                ax.spines[side].set_color(GRID)

        # 中间的分隔虚线（贯穿整幅图）
        self.fig.add_artist(Line2D(
            [0.03, 0.975], [0.5, 0.5],
            transform=self.fig.transFigure,
            linestyle=(0, (7, 7)), linewidth=1.4, color="#a7b1bd", zorder=1,
        ))

        self.fig.text(0.5, 0.962, "UR5 + INSPIRE HAND  joint target monitor",
                      ha="center", fontsize=15, color=FG, weight="bold")
        self.status = self.fig.text(
            0.5, 0.932, "waiting for /hand_joint_targets , /arm_joint_targets ...",
            ha="center", fontsize=9.5, color=MUTED)

        # 角度条（一次性创建，之后只改宽度/文字）
        self.bars_arm, self.texts_arm = self._make_bars(
            self.ax_arm_bar, ARM_LABELS, ARM_ACCENT, (-200, 200))
        self.bars_hand, self.texts_hand = self._make_bars(
            self.ax_hand_bar, HAND_LABELS, HAND_ACCENT, (-6, 92))

        self.update()

    # -- 角度条 ------------------------------------------------------------
    @staticmethod
    def _make_bars(ax, labels, color, xlim):
        n = len(labels)
        ypos = list(range(n - 1, -1, -1))  # 第一个关节显示在最上面
        bars = ax.barh(ypos, [0.0] * n, height=0.56, color=color,
                       alpha=0.85, edgecolor="none", zorder=2)
        ax.set_yticks(ypos)
        ax.set_yticklabels(labels, fontsize=9, color=FG)
        ax.set_xlim(*xlim)
        ax.axvline(0.0, color="#c3ccd6", lw=1.0, zorder=1)
        ax.grid(True, axis="x", color=GRID, lw=0.6, zorder=0)
        ax.set_axisbelow(True)
        ax.tick_params(axis="x", colors=MUTED, labelsize=8)
        ax.set_title("joint angle (deg)", fontsize=10, color=MUTED,
                     loc="right", pad=4)
        texts = []
        for y in ypos:
            texts.append(ax.text(4, y, "0", va="center", ha="left",
                                 fontsize=9, color=FG))
        return bars, texts

    def _update_bars(self, bars, texts, values):
        for i, v in enumerate(values):
            deg = math.degrees(v)
            bars[i].set_width(deg)
            t = texts[i]
            t.set_text("%+0.0f°  (%.3f rad)" % (deg, v))
            if deg >= 0:
                t.set_x(deg + 3)
                t.set_ha("left")
            else:
                t.set_x(deg - 3)
                t.set_ha("right")

    # -- 机械臂侧视 --------------------------------------------------------
    def _draw_arm(self):
        ax = self.ax_arm
        ax.clear()
        ax.axhline(0.0, color=GRID, lw=1.2, zorder=0)
        pts = fk_ur5(self.node.arm)
        x, z = pts[:, 0], pts[:, 2]

        ax.add_patch(Polygon(
            [[-0.065, 0.0], [0.065, 0.0], [0.048, 0.052], [-0.048, 0.052]],
            closed=True, facecolor="#9aa5b1", edgecolor="#5c6975",
            lw=1.2, zorder=2))

        for i in range(6):
            ax.plot(x[i:i + 2], z[i:i + 2],
                    lw=10.0 - i * 0.75, solid_capstyle="round",
                    color=LINK_COLORS[i], zorder=3)

        ax.plot(x[:6], z[:6], "o", ms=6.5, mfc="white", mec="#2b3440",
                mew=1.4, zorder=4)
        ax.plot([x[6]], [z[6]], "s", ms=9, mfc="#2b3440", mec="white",
                mew=1.2, zorder=5)
        ax.annotate("TCP", (x[6], z[6]), textcoords="offset points",
                    xytext=(6, 4), fontsize=8, color=MUTED)

        ax.set_xlim(-0.86, 0.86)
        ax.set_ylim(-0.10, 1.02)
        ax.set_aspect("equal")
        ax.grid(True, color=GRID, lw=0.6)
        ax.set_axisbelow(True)
        ax.tick_params(colors=MUTED, labelsize=8)
        ax.set_title("ARM (UR5) - side view X-Z  |  m", fontsize=11,
                     color=ARM_ACCENT, loc="left", pad=6)
        ax.set_xlabel("x [m]", fontsize=8, color=MUTED)
        ax.set_ylabel("z [m]", fontsize=8, color=MUTED)

    # -- 底盘俯视（pan） ---------------------------------------------------
    def _draw_top(self):
        ax = self.ax_top
        ax.clear()
        ax.set_aspect("equal")
        ax.set_xlim(-1.3, 1.3)
        ax.set_ylim(-1.45, 1.25)
        ax.axis("off")
        ax.add_patch(Circle((0, 0), 1.0, fill=False, ec=GRID, lw=1.2,
                            ls=(0, (2, 3))))
        pan = self.node.arm[0]
        ax.annotate("", xy=(0.86 * math.cos(pan), 0.86 * math.sin(pan)),
                    xytext=(0, 0),
                    arrowprops=dict(arrowstyle="-|>", color="#d64545",
                                    lw=2.0, mutation_scale=13))
        ax.plot([0], [0], "o", ms=6, mfc="#2b3440", mec="white", zorder=5)
        ax.text(0.0, -1.30, "pan %+0.0f°" % math.degrees(pan),
                ha="center", fontsize=9.5, color=FG)
        ax.set_title("top view", fontsize=9, color=MUTED, pad=2)

    # -- 灵巧手 ------------------------------------------------------------
    def _draw_hand(self):
        ax = self.ax_hand
        ax.clear()
        ax.axis("off")
        ax.set_xlim(-1.30, 0.08)
        ax.set_ylim(-0.66, 1.42)

        # 掌心：竖直的圆角条，5 根手指都从它的右边缘长出来
        ax.add_patch(FancyBboxPatch(
            (-0.92, -0.62), 0.38, 1.96,
            boxstyle="round,pad=0.01,rounding_size=0.07",
            facecolor="#f6dcc4", edgecolor="#c99a6f", lw=1.4,
            zorder=2, alpha=0.95))
        ax.text(-0.73, 0.40, "PALM", rotation=90, ha="center", va="center",
                fontsize=7, color="#ad8259", zorder=3)

        # 四指：每根占一条横向泳道，0 弧度=水平伸直，角度越大越向上卷回
        lanes = [(1.14, 2, "index"), (0.78, 3, "middle"),
                 (0.42, 4, "ring"), (0.06, 5, "little")]
        for y, idx, name in lanes:
            a1 = self.node.hand[idx]
            a2 = a1 * MIMIC
            self._finger_lane(ax, y, a1, a2, 0.26, 0.19, name)

        # 拇指：两个关节各自独立（thumb_1 / thumb_2）
        self._finger_lane(ax, -0.40, self.node.hand[0], self.node.hand[1],
                          0.21, 0.15, "thumb")

        ax.set_title("HAND - finger curl  (0 rad = straight, larger = curled)",
                     fontsize=11, color=HAND_ACCENT, loc="left", pad=6)

    def _finger_lane(self, ax, y, a1, a2, l1, l2, name):
        """在 y 高度画一条手指：第 1 节按 a1 抬起，第 2 节再按 a2 叠加弯曲。"""
        p0 = np.array([-0.54, y])
        p1 = p0 + l1 * np.array([math.cos(a1), math.sin(a1)])
        p2 = p1 + l2 * np.array([math.cos(a1 + a2), math.sin(a1 + a2)])

        ax.plot([-0.54, -0.02], [y, y], color="#e8ecf1", lw=1.0,
                linestyle=(0, (1, 2)), zorder=0)
        ax.text(-0.98, y, "%s  %.2f" % (name, a1), ha="right", va="center",
                fontsize=8, color=MUTED)
        self._finger(ax, [p0, p1, p2], "#b4602f")

    @staticmethod
    def _finger(ax, points, color):
        p = np.array(points)
        ax.plot(p[:, 0], p[:, 1], lw=7, solid_capstyle="round",
                color=color, zorder=3)
        ax.plot(p[:, 0], p[:, 1], "o", ms=5.5, mfc="white", mec=color,
                mew=1.6, zorder=4)

    # -- 刷新 --------------------------------------------------------------
    def update(self):
        self._draw_arm()
        self._draw_top()
        self._draw_hand()
        self._update_bars(self.bars_arm, self.texts_arm, self.node.arm)
        self._update_bars(self.bars_hand, self.texts_hand, self.node.hand)
        self.status.set_text(
            "ARM target: %s  @%s          |          HAND target: %s  @%s"
            % (self.node.labels["arm"], self.node.stamps["arm"],
               self.node.labels["hand"], self.node.stamps["hand"]))
        self.pulse(force=True)

    def pulse(self, force=False):
        if force:
            self.fig.canvas.draw()
        else:
            self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def save(self, path):
        self.fig.savefig(path, dpi=110, facecolor=self.fig.get_facecolor())


def main():
    snapshot = None
    duration = None
    argv = sys.argv[1:]
    i = 0
    while i < len(argv):
        if argv[i] == "--snapshot" and i + 1 < len(argv):
            snapshot = argv[i + 1]
            i += 2
        elif argv[i] == "--duration" and i + 1 < len(argv):
            duration = float(argv[i + 1])
            i += 2
        else:
            print(__doc__)
            return 1

    rclpy.init()
    node = Monitor()
    node.get_logger().info("matplotlib 后端: %s" % plt.get_backend())

    viewer = Viewer(node)
    plt.show(block=False)

    start = time.time()
    last_draw = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            # 节流：最多每秒重绘约 12 次，避免高频率 joint_states 把界面拖卡
            if node.dirty and (now - last_draw) > 0.08:
                node.dirty = False
                last_draw = now
                viewer.update()
                node.get_logger().info("界面已刷新")
                if snapshot:
                    viewer.save(snapshot)
                    node.get_logger().info("已保存截图: %s" % snapshot)
            if not plt.fignum_exists(viewer.fig.number):
                node.get_logger().info("窗口已关闭，退出")
                break
            viewer.pulse()
            time.sleep(0.02)
            if duration is not None and time.time() - start > duration:
                node.get_logger().info("达到 --duration %.1fs，退出" % duration)
                break
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C 退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close(viewer.fig)
    return 0


if __name__ == "__main__":
    sys.exit(main())
