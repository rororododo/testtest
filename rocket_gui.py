import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import math
import random

plt.style.use('dark_background')

DT = 0.05
MAX_HIST = 300

# =========================================================
#  시뮬레이션 모델 (Rocket + Reaction Wheel + PID)
# =========================================================

class Rocket:
    def __init__(self):
        self.altitude = 0.0
        self.velocity = 0.0
        self.time = 0.0
        self.roll = 0.0
        self.roll_rate = 0.0
        self.wind = 0.0
        self._wtimer = 0.0
        self._cwind = 0.0

    def update(self, dt, torque):
        T = 60.0
        t = self.time / T * math.pi
        t2 = (self.time + dt) / T * math.pi
        self.altitude = max(0.0, 10000.0 * math.sin(t))
        self.velocity = (max(0.0, 10000.0 * math.sin(t2)) - self.altitude) / dt
        self._wtimer += dt
        if self._wtimer > 2.0:
            self._wtimer = 0.0
            self._cwind = random.uniform(-3.0, 3.0)
        self.wind = self._cwind
        accel = (self.wind - torque) / 2.0
        self.roll_rate += accel * dt
        self.roll += self.roll_rate * dt
        self.time += dt

    def is_flying(self):
        return self.altitude >= 0 and self.time <= 65.0


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev = 0.0
        self.first = True

    def compute(self, sp, meas, dt):
        e = sp - meas
        self.integral += e * dt
        self.integral = max(-10.0, min(10.0, self.integral))
        d = 0.0 if self.first else (meas - self.prev) / dt
        self.prev = meas
        self.first = False
        out = self.kp * e + self.ki * self.integral - self.kd * d
        return max(-5.0, min(5.0, out))


class ReactionWheel:
    def __init__(self):
        self.speed = 0.0
        self.current_torque = 0.0

    def apply_torque(self, torque_req, dt):
        tau = 0.05
        self.current_torque += (torque_req - self.current_torque) * (dt / (tau + dt))
        accel = self.current_torque / 0.01
        self.speed += accel * dt
        self.speed = max(-1000.0, min(1000.0, self.speed))
        return self.current_torque

    def get_rpm(self):
        return self.speed * 60.0 / (2.0 * math.pi)


# =========================================================
#  GUI & 시각화 레이아웃
# =========================================================

fig = plt.figure(figsize=(14, 8))
fig.canvas.manager.set_window_title("Rocket Ground Station - 3D Attitude & Telemetry")
gs = gridspec.GridSpec(3, 2, width_ratios=[1.2, 1], hspace=0.35, wspace=0.25)

ax_3d = fig.add_subplot(gs[:, 0], projection='3d')
ax_alt = fig.add_subplot(gs[0, 1])
ax_roll = fig.add_subplot(gs[1, 1])
ax_rpm = fig.add_subplot(gs[2, 1])

# 데이터 버퍼
t_data = []
alt_data = []
roll_data = []
target_data = []
rpm_data = []

# 인스턴스 생성
rocket = Rocket()
pid = PID(kp=3.5, ki=0.8, kd=1.2)
wheel = ReactionWheel()

# 플롯 라인 초기화
line_alt, = ax_alt.plot([], [], color='#00d2ff', lw=2, label='Altitude (m)')
line_roll, = ax_roll.plot([], [], color='#ff007f', lw=2, label='Roll Angle (deg)')
line_target, = ax_roll.plot([], [], color='#ffff00', lw=1.5, ls='--', label='Target (0 deg)')
line_rpm, = ax_rpm.plot([], [], color='#00ff88', lw=2, label='RW Speed (RPM)')

for ax, title, ylabel in [
    (ax_alt, "Altitude Telemetry", "Altitude (m)"),
    (ax_roll, "Roll Attitude & PID Control", "Angle (deg)"),
    (ax_rpm, "Reaction Wheel Dynamics", "RPM")
]:
    ax.set_title(title, fontsize=10, fontweight='bold', color='#e0e0e0', pad=6)
    ax.set_ylabel(ylabel, fontsize=8, color='#aaaaaa')
    ax.grid(True, linestyle=':', alpha=0.4, color='#555555')
    ax.tick_params(colors='#888888', labelsize=8)
    ax.legend(loc='upper right', fontsize=8, facecolor='#222222', edgecolor='none')

ax_rpm.set_xlabel("Time (s)", fontsize=8, color='#aaaaaa')

# 3D 로켓 렌더링 함수
def draw_3d_rocket(ax, roll_rad):
    ax.cla()
    ax.set_facecolor('#121212')
    
    # 원통 바디
    z = np.linspace(-3, 3, 20)
    theta = np.linspace(0, 2*np.pi, 20) + roll_rad
    theta_grid, z_grid = np.meshgrid(theta, z)
    r = 0.6
    x_grid = r * np.cos(theta_grid)
    y_grid = r * np.sin(theta_grid)
    ax.plot_surface(x_grid, y_grid, z_grid, color='#4488ff', alpha=0.85, edgecolor='none')

    # 노즈 콘
    z_cone = np.linspace(3, 4.5, 12)
    theta_cone = np.linspace(0, 2*np.pi, 20) + roll_rad
    t_c, z_c = np.meshgrid(theta_cone, z_cone)
    r_c = 0.6 * (1.0 - (z_c - 3) / 1.5)
    x_c = r_c * np.cos(t_c)
    y_c = r_c * np.sin(t_c)
    ax.plot_surface(x_c, y_c, z_c, color='#ff3366', alpha=0.9, edgecolor='none')

    # 4개 핀 (Fins)
    fin_angles = [0, np.pi/2, np.pi, 3*np.pi/2]
    for fa in fin_angles:
        ang = fa + roll_rad
        fx = [0.6 * np.cos(ang), 1.4 * np.cos(ang), 1.4 * np.cos(ang), 0.6 * np.cos(ang)]
        fy = [0.6 * np.sin(ang), 1.4 * np.sin(ang), 1.4 * np.sin(ang), 0.6 * np.sin(ang)]
        fz = [-2.8, -3.0, -2.0, -1.5]
        ax.plot(fx, fy, fz, color='#00ffcc', lw=3)

    # 중심축 & 그리드 설정
    ax.set_xlim([-2.5, 2.5])
    ax.set_ylim([-2.5, 2.5])
    ax.set_zlim([-3.5, 5.0])
    ax.set_axis_off()
    
    # 텍스트 HUD
    status = f"Time: {rocket.time:.1f}s | Alt: {rocket.altitude:.0f}m\nRoll: {math.degrees(rocket.roll):.1f} deg | Wind: {rocket.wind:.2f} Nm\nRW: {wheel.get_rpm():.0f} RPM"
    ax.text2D(0.05, 0.92, status, transform=ax.transAxes, color='#00ffcc', fontsize=10, family='monospace',
              bbox=dict(boxstyle='round,pad=0.5', facecolor='#1e1e1e', alpha=0.8, edgecolor='#333333'))
    ax.set_title("[3D Rocket Attitude Realtime View]", fontsize=12, fontweight='bold', color='#ffffff', pad=10)


def update_frame(frame):
    # 시뮬레이션 계산
    torque_cmd = pid.compute(0.0, rocket.roll, DT)
    applied_torque = wheel.apply_torque(torque_cmd, DT)
    rocket.update(DT, applied_torque)

    # 데이터 누적
    t_data.append(rocket.time)
    alt_data.append(rocket.altitude)
    roll_data.append(math.degrees(rocket.roll))
    target_data.append(0.0)
    rpm_data.append(wheel.get_rpm())

    if len(t_data) > MAX_HIST:
        t_data.pop(0)
        alt_data.pop(0)
        roll_data.pop(0)
        target_data.pop(0)
        rpm_data.pop(0)

    # 3D 뷰 업데이트
    draw_3d_rocket(ax_3d, rocket.roll)

    # 그래프 업데이트
    line_alt.set_data(t_data, alt_data)
    line_roll.set_data(t_data, roll_data)
    line_target.set_data(t_data, target_data)
    line_rpm.set_data(t_data, rpm_data)

    for ax, data_list in [(ax_alt, alt_data), (ax_roll, roll_data), (ax_rpm, rpm_data)]:
        if t_data:
            ax.set_xlim(max(0, t_data[-1] - 15), max(15, t_data[-1] + 1))
            if data_list:
                ymin, ymax = min(data_list), max(data_list)
                margin = max(1.0, (ymax - ymin) * 0.15)
                ax.set_ylim(ymin - margin, ymax + margin)

    return line_alt, line_roll, line_target, line_rpm

# FuncAnimation 실행 (전역 변수로 유지)
anim = animation.FuncAnimation(fig, update_frame, interval=50, blit=False)

print("[INFO] Launching Ground Station 3D Simulation...")
plt.tight_layout()
plt.show()
