import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import math
import random

plt.style.use('dark_background')

DT = 0.05
MAX_HIST = 300

# =========================================================
#  위성 물리 & 제어 모델 (Satellite + 3-Axis RW + Sun Tracking)
# =========================================================

class Satellite:
    def __init__(self):
        self.time = 0.0
        self.altitude = 550.0  # LEO 궤도 고도 (km)
        self.velocity = 7.59   # 궤도 속도 (km/s)
        
        # 자세 각도 (Roll, Pitch, Yaw in rad)
        self.roll = 0.2
        self.pitch = -0.15
        self.yaw = 0.0
        
        # 각속도 (rad/s)
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        
        # 환경 외란 토크 (태양풍 / 자기장 외란, Nm)
        self.dist_torque = 0.0
        self._dtimer = 0.0
        
        # 전력 시스템
        self.battery = 85.0  # %
        self.solar_power = 45.0  # Watts

    def update(self, dt, rw_torque_roll, rw_torque_pitch):
        self.time += dt
        
        # 간헐적 우주 환경 외란
        self._dtimer += dt
        if self._dtimer > 3.0:
            self._dtimer = 0.0
            self.dist_torque = random.uniform(-0.8, 0.8)
        
        # 롤 & 피치 동역학
        accel_roll = (self.dist_torque - rw_torque_roll) / 1.5
        accel_pitch = (-self.dist_torque * 0.5 - rw_torque_pitch) / 1.5
        
        self.roll_rate += accel_roll * dt
        self.roll += self.roll_rate * dt
        
        self.pitch_rate += accel_pitch * dt
        self.pitch += self.pitch_rate * dt
        
        # 태양광 발전 효율 (자세가 0에 가까울수록 태양 정면 지향 -> 최대 발전)
        pointing_error = math.sqrt(self.roll**2 + self.pitch**2)
        self.solar_power = max(5.0, 50.0 * math.cos(min(math.pi/2, pointing_error)))
        self.battery = min(100.0, max(0.0, self.battery + (self.solar_power - 20.0) * dt * 0.05))


class PID:
    def __init__(self, kp, ki, kd, max_out=4.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.integral = 0.0
        self.prev_meas = 0.0
        self.first = True

    def compute(self, target, measured, dt):
        error = target - measured
        self.integral += error * dt
        self.integral = max(-5.0, min(5.0, self.integral))
        
        derivative = 0.0 if self.first else (measured - self.prev_meas) / dt
        self.prev_meas = measured
        self.first = False
        
        output = self.kp * error + self.ki * self.integral - self.kd * derivative
        return max(-self.max_out, min(self.max_out, output))


class ReactionWheel:
    def __init__(self, inertia=0.02):
        self.speed = 0.0  # rad/s
        self.torque = 0.0
        self.inertia = inertia

    def apply(self, torque_req, dt):
        tau = 0.03
        self.torque += (torque_req - self.torque) * (dt / (tau + dt))
        self.speed += (self.torque / self.inertia) * dt
        self.speed = max(-800.0, min(800.0, self.speed))
        return self.torque

    def get_rpm(self):
        return self.speed * 60.0 / (2.0 * math.pi)


# =========================================================
#  GUI & 3D 위성 시각화 대시보드
# =========================================================

fig = plt.figure(figsize=(15, 8.5))
fig.canvas.manager.set_window_title("CubeSat Ground Station - 3D Attitude & Power Telemetry")
gs = gridspec.GridSpec(3, 2, width_ratios=[1.3, 1], hspace=0.35, wspace=0.25)

ax_3d = fig.add_subplot(gs[:, 0], projection='3d')
ax_att = fig.add_subplot(gs[0, 1])
ax_rpm = fig.add_subplot(gs[1, 1])
ax_pwr = fig.add_subplot(gs[2, 1])

# 시뮬레이션 및 제어기 인스턴스
sat = Satellite()
pid_roll = PID(kp=4.2, ki=0.5, kd=1.8)
pid_pitch = PID(kp=4.2, ki=0.5, kd=1.8)
rw_x = ReactionWheel()
rw_y = ReactionWheel()

# 시계열 데이터 버퍼
t_buf = []
roll_buf = []
pitch_buf = []
rpm_x_buf = []
rpm_y_buf = []
power_buf = []
battery_buf = []

# 플롯 라인 초기화
line_roll, = ax_att.plot([], [], color='#00d2ff', lw=2, label='Roll Error (deg)')
line_pitch, = ax_att.plot([], [], color='#ff007f', lw=2, label='Pitch Error (deg)')
line_target, = ax_att.plot([], [], color='#ffffff', lw=1, ls='--', label='Target (0 deg)')

line_rpm_x, = ax_rpm.plot([], [], color='#00ff88', lw=2, label='RW-X (RPM)')
line_rpm_y, = ax_rpm.plot([], [], color='#ffaa00', lw=2, label='RW-Y (RPM)')

line_pwr, = ax_pwr.plot([], [], color='#ffee00', lw=2, label='Solar Gen (W)')
line_bat, = ax_pwr.plot([], [], color='#00e5ff', lw=1.5, ls=':', label='Battery (%)')

# 서브플롯 스타일 설정
configs = [
    (ax_att, "Satellite Attitude Pointing [Sun Tracking]", "Angle Error (deg)"),
    (ax_rpm, "Reaction Wheels Momentum Management", "Wheel Speed (RPM)"),
    (ax_pwr, "EPS Telemetry (Power & Battery)", "Watts / %")
]
for ax, title, ylabel in configs:
    ax.set_title(title, fontsize=10, fontweight='bold', color='#e0e0e0', pad=6)
    ax.set_ylabel(ylabel, fontsize=8, color='#aaaaaa')
    ax.grid(True, linestyle=':', alpha=0.35, color='#666666')
    ax.tick_params(colors='#888888', labelsize=8)
    ax.legend(loc='upper right', fontsize=8, facecolor='#1f1f1f', edgecolor='#333333')

ax_pwr.set_xlabel("Orbit Time (s)", fontsize=8, color='#aaaaaa')


def rotate_point(p, r, p_angle, y):
    # Rz(y) * Ry(p) * Rx(r) 회전 변환
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p_angle), math.sin(p_angle)
    cy, sy = math.cos(y), math.sin(y)
    
    # Rx
    x1 = p[0]
    y1 = cr * p[1] - sr * p[2]
    z1 = sr * p[1] + cr * p[2]
    
    # Ry
    x2 = cp * x1 + sp * z1
    y2 = y1
    z2 = -sp * x1 + cp * z1
    
    # Rz
    x3 = cy * x2 - sy * y2
    y3 = sy * x2 + cy * y2
    z3 = z2
    return np.array([x3, y3, z3])


def draw_3d_cubesat(ax, roll, pitch, yaw):
    ax.cla()
    ax.set_facecolor('#0d1117')
    
    # 1. 큐브위성 본체 (골드박스 3U CubeSat: 1.0 x 1.0 x 1.8)
    dx, dy, dz = 0.7, 0.7, 1.2
    verts_base = [
        [-dx, -dy, -dz], [dx, -dy, -dz], [dx, dy, -dz], [-dx, dy, -dz],
        [-dx, -dy,  dz], [dx, -dy,  dz], [dx, dy,  dz], [-dx, dy,  dz]
    ]
    # 회전 적용
    rotated_verts = [rotate_point(v, roll, pitch, yaw) for v in verts_base]
    
    # 6개 면 정의
    faces = [
        [rotated_verts[0], rotated_verts[1], rotated_verts[2], rotated_verts[3]], # 밑면
        [rotated_verts[4], rotated_verts[5], rotated_verts[6], rotated_verts[7]], # 윗면
        [rotated_verts[0], rotated_verts[1], rotated_verts[5], rotated_verts[4]], # 앞면
        [rotated_verts[2], rotated_verts[3], rotated_verts[7], rotated_verts[6]], # 뒷면
        [rotated_verts[1], rotated_verts[2], rotated_verts[6], rotated_verts[5]], # 우측
        [rotated_verts[0], rotated_verts[3], rotated_verts[7], rotated_verts[4]], # 좌측
    ]
    poly = Poly3DCollection(faces, facecolors='#e5a93c', linewidths=1.2, edgecolors='#ffd700', alpha=0.85)
    ax.add_collection3d(poly)
    
    # 2. 태양광 패널 (좌우 날개 펼침)
    panel_left = [
        [-dx, -dy*0.9, -dz*0.8], [-dx - 1.8, -dy*0.9, -dz*0.8],
        [-dx - 1.8,  dy*0.9, -dz*0.8], [-dx,  dy*0.9, -dz*0.8]
    ]
    panel_right = [
        [dx, -dy*0.9, -dz*0.8], [dx + 1.8, -dy*0.9, -dz*0.8],
        [dx + 1.8,  dy*0.9, -dz*0.8], [dx,  dy*0.9, -dz*0.8]
    ]
    p_left_rot = [rotate_point(v, roll, pitch, yaw) for v in panel_left]
    p_right_rot = [rotate_point(v, roll, pitch, yaw) for v in panel_right]
    
    poly_panels = Poly3DCollection([p_left_rot, p_right_rot], facecolors='#0055ff', linewidths=1.5, edgecolors='#00d2ff', alpha=0.95)
    ax.add_collection3d(poly_panels)
    
    # 3. 통신 안테나 (상단 붐)
    ant_start = rotate_point([0, 0, dz], roll, pitch, yaw)
    ant_end = rotate_point([0, 0, dz + 0.9], roll, pitch, yaw)
    ax.plot([ant_start[0], ant_end[0]], [ant_start[1], ant_end[1]], [ant_start[2], ant_end[2]], color='#00ffcc', lw=3)
    
    # 태양 방향 화살표 지시선 (항상 +Z 축 상공)
    ax.quiver(0, 0, 2.5, 0, 0, 1.2, color='#ffff00', arrow_length_ratio=0.3, lw=2)
    ax.text(0, 0, 3.8, "SUN Vector", color='#ffff00', fontsize=9, fontweight='bold', ha='center')

    # 축 및 범위 설정
    ax.set_xlim([-3.0, 3.0])
    ax.set_ylim([-3.0, 3.0])
    ax.set_zlim([-3.0, 4.0])
    ax.set_axis_off()
    
    # HUD 텔레메트리 오버레이
    hud = (
        f"Orbit: LEO {sat.altitude:.0f}km ({sat.velocity:.2f} km/s)\n"
        f"Attitude Error: Roll {math.degrees(sat.roll):+.1f} deg | Pitch {math.degrees(sat.pitch):+.1f} deg\n"
        f"Power Gen: {sat.solar_power:.1f}W | Battery: {sat.battery:.1f}%\n"
        f"RW-X: {rw_x.get_rpm():+.0f} RPM | RW-Y: {rw_y.get_rpm():+.0f} RPM"
    )
    ax.text2D(0.04, 0.92, hud, transform=ax.transAxes, color='#00ffcc', fontsize=9.5, family='monospace',
              bbox=dict(boxstyle='round,pad=0.5', facecolor='#161b22', alpha=0.85, edgecolor='#30363d'))
    ax.set_title("[3D CubeSat Sun-Pointing Realtime Simulation]", fontsize=11, fontweight='bold', color='#ffffff', pad=10)


def update_frame(frame):
    # 태양 정밀 지향 제어 (목표 각도: 0, 0)
    t_roll = pid_roll.compute(0.0, sat.roll, DT)
    t_pitch = pid_pitch.compute(0.0, sat.pitch, DT)
    
    rw_torq_x = rw_x.apply(t_roll, DT)
    rw_torq_y = rw_y.apply(t_pitch, DT)
    
    sat.update(DT, rw_torq_x, rw_torq_y)
    
    # 데이터 버퍼 추가
    t_buf.append(sat.time)
    roll_buf.append(math.degrees(sat.roll))
    pitch_buf.append(math.degrees(sat.pitch))
    rpm_x_buf.append(rw_x.get_rpm())
    rpm_y_buf.append(rw_y.get_rpm())
    power_buf.append(sat.solar_power)
    battery_buf.append(sat.battery)
    
    if len(t_buf) > MAX_HIST:
        for b in [t_buf, roll_buf, pitch_buf, rpm_x_buf, rpm_y_buf, power_buf, battery_buf]:
            b.pop(0)
            
    # 3D 뷰 업데이트
    draw_3d_cubesat(ax_3d, sat.roll, sat.pitch, sat.yaw)
    
    # 2D 차트 업데이트
    line_roll.set_data(t_buf, roll_buf)
    line_pitch.set_data(t_buf, pitch_buf)
    line_target.set_data(t_buf, [0.0]*len(t_buf))
    
    line_rpm_x.set_data(t_buf, rpm_x_buf)
    line_rpm_y.set_data(t_buf, rpm_y_buf)
    
    line_pwr.set_data(t_buf, power_buf)
    line_bat.set_data(t_buf, battery_buf)
    
    for ax, data_group in [
        (ax_att, [roll_buf, pitch_buf]),
        (ax_rpm, [rpm_x_buf, rpm_y_buf]),
        (ax_pwr, [power_buf, battery_buf])
    ]:
        if t_buf:
            ax.set_xlim(max(0, t_buf[-1] - 15), max(15, t_buf[-1] + 1))
            all_vals = [v for sublist in data_group for v in sublist]
            if all_vals:
                ymin, ymax = min(all_vals), max(all_vals)
                margin = max(2.0, (ymax - ymin) * 0.15)
                ax.set_ylim(ymin - margin, ymax + margin)

    return line_roll, line_pitch, line_rpm_x, line_rpm_y, line_pwr, line_bat


anim = animation.FuncAnimation(fig, update_frame, interval=50, blit=False)

print("[INFO] Launching Satellite Ground Station Simulation...")
plt.tight_layout()
plt.show()
