#!/usr/bin/env python3
"""
mpc_debug_monitor.py  —  MPC 실시간 단일 패널 진단 그래프
-----------------------------------------------------------
구독 토픽:
  /cav01/mpc_performance  (bisa/MPCPerformance)
  /cav01/accel_cmd        (geometry_msgs/Accel)  linear.x=v  angular.z=w
  /cav01/local_path       (nav_msgs/Path)

단일 그래프:
  • v_cmd  파란 선  (주 y축, m/s)
  • |w_cmd| 주황 점선 (주 y축, rad/s)
  • solver_time  보라 bar (우측 y축, ms)
  • 배경색: 초록=SOLVED  노랑=PATH_ISSUE  빨강=FAILED/NO_DATA
  • 정지 이벤트: 빨간 수직선 + 원인 라벨 (자동 감지)
  • 우상단 현재 상태 박스

실행:
  ros2 run bisa mpc_debug_monitor.py          (런치에서 자동 실행)
  python3 scripts/mpc_debug_monitor.py        (단독 실행)
"""

import collections, os, sys, threading, time

# ── matplotlib 백엔드 자동 선택 ──────────────────────────────────────────
_DISPLAY = os.environ.get("DISPLAY", "")
if _DISPLAY:
    try:
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as _plt_test
        _plt_test.figure(); _plt_test.close()
        _BACKEND = "TkAgg"
    except Exception:
        try:
            matplotlib.use("Qt5Agg")
            _BACKEND = "Qt5Agg"
        except Exception:
            matplotlib.use("Agg")
            _BACKEND = "Agg"   # headless → 파일 저장
else:
    import matplotlib
    matplotlib.use("Agg")
    _BACKEND = "Agg"

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import matplotlib.animation as animation
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Accel
from nav_msgs.msg import Path

try:
    from bisa.msg import MPCPerformance
    _HAS_PERF = True
except ImportError:
    _HAS_PERF = False

# ── 설정 ────────────────────────────────────────────────────────────────────
CAV_ID       = "cav01"
WINDOW_S     = 30.0     # 그래프 표시 시간 윈도우 (초)
UPDATE_HZ    = 8.0      # 화면 갱신 주기
MAX_BUF      = 4000     # 버퍼 크기
FILE_OUT     = "/tmp/mpc_debug_live.png"  # Agg 모드 저장 경로
FILE_SAVE_HZ = 1.0      # Agg 모드 저장 주기 (초)

# 상태별 배경/라벨 색상
BG = {
    "SOLVED":               "#0a2010",
    "LEGACY":               "#0a1020",
    "LOCAL_PATH_TOO_SHORT": "#2a1800",
    "NO_DATA":              "#2a0808",
    "FAILED":               "#3a0000",
    "UNKNOWN":              "#181018",
}
FG = {
    "SOLVED":               "#00e060",
    "LEGACY":               "#4488ff",
    "LOCAL_PATH_TOO_SHORT": "#ffbb00",
    "NO_DATA":              "#ff5555",
    "FAILED":               "#ff2222",
    "UNKNOWN":              "#cc88ff",
}
ANOMALY = {"FAILED", "NO_DATA", "LOCAL_PATH_TOO_SHORT"}
FREEZE_THRESH_S  = 0.8   # v < 0.005 m/s 이 시간 이상 → 정지 이벤트
FREEZE_V_THRESH  = 0.005 # m/s
DEVIATION_W_THRESH  = 0.80  # |w_cmd| 이 값 초과 시 이탈 경고 (rad/s)
SOLVER_WARN_MS      = 30.0  # solver time 경고 임계 (ms)
SOLVER_FAIL_MS      = 55.0  # solver time FAILED 예상 임계 (ms)


# ── ROS2 노드 ────────────────────────────────────────────────────────────────
class MonitorNode(Node):
    def __init__(self):
        super().__init__("mpc_debug_monitor")
        self._lock = threading.Lock()
        self._t0   = time.monotonic()

        # 데이터 버퍼
        self.t_buf   = collections.deque(maxlen=MAX_BUF)
        self.v_buf   = collections.deque(maxlen=MAX_BUF)
        self.w_buf   = collections.deque(maxlen=MAX_BUF)
        self.st_buf  = collections.deque(maxlen=MAX_BUF)  # solver_status str
        self.ms_buf  = collections.deque(maxlen=MAX_BUF)  # solver ms
        self.ps_buf  = collections.deque(maxlen=MAX_BUF)  # path size

        # 현재값
        self._last_status = "UNKNOWN"
        self._last_ms     = 0.0
        self._last_ps     = 0

        # 이벤트 로그: [(t, label, color)]
        self.events: list = []

        # 정지 감지 상태
        self._freeze_t0      = None
        self._freeze_pos_t   = None
        self._freeze_done    = False

        # 직전 상태 (변화 감지)
        self._prev_status = "UNKNOWN"

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(Accel, f"/{CAV_ID}/accel_cmd", self._accel_cb, qos)
        self.create_subscription(Path,  f"/{CAV_ID}/local_path", self._path_cb,  qos)
        if _HAS_PERF:
            self.create_subscription(MPCPerformance,
                                     f"/{CAV_ID}/mpc_performance",
                                     self._perf_cb, qos)

        self.get_logger().info(
            f"[mpc_debug_monitor] backend={_BACKEND}  monitoring /{CAV_ID}/*")
        if _BACKEND == "Agg":
            self.get_logger().warn(
                f"  No display — saving PNG to {FILE_OUT} @ {FILE_SAVE_HZ:.0f} Hz")

    # ── 콜백 ──────────────────────────────────────────────────────────────
    def _now(self):
        return time.monotonic() - self._t0

    def _accel_cb(self, msg: Accel):
        t = self._now()
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        with self._lock:
            self.t_buf.append(t)
            self.v_buf.append(v)
            self.w_buf.append(w)
            self.st_buf.append(self._last_status)
            self.ms_buf.append(self._last_ms)
            self.ps_buf.append(self._last_ps)

            # 이탈 감지 (|w_cmd| 급등)
            if abs(w) > DEVIATION_W_THRESH and abs(v) > FREEZE_V_THRESH:
                # 직전 이벤트와 1.5s 이상 떨어진 경우만 기록
                last_dev = next((e for e in reversed(self.events) if "DEVIATION" in e[1]), None)
                if last_dev is None or (t - last_dev[0]) > 1.5:
                    self._log_event(t, f"DEVIATION |ω|={abs(w):.2f}", "#ff6b6b")

            # 정지 감지
            if abs(v) < FREEZE_V_THRESH:
                if self._freeze_t0 is None:
                    self._freeze_t0   = t
                    self._freeze_done = False
                elif (not self._freeze_done and
                      (t - self._freeze_t0) >= FREEZE_THRESH_S):
                    cause = self._freeze_cause()
                    self._log_event(self._freeze_t0,
                                    f"■ FREEZE  {cause}",
                                    "#facc15")
                    self._freeze_done = True
            else:
                self._freeze_t0   = None
                self._freeze_done = False

    def _freeze_cause(self) -> str:
        """현재 상태에서 정지 원인 문자열 반환"""
        s = self._last_status
        if s in ("FAILED",):
            return "QP FAILED (lateral_bound / infeasible)"
        if s == "NO_DATA":
            return "NO_DATA (path/pose not received)"
        if s == "LOCAL_PATH_TOO_SHORT":
            return "PATH < 5 pts"
        if self._last_ps < 5:
            return f"PATH EMPTY ({self._last_ps} pts)"
        return f"v=0  [{s}]"

    def _perf_cb(self, msg):
        with self._lock:
            self._last_status = msg.solver_status
            self._last_ms     = msg.total_time_us / 1000.0
            if msg.solver_status != self._prev_status:
                t = self._now()
                if msg.solver_status in ANOMALY:
                    self._log_event(t, msg.solver_status,
                                    FG.get(msg.solver_status, "white"))
                self._prev_status = msg.solver_status

    def _path_cb(self, msg: Path):
        with self._lock:
            self._last_ps = len(msg.poses)

    def _log_event(self, t, label, color):
        self.events.append((t, label, color))
        if len(self.events) > 30:
            self.events.pop(0)

    # ── 스냅샷 ────────────────────────────────────────────────────────────
    def snapshot(self):
        with self._lock:
            return (list(self.t_buf), list(self.v_buf), list(self.w_buf),
                    list(self.st_buf), list(self.ms_buf), list(self.ps_buf),
                    list(self.events),
                    self._last_status, self._last_ms, self._last_ps)


# ── 그래프 빌드 ──────────────────────────────────────────────────────────────
def build_graph(node: MonitorNode):
    """단일 패널 실시간 그래프 생성. (fig, update_fn) 반환"""

    fig, ax = plt.subplots(figsize=(15, 5))
    fig.patch.set_facecolor("#0d1117")
    ax.set_facecolor("#0d1117")
    ax.tick_params(colors="white", labelsize=9)
    for sp in ax.spines.values():
        sp.set_edgecolor("#2d3748")
    ax.set_xlabel("Time (s)", color="white", fontsize=10)
    ax.set_ylabel("v (m/s)  /  |ω| (rad/s)", color="white", fontsize=10)
    ax.set_title(
        f"MPC Debug Monitor  [{CAV_ID}]  —  sigmoid v-profile  |  deviation & freeze 실시간 진단",
        color="white", fontsize=11, fontweight="bold", pad=8)
    ax.grid(True, color="#2d3748", lw=0.5, alpha=0.6)

    ax2 = ax.twinx()
    ax2.set_ylabel("solver (ms)", color="#666", fontsize=8)
    ax2.tick_params(colors="#555", labelsize=7)
    for sp in ax2.spines.values():
        sp.set_edgecolor("#222")

    line_v, = ax.plot([], [], color="#38bdf8", lw=2.0, label="v_cmd (m/s)", zorder=4)
    line_w, = ax.plot([], [], color="#fb923c", lw=1.3, ls="--",
                       label="|ω_cmd| (rad/s)", zorder=4)

    # 범례
    # 이탈 임계선
    ax.axhline(DEVIATION_W_THRESH, color="#ff6b6b", lw=0.8, ls=":",
               alpha=0.6, label=f"|ω| deviation threshold ({DEVIATION_W_THRESH} rad/s)", zorder=2)

    handles = [
        mpatches.Patch(color="#38bdf8", label="v_cmd sigmoid (m/s)"),
        mpatches.Patch(color="#fb923c", label="|ω_cmd| (rad/s)"),
        mpatches.Patch(color="#ff6b6b", label=f"─ ─  deviation |ω|>{DEVIATION_W_THRESH}"),
        mpatches.Patch(color="#00e060", label="SOLVED"),
        mpatches.Patch(color="#ffbb00", label="PATH_TOO_SHORT"),
        mpatches.Patch(color="#ff2222", label="FAILED / NO_DATA"),
        mpatches.Patch(color="#facc15", label="■ FREEZE event"),
        mpatches.Patch(color="#6366f1", label="solver time (ms)"),
    ]
    ax.legend(handles=handles, loc="upper left", fontsize=7.5,
              facecolor="#1a1a2e", labelcolor="white", framealpha=0.85)

    # 우상단 상태 박스
    sbox = ax.text(
        0.995, 0.98, "", transform=ax.transAxes,
        fontsize=9, fontweight="bold", va="top", ha="right",
        fontfamily="monospace",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="#111122",
                  edgecolor="#444", alpha=0.92),
        zorder=10)

    # 동적 요소 저장 리스트
    _bg_spans    = []
    _ev_artists  = []  # (vline, text)
    _bars        = []

    def _clear_dynamic():
        for s in _bg_spans:  s.remove()
        _bg_spans.clear()
        for ln, tx in _ev_artists:
            try: ln.remove()
            except Exception: pass
            try: tx.remove()
            except Exception: pass
        _ev_artists.clear()
        for b in _bars:
            try: b.remove()
            except Exception: pass
        _bars.clear()

    def update(_):
        (ts, vs, ws, sts, sms, pss, events,
         last_st, last_ms, last_ps) = node.snapshot()

        if len(ts) < 2:
            return

        ts  = np.asarray(ts,  dtype=float)
        vs  = np.asarray(vs,  dtype=float)
        ws  = np.abs(ws)
        sms = np.asarray(sms, dtype=float)

        t_now = ts[-1]
        t_lo  = t_now - WINDOW_S
        m     = ts >= t_lo

        tw = ts[m];  vw = vs[m];  ww = ws[m]
        smw = sms[m]; stw = [sts[i] for i in range(len(ts)) if m[i]]

        if len(tw) < 2:
            return

        _clear_dynamic()

        # ── 배경 (상태별 색상) ───────────────────────────────────────────
        seg_s = tw[0]; seg_st = stw[0]
        for i in range(1, len(tw)):
            changed = (stw[i] != seg_st) or (i == len(tw) - 1)
            if changed:
                t_end = tw[i]
                col   = BG.get(seg_st, "#111111")
                sp = ax.axvspan(seg_s, t_end, facecolor=col, alpha=0.6, zorder=0)
                _bg_spans.append(sp)
                seg_s = tw[i]; seg_st = stw[i]

        # ── 이벤트 수직선 + 라벨 ────────────────────────────────────────
        y_top = max(0.55, float(np.nanmax(vw)) * 1.15,
                    float(np.nanmax(ww)) * 1.15) if len(vw) else 0.6
        for ev_t, ev_label, ev_col in events:
            if ev_t < t_lo:
                continue
            ln = ax.axvline(ev_t, color=ev_col, lw=2.0, ls="-", zorder=5, alpha=0.9)
            is_freeze = "FREEZE" in ev_label
            tx = ax.text(
                ev_t + 0.15,
                y_top * (0.97 if is_freeze else 0.75),
                ev_label,
                color=ev_col,
                fontsize=7 if is_freeze else 6.5,
                fontweight="bold" if is_freeze else "normal",
                fontfamily="monospace",
                rotation=90, va="top", zorder=6,
                bbox=dict(boxstyle="round,pad=0.15",
                          facecolor="#000000", alpha=0.55,
                          edgecolor="none") if is_freeze else None,
            )
            _ev_artists.append((ln, tx))

        # ── solver time bar ──────────────────────────────────────────────
        ax2.cla()
        ax2.set_facecolor("none")
        ax2.tick_params(colors="#555", labelsize=7)
        ax2.set_ylabel("solver (ms)", color="#555", fontsize=7)
        for sp in ax2.spines.values():
            sp.set_edgecolor("#222")
        if len(tw) > 1:
            bw = (tw[-1] - tw[0]) / max(1, len(tw)) * 0.85
            # bar 색상: 초록<30ms, 노랑30-55ms, 빨강>55ms
            bar_colors = ["#22c55e" if v < SOLVER_WARN_MS
                          else "#f59e0b" if v < SOLVER_FAIL_MS
                          else "#ef4444"
                          for v in smw]
            bars = ax2.bar(tw, smw, width=bw, color=bar_colors, alpha=0.50, zorder=1)
            _bars.extend(bars)
        ms_max = max(80.0, float(np.nanmax(smw)) * 1.4) if len(smw) else 80.0
        ax2.set_ylim(0, ms_max)
        ax2.axhline(SOLVER_WARN_MS, color="#f59e0b", lw=0.8, ls="--", alpha=0.7)
        ax2.axhline(SOLVER_FAIL_MS, color="#ef4444", lw=0.8, ls="--", alpha=0.7)

        # ── 주 라인 ──────────────────────────────────────────────────────
        line_v.set_data(tw, vw)
        line_w.set_data(tw, ww)

        # ── 축 범위 ──────────────────────────────────────────────────────
        ax.set_xlim(t_lo, t_now + 0.3)
        v_hi = max(0.6, float(np.nanmax(vw)) * 1.25,
                   float(np.nanmax(ww)) * 1.25) if len(vw) else 0.6
        ax.set_ylim(-0.03, v_hi)

        # ── 상태 박스 ────────────────────────────────────────────────────
        col  = FG.get(last_st, "white")
        v_now = float(vs[-1]) if len(vs) else 0.0
        w_now = float(abs(ws[-1])) if len(ws) else 0.0
        ps_ok = "OK" if last_ps >= 5 else f"!! {last_ps} pts"
        # FAILED 카운트
        fail_count = sum(1 for s in stw if s in ANOMALY)
        fail_rate  = fail_count / max(1, len(stw)) * 100

        sbox.set_text(
            f" STATUS  : {last_st}\n"
            f" v_cmd   : {v_now:+.3f} m/s\n"
            f" |ω_cmd| : {w_now:.3f} rad/s\n"
            f" solver  : {last_ms:.1f} ms"
            + (" ⚠" if last_ms > SOLVER_WARN_MS else "") + "\n"
            f" path    : {last_ps}  [{ps_ok}]\n"
            f" FAILED  : {fail_count}/{len(stw)} ({fail_rate:.0f}%)")
        sbox.set_color(col)
        sbox.get_bbox_patch().set_edgecolor(col)

        fig.canvas.draw_idle()

    plt.tight_layout(pad=0.6)
    return fig, update


# ── 진입점 ───────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()

    spin_th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_th.start()

    fig, update_fn = build_graph(node)

    if _BACKEND != "Agg":
        # 실시간 GUI
        ani = animation.FuncAnimation(
            fig, update_fn,
            interval=int(1000 / UPDATE_HZ),
            cache_frame_data=False)
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
    else:
        # 디스플레이 없음 → 주기적으로 파일 저장
        node.get_logger().info(
            f"[mpc_debug_monitor] Agg mode: saving to {FILE_OUT}")
        _interval = 1.0 / FILE_SAVE_HZ
        try:
            while rclpy.ok():
                update_fn(None)
                fig.savefig(FILE_OUT, dpi=110, bbox_inches="tight",
                            facecolor=fig.get_facecolor())
                time.sleep(_interval)
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
