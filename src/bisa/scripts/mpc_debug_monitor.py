#!/usr/bin/env python3
"""
mpc_debug_monitor.py  —  MPC 실시간 단일 패널 진단 그래프 (스레드 안전 버전)
---------------------------------------------------------------------------
FuncAnimation 대신 plt.pause() 메인 루프 사용 → TkAgg 크래시 방지

구독:
  /cav01/mpc_performance  bisa/MPCPerformance
  /cav01/accel_cmd        geometry_msgs/Accel  (linear.x=v, angular.z=w)
  /cav01/local_path       nav_msgs/Path

그래프:
  파란선  v_cmd sigmoid (m/s)
  주황점선 |ω_cmd| (rad/s)
  보라bar  solver time (ms, 색상: 초록<30 / 노랑<55 / 빨강≥55)
  배경색   SOLVED=초록  FAILED=빨강  PATH_SHORT=노랑
  수직선   FREEZE / DEVIATION 이벤트 자동 라벨
  상태박스 현재값 + FAILED 비율 실시간 표시
"""

import collections, os, sys, threading, time, traceback

# ── matplotlib 백엔드 ─────────────────────────────────────────────────────
_DISPLAY = os.environ.get("DISPLAY", "")
import matplotlib
if _DISPLAY:
    for _be in ("TkAgg", "Qt5Agg", "Agg"):
        try:
            matplotlib.use(_be)
            import matplotlib.pyplot as _t; _t.figure(); _t.close("all")
            _BACKEND = _be; break
        except Exception:
            continue
    else:
        matplotlib.use("Agg"); _BACKEND = "Agg"
else:
    matplotlib.use("Agg"); _BACKEND = "Agg"

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
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

# ── 상수 ─────────────────────────────────────────────────────────────────
CAV_ID          = "cav01"
WINDOW_S        = 30.0
UPDATE_HZ       = 5.0          # 낮게 유지 → 크래시 방지
MAX_BUF         = 3000
FILE_OUT        = "/tmp/mpc_debug_live.png"
FILE_SAVE_S     = 2.0
FREEZE_S        = 0.8
FREEZE_V        = 0.005
DEV_W           = 0.80
SOLVER_WARN     = 30.0
SOLVER_CRIT     = 55.0

BG = {"SOLVED":"#0a2010","LEGACY":"#0a1020","LOCAL_PATH_TOO_SHORT":"#2a1800",
      "NO_DATA":"#2a0808","FAILED":"#3a0000","MAX_ITER":"#1a1000","UNKNOWN":"#181018"}
FG = {"SOLVED":"#00e060","LEGACY":"#4488ff","LOCAL_PATH_TOO_SHORT":"#ffbb00",
      "NO_DATA":"#ff5555","FAILED":"#ff2222","MAX_ITER":"#ff8800","UNKNOWN":"#cc88ff"}
ANOMALY = {"FAILED","NO_DATA","LOCAL_PATH_TOO_SHORT","MAX_ITER"}


# ── ROS2 노드 ─────────────────────────────────────────────────────────────
class MonitorNode(Node):
    def __init__(self):
        super().__init__("mpc_debug_monitor")
        self._lock = threading.Lock()
        self._t0   = time.monotonic()
        self.t_buf  = collections.deque(maxlen=MAX_BUF)
        self.v_buf  = collections.deque(maxlen=MAX_BUF)
        self.w_buf  = collections.deque(maxlen=MAX_BUF)
        self.st_buf = collections.deque(maxlen=MAX_BUF)
        self.ms_buf = collections.deque(maxlen=MAX_BUF)
        self.ps_buf = collections.deque(maxlen=MAX_BUF)
        self._last_status = "UNKNOWN"
        self._last_ms = 0.0
        self._last_ps = 0
        self._prev_status = "UNKNOWN"
        self.events: list = []
        self._freeze_t0 = None
        self._freeze_done = False

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Accel, f"/{CAV_ID}/accel_cmd", self._accel_cb, qos)
        self.create_subscription(Path,  f"/{CAV_ID}/local_path", self._path_cb,  qos)
        if _HAS_PERF:
            self.create_subscription(MPCPerformance,
                                     f"/{CAV_ID}/mpc_performance", self._perf_cb, qos)
        self.get_logger().info(f"[mpc_debug_monitor] backend={_BACKEND}")
        if _BACKEND == "Agg":
            self.get_logger().warn(f"  No display → saving PNG to {FILE_OUT}")

    def _now(self): return time.monotonic() - self._t0

    def _log_event(self, t, label, color):
        self.events.append((t, label, color))
        if len(self.events) > 30: self.events.pop(0)

    def _accel_cb(self, msg: Accel):
        t = self._now(); v = float(msg.linear.x); w = float(msg.angular.z)
        with self._lock:
            self.t_buf.append(t); self.v_buf.append(v); self.w_buf.append(w)
            self.st_buf.append(self._last_status)
            self.ms_buf.append(self._last_ms); self.ps_buf.append(self._last_ps)
            # 이탈 감지
            if abs(w) > DEV_W and abs(v) > FREEZE_V:
                last_dev = next((e for e in reversed(self.events)
                                 if "DEVIATION" in e[1]), None)
                if last_dev is None or (t - last_dev[0]) > 1.5:
                    self._log_event(t, f"DEV |ω|={abs(w):.2f}", "#ff6b6b")
            # 정지 감지
            if abs(v) < FREEZE_V:
                if self._freeze_t0 is None:
                    self._freeze_t0 = t; self._freeze_done = False
                elif not self._freeze_done and (t - self._freeze_t0) >= FREEZE_S:
                    s = self._last_status
                    cause = ("QP FAILED" if s=="FAILED" else
                             "NO DATA"  if s=="NO_DATA" else
                             f"v=0 [{s}]")
                    self._log_event(self._freeze_t0, f"■FREEZE {cause}", "#facc15")
                    self._freeze_done = True
            else:
                self._freeze_t0 = None; self._freeze_done = False

    def _perf_cb(self, msg):
        with self._lock:
            self._last_status = msg.solver_status
            self._last_ms = msg.total_time_us / 1000.0
            if msg.solver_status != self._prev_status:
                t = self._now()
                if msg.solver_status in ANOMALY:
                    self._log_event(t, msg.solver_status,
                                    FG.get(msg.solver_status, "white"))
                self._prev_status = msg.solver_status

    def _path_cb(self, msg: Path):
        with self._lock: self._last_ps = len(msg.poses)

    def snapshot(self):
        with self._lock:
            return (list(self.t_buf), list(self.v_buf), list(self.w_buf),
                    list(self.st_buf), list(self.ms_buf), list(self.ps_buf),
                    list(self.events),
                    self._last_status, self._last_ms, self._last_ps)


# ── 그래프 빌드 (정적 요소만) ─────────────────────────────────────────────
def build_fig():
    fig, ax = plt.subplots(figsize=(15, 5))
    fig.patch.set_facecolor("#0d1117")
    ax.set_facecolor("#0d1117")
    ax.tick_params(colors="white", labelsize=9)
    for sp in ax.spines.values(): sp.set_edgecolor("#2d3748")
    ax.set_xlabel("Time (s)", color="white", fontsize=10)
    ax.set_ylabel("v (m/s)  /  |ω| (rad/s)", color="white", fontsize=10)
    ax.set_title("MPC Debug Monitor  [cav01]  —  sigmoid v  |  DEVIATION & FREEZE",
                 color="white", fontsize=11, fontweight="bold", pad=8)
    ax.grid(True, color="#2d3748", lw=0.5, alpha=0.6)
    ax.axhline(DEV_W, color="#ff6b6b", lw=0.8, ls=":", alpha=0.55)

    ax2 = ax.twinx()
    ax2.set_ylabel("solver (ms)", color="#555", fontsize=8)
    ax2.tick_params(colors="#555", labelsize=7)
    for sp in ax2.spines.values(): sp.set_edgecolor("#222")

    line_v, = ax.plot([], [], color="#38bdf8", lw=2.0, zorder=4)
    line_w, = ax.plot([], [], color="#fb923c", lw=1.3, ls="--", zorder=4)

    handles = [
        mpatches.Patch(color="#38bdf8", label="v_cmd (m/s)"),
        mpatches.Patch(color="#fb923c", label="|ω_cmd| (rad/s)"),
        mpatches.Patch(color="#ff6b6b", label=f"|ω|>{DEV_W} deviation"),
        mpatches.Patch(color="#00e060", label="SOLVED"),
        mpatches.Patch(color="#ffbb00", label="PATH_TOO_SHORT"),
        mpatches.Patch(color="#ff8800", label="MAX_ITER (partial)"),
        mpatches.Patch(color="#ff2222", label="FAILED/NO_DATA"),
        mpatches.Patch(color="#facc15", label="■ FREEZE"),
        mpatches.Patch(color="#6366f1", label="solver ms"),
    ]
    ax.legend(handles=handles, loc="upper left", fontsize=7.5,
              facecolor="#1a1a2e", labelcolor="white", framealpha=0.85)

    sbox = ax.text(0.995, 0.98, "", transform=ax.transAxes,
                   fontsize=9, fontweight="bold", va="top", ha="right",
                   fontfamily="monospace",
                   bbox=dict(boxstyle="round,pad=0.4", facecolor="#111122",
                             edgecolor="#444", alpha=0.92), zorder=10)
    plt.tight_layout(pad=0.6)
    return fig, ax, ax2, line_v, line_w, sbox


# ── 단일 프레임 업데이트 (메인 스레드 전용) ──────────────────────────────
_bg_spans   = []
_ev_artists = []

def draw_frame(fig, ax, ax2, line_v, line_w, sbox, node: MonitorNode):
    global _bg_spans, _ev_artists

    try:
        (ts, vs, ws, sts, sms, pss, events,
         last_st, last_ms, last_ps) = node.snapshot()
    except Exception:
        return

    if len(ts) < 2:
        return

    ts  = np.asarray(ts,  float)
    vs  = np.asarray(vs,  float)
    ws  = np.abs(np.asarray(ws, float))
    sms = np.asarray(sms, float)

    t_now = ts[-1]; t_lo = t_now - WINDOW_S
    m   = ts >= t_lo
    tw  = ts[m]; vw = vs[m]; ww = ws[m]; smw = sms[m]
    stw = [sts[i] for i in range(len(ts)) if m[i]]

    if len(tw) < 2:
        return

    # ── 배경 제거 후 재그리기 ─────────────────────────────────────────
    for s in _bg_spans:
        try: s.remove()
        except Exception: pass
    _bg_spans.clear()

    seg_s = tw[0]; seg_st = stw[0]
    for i in range(1, len(tw)):
        if stw[i] != seg_st or i == len(tw)-1:
            col = BG.get(seg_st, "#111111")
            try:
                sp = ax.axvspan(seg_s, tw[i], facecolor=col, alpha=0.55, zorder=0)
                _bg_spans.append(sp)
            except Exception:
                pass
            seg_s = tw[i]; seg_st = stw[i]

    # ── 이벤트 수직선 ────────────────────────────────────────────────
    for ln, tx in _ev_artists:
        try: ln.remove()
        except Exception: pass
        try: tx.remove()
        except Exception: pass
    _ev_artists.clear()

    v_hi = max(0.6, float(np.nanmax(vw))*1.2, float(np.nanmax(ww))*1.2)
    for ev_t, ev_label, ev_col in events:
        if ev_t < t_lo: continue
        try:
            ln = ax.axvline(ev_t, color=ev_col, lw=1.8, ls="-", zorder=5, alpha=0.85)
            tx = ax.text(ev_t+0.15, v_hi*0.95, ev_label,
                         color=ev_col, fontsize=7, fontfamily="monospace",
                         rotation=90, va="top", zorder=6,
                         bbox=dict(boxstyle="round,pad=0.12", facecolor="#000",
                                   alpha=0.5, edgecolor="none"))
            _ev_artists.append((ln, tx))
        except Exception:
            pass

    # ── 메인 라인 ────────────────────────────────────────────────────
    line_v.set_data(tw, vw)
    line_w.set_data(tw, ww)

    # ── solver bar (ax2 매번 재그리기) ───────────────────────────────
    ax2.cla()
    ax2.set_facecolor("none")
    ax2.tick_params(colors="#555", labelsize=7)
    ax2.set_ylabel("solver (ms)", color="#555", fontsize=7)
    for sp in ax2.spines.values(): sp.set_edgecolor("#222")
    if len(tw) > 1:
        bw = (tw[-1]-tw[0])/max(1,len(tw))*0.8
        cols = ["#22c55e" if v < SOLVER_WARN else
                "#f59e0b" if v < SOLVER_CRIT else "#ef4444"
                for v in smw]
        ax2.bar(tw, smw, width=bw, color=cols, alpha=0.45, zorder=1)
    ms_max = max(80.0, float(np.nanmax(smw))*1.4) if len(smw) else 80.0
    ax2.set_ylim(0, ms_max)
    ax2.axhline(SOLVER_WARN, color="#f59e0b", lw=0.7, ls="--", alpha=0.6)
    ax2.axhline(SOLVER_CRIT, color="#ef4444", lw=0.7, ls="--", alpha=0.6)

    # ── 축 범위 ───────────────────────────────────────────────────────
    ax.set_xlim(t_lo, t_now+0.3)
    ax.set_ylim(-0.03, max(0.6, v_hi))

    # ── 상태 박스 ─────────────────────────────────────────────────────
    col  = FG.get(last_st, "white")
    v_now = float(vs[-1]) if len(vs) else 0.0
    w_now = float(ws[-1]) if len(ws) else 0.0
    ps_ok = "OK" if last_ps >= 5 else f"!! {last_ps}"
    fail_n = sum(1 for s in stw if s in ANOMALY)
    fail_r = fail_n / max(1, len(stw)) * 100
    warn   = " ⚠" if last_ms > SOLVER_WARN else ""
    sbox.set_text(
        f" STATUS  : {last_st}\n"
        f" v_cmd   : {v_now:+.3f} m/s\n"
        f" |ω_cmd| : {w_now:.3f} rad/s\n"
        f" solver  : {last_ms:.1f} ms{warn}\n"
        f" path    : {last_ps}  [{ps_ok}]\n"
        f" ANOMALY : {fail_n}/{len(stw)} ({fail_r:.0f}%)")
    sbox.set_color(col)
    try: sbox.get_bbox_patch().set_edgecolor(col)
    except Exception: pass

    try:
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
    except Exception:
        pass


# ── 진입점 ────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()

    spin_th = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_th.start()

    fig, ax, ax2, line_v, line_w, sbox = build_fig()
    interval = 1.0 / UPDATE_HZ

    if _BACKEND != "Agg":
        plt.ion()
        plt.show(block=False)
        try:
            while rclpy.ok():
                draw_frame(fig, ax, ax2, line_v, line_w, sbox, node)
                plt.pause(interval)   # 메인 스레드에서만 Tk 이벤트 처리
        except KeyboardInterrupt:
            pass
        except Exception as e:
            node.get_logger().error(f"[mpc_debug_monitor] crash: {e}\n{traceback.format_exc()}")
    else:
        _last_save = 0.0
        try:
            while rclpy.ok():
                draw_frame(fig, ax, ax2, line_v, line_w, sbox, node)
                now = time.monotonic()
                if now - _last_save >= FILE_SAVE_S:
                    try: fig.savefig(FILE_OUT, dpi=110, bbox_inches="tight",
                                     facecolor=fig.get_facecolor())
                    except Exception: pass
                    _last_save = now
                time.sleep(interval)
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
