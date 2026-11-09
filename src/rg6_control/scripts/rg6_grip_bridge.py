#!/usr/bin/env python3
"""rg6_grip_bridge: commands the RG6 over XML-RPC to the OnRobot URCap.

Why not over a tool digital output:  the URCap is an RTDE client itself and occupies ``tool_digital_output_mask``.  For
``ur_robot_driver`` to start at all it runs on an input recipe WITHOUT the ``tool_digital_output*`` lines -- so ROS
cannot set a tool DO, and a driver that steers the gripper that way cannot work here.

Why not by URScript over port 30002:  ``rg_grip`` is only defined by the installation preamble that PolyScope puts in
front of every generated program.  A script sent over 30002 runs without that preamble, so the symbol is discarded
(measured: no program change, AI2 unchanged, ``textmsg("literal")`` passes through as a control).

Why onboard and not in the offboard container:  the endpoint hangs off the arm subnet 192.168.131.0/24, and from the
workstation there is no route to it.  And the robot must be able to grip even when the radio link is gone -- the same
argument with which R16 puts the reflex layer onboard.

The endpoint offers more than ``rg_grip``: a complete status path back (``rg_get_width``, ``rg_get_busy``,
``rg_get_grip_detected``, ``rg_get_status``, ``rg_get_safety_failed``).  That makes the voltage approximation over AI2
unnecessary -- and AI2 has turned out to be mis-calibrated by ~17 mm, measured against exactly these getters.

What this node does NOT do:  it does not speak the ``/motion/*`` JSON
protocol.  That is ``plan_server`` in the offboard container, and it does so
identically on ``mock`` and ``real`` -- one code path instead of two.  Here
there are exclusively standard ROS interfaces::

    control_msgs/GripperCommand  (action)   ◀─ MoveIt and plan_server
    sensor_msgs/JointState       (topic)    ─▶ rg6_finger_joint
    std_msgs/String              (topic)    ─▶ rg6/bridge_state, own JSON

So the robot does NOT need ``robot_contract``.  That package is private (not even clonable from the robot), and a
dependency that prevents the deployment is no safeguard.  What is kept are names (ROS parameters) and the linkage
kinematics (a generated table, see FingerKinematics).

Selftest without ROS (runs on the workstation too)::

    python3 rg6_grip_bridge.py --selftest
"""

from __future__ import annotations

import json
import pathlib
import sys
import threading
import time
import xmlrpc.client
from dataclasses import dataclass

#: Default endpoint.  The path is "/", NOT "/RPC2" -- the xmlrpc-c abyss in
#: the CB3 tool daemon serves the root only.
DEFAULT_URL = "http://192.168.131.40:41414/"
#: RG6 nominal ranges (user manual v6.6.2).  Clamping happens HERE so that an
#: oversized request does not come back as a fault but as what the device can
#: actually do.
WIDTH_RANGE_MM = (0.0, 160.0)
FORCE_RANGE_N = (0.0, 120.0)


class Rg6Error(Exception):
    """The gripper rejected a command or did not answer."""


@dataclass(frozen=True)
class Rg6State:
    """Snapshot of the gripper, straight from the device."""

    width_m: float
    busy: bool
    grip_detected: bool
    status: int
    safety_failed: bool

    @property
    def readable(self) -> bool:
        """Did the URCap actually MEASURE -- or did it merely answer?

        The endpoint sits in the control box and is reachable even when
        nothing is powered at the tool connector.  It then throws NO fault but
        answers with its own marker for "no measurement"::

            rg_get_width ─▶ -999.0    rg_get_status ─▶ -1
            rg_get_busy  ─▶ True      rg_get_safety_failed ─▶ True

        Queried directly at the endpoint on the a200-0553 on 2026-08-24 while the arm stood at POWER_OFF.  Without this
        check, -999 mm passes through ``angle_from_width`` (which clamps the WIDTH instead of extrapolating) and comes
        out as 1.25478 rad -- a FULLY CLOSED gripper, published as a measurement.  That was visible live:
        rg6_finger_joint = 1.25478 on platform/joint_states, hence in RSP, TF and the planning scene of move_group, with
        the gripper unpowered.

        This check is the only guard: a dead gripper does NOT raise an exception, it answers with the sentinels above.
        """
        lo_mm, hi_mm = WIDTH_RANGE_MM
        return self.status >= 0 and lo_mm <= self.width_m * 1000.0 <= hi_mm


def _clamp(value: float, lo: float, hi: float) -> float:
    return min(max(float(value), lo), hi)


class _TimeoutTransport(xmlrpc.client.Transport):
    """Apply the request timeout to XML-RPC's cached HTTP connection."""

    def __init__(self, timeout_s: float) -> None:
        super().__init__()
        self._timeout_s = float(timeout_s)

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self._timeout_s
        return connection


class Rg6Client:
    """XML-RPC interface to the OnRobot URCap.

    The ONLY place where units change: the profile and the ``/motion/*`` wire work in meters, the endpoint in
    millimeters.
    """

    def __init__(self, url: str = DEFAULT_URL, tool_index: int = 0, timeout_s: float = 3.0) -> None:
        self._url = url
        self._tool = int(tool_index)
        # Bound socket I/O so an unresponsive endpoint cannot hold the state poller indefinitely.
        transport = _TimeoutTransport(timeout_s)
        self._proxy = xmlrpc.client.ServerProxy(url, transport=transport, allow_none=True)
        # ServerProxy is NOT thread safe: proxy and transport share ONE HTTP connection.  Two threads reach for it here
        # -- the grip worker and the state poller of the finger joint -- and without this lock their requests interleave
        # on the socket.
        self._lock = threading.Lock()

    @property
    def url(self) -> str:
        return self._url

    def grip(self, width_m: float, force_n: float) -> None:
        """Drive to ``width_m``.  ``Rg6Error`` if the device says no."""
        width_mm = _clamp(width_m * 1000.0, *WIDTH_RANGE_MM)
        force = _clamp(force_n, *FORCE_RANGE_N)
        # "+ 0.0" is NOT cosmetic: an int gives fault -501.  A commanded 0 mm or 60 N would otherwise go onto the wire
        # as an int.
        rc = self._call("rg_grip", self._tool, width_mm + 0.0, force + 0.0)
        if int(rc) != 0:
            raise Rg6Error(f"rg_grip({width_mm:.1f} mm, {force:.1f} N) answered {rc!r} instead of 0")

    def stop(self) -> None:
        self._call("rg_stop", self._tool)

    def state(self) -> Rg6State:
        return Rg6State(
            width_m=float(self._call("rg_get_width", self._tool)) / 1000.0,
            busy=bool(self._call("rg_get_busy", self._tool)),
            grip_detected=bool(self._call("rg_get_grip_detected", self._tool)),
            status=int(self._call("rg_get_status", self._tool)),
            safety_failed=bool(self._call("rg_get_safety_failed", self._tool)),
        )

    def _call(self, method: str, *args):
        try:
            with self._lock:
                return getattr(self._proxy, method)(*args)
        except xmlrpc.client.Fault as exc:
            raise Rg6Error(f"{method}: Fault {exc.faultCode} {exc.faultString}") from exc
        except OSError as exc:
            raise Rg6Error(f"{method}: {self._url} not reachable ({exc})") from exc


def await_settled(
    client, start_timeout_s: float = 1.0, motion_timeout_s: float = 10.0, poll_s: float = 0.05
) -> Rg6State:
    """Wait until the hand stands still, and THEN read the state.

    ``rg_grip`` acknowledges the **acceptance**, not the result.  Reading immediately afterwards yields the width from
    before -- measured over the wire: commanded 60 mm, driven to 64.96 mm, reported 2.8 mm (the starting value).  With
    ``width_m`` wrong, ``grasped`` is worthless too, and that is the field the whole return path exists for.

    Both edges are waited for, and the reason for the first one is measured: after the command ``busy`` stays false for
    about 0.4 s before the gripper starts moving.  A plain "wait while busy" returns immediately inside that gap.

    Both windows expire instead of hanging: if the gripper never starts moving (it already stands at the target), the
    function answers after ``start_timeout_s`` with whatever is there.
    """
    deadline = time.monotonic() + start_timeout_s
    state = client.state()
    while not state.busy and time.monotonic() < deadline:
        time.sleep(poll_s)
        state = client.state()
    deadline = time.monotonic() + motion_timeout_s
    while state.busy and time.monotonic() < deadline:
        time.sleep(poll_s)
        state = client.state()
    return state


class FingerKinematics:
    """Joint angle ◀─▶ grip width, from a generated table.

    Why a table and not an import:  this node runs on the ROBOT and must need nothing there that does not belong to the
    robot.  ``robot_contract`` is private, so importing it would make the bridge undeployable.

    Why a table and not a formula:  the fingers of the rg6_v2 are a four-bar linkage with no closed form.  An
    approximation placed next to it would be the second version model and driver have already drifted apart on (R19).

    The file is generated by ``tools/derive_finger_kinematics.py`` from the GENERATED URDF; it is data, not code, and
    carries its provenance in its head.  27 support points keep the interpolation error at 0.047 mm -- below the finger
    position resolution of the RG6 (0.1 mm per datasheet).
    """

    def __init__(self, path: str) -> None:
        with open(path, encoding="utf-8") as fh:
            raw = json.load(fh)
        tab = raw["table_q_rad_width_m"]
        self._q = [float(z[0]) for z in tab]
        self._w = [float(z[1]) for z in tab]
        if sorted(self._q) != self._q:
            raise ValueError(f"{path}: support points not ascending in q")
        # The width MUST fall: the inversion rests on that.  If it rises anywhere, support points beyond the zero
        # crossing were caught, where the fingers pass through each other in the model.
        if any(b >= a for a, b in zip(self._w, self._w[1:])):
            raise ValueError(f"{path}: width does not fall monotonically")
        self.joint = str(raw.get("joint", "rg6_finger_joint"))
        self.q_min, self.q_max = self._q[0], self._q[-1]
        self.max_width_m, self.min_width_m = self._w[0], self._w[-1]
        self.source = path

    def width_from_angle(self, q: float) -> float:
        """Clamped to the table edge: beyond the stop, the stop applies."""
        q = min(max(float(q), self.q_min), self.q_max)
        for i in range(1, len(self._q)):
            if q <= self._q[i]:
                t = (q - self._q[i - 1]) / (self._q[i] - self._q[i - 1])
                return self._w[i - 1] + t * (self._w[i] - self._w[i - 1])
        return self._w[-1]

    def angle_from_width(self, width_m: float) -> float:
        """Inversion, also clamped.  The table falls, so search backwards."""
        w = min(max(float(width_m), self.min_width_m), self.max_width_m)
        for i in range(1, len(self._w)):
            if w >= self._w[i]:
                t = (self._w[i - 1] - w) / (self._w[i - 1] - self._w[i])
                return self._q[i - 1] + t * (self._q[i] - self._q[i - 1])
        return self._q[-1]


#: What went to the gripper last.  These are the names the manipulator
#: diagnostics displays verbatim, so they are part of its plain-text output.
COMMAND_NONE = "NONE"
COMMAND_GRIP = "GRIP"
COMMAND_STOP = "STOP"


def goal_to_grip(position_rad: float, max_effort_n: float, linkage, default_force_n: float, force_range_n) -> tuple:
    """``control_msgs/GripperCommand`` goal ─▶ ``(width in m, force in N)``.

    MoveIt commands the gripper as a JOINT VALUE, not as a width -- the conversion uses the same linkage geometry the
    URDF carries.

    ``max_effort <= 0`` means "take whatever fits" in the GripperCommand contract, not "zero force": MoveIt frequently
    leaves the field empty.  The profile default then applies.  Clamping to the device range makes an oversized request
    arrive as what the RG6 can do.
    """
    lo, hi = float(force_range_n[0]), float(force_range_n[1])
    force = (
        float(default_force_n) if max_effort_n is None or max_effort_n <= 0.0 else min(max(float(max_effort_n), lo), hi)
    )
    return float(linkage.width_from_angle(float(position_rad))), force


def goal_result(state, target_width_m: float, force_n: float, linkage, tolerance_m: float) -> dict:
    """Result fields for GripperCommand, from the MEASURED state.

    ``stalled`` is ``grip_detected``:  with it the RG6 reports that it reached the force limit BEFORE the target width
    -- which is exactly "standing, but not at the goal", what the field means.

    ``effort`` is the COMMANDED force, not a measured one: the endpoint offers no force reading.  An invented number
    would be worse than an honest repeat of the setpoint.

    ``tolerance_m`` is deliberately coarse:  the value the device returns lies
    +3 to +5 mm above the true width (R19, anchored with the caliper).  As
    long as that deviation is not compensated, ``reached_goal`` cannot be
    sharper than this error.
    """
    return {
        "position": float(linkage.angle_from_width(state.width_m)),
        "effort": float(force_n),
        "stalled": bool(state.grip_detected),
        "reached_goal": abs(state.width_m - float(target_width_m)) <= float(tolerance_m),
    }


def status_payload(state, last_command: str = COMMAND_NONE) -> dict:
    """Device state for ``<ns>/rg6/bridge_state`` -- flat, as JSON.

    Why an own topic and not rg6_msgs/GripperState:  rg6_msgs does not sit in the boot path of the robot, and a status
    message that needs a package from there would be exactly the dependency this node avoids.  JSON inside a
    std_msgs/String costs no build and no overlay.

    NOT included are AI2/AI3:  the raw voltages sit on ``io_and_status_controller/tool_data``, and whoever needs them
    reads them there.  Mirroring them here would create a second source for the same number -- and AI2 has been measured
    as mis-calibrated by up to 17 mm (R19), so it is precisely not a good second opinion.
    """
    return {
        "width_m": state.width_m,
        "busy": state.busy,
        "grip_detected": state.grip_detected,
        "status": state.status,
        "safety_failed": state.safety_failed,
        "last_command": last_command,
    }


def finger_angle_to_publish(
    state: Rg6State | None, linkage: FingerKinematics, last_measured_rad: float | None
) -> tuple[float, bool]:
    """The angle for ``rg6_finger_joint`` -- also when the gripper measures nothing (R43).

    Returns ``(angle_rad, measured)``.  ``measured`` is false for every substituted value; the caller keeps it out of
    its history and the health signal stays where it belongs, on ``<ns>/rg6/bridge_state`` (``status: -1``) and in the
    edge-triggered log line.

    WHY THIS EXISTS AT ALL -- the topic used to stay silent whenever the gripper did not measure, and that silence
    reaches much further than it looks.  Eight of the ten RG6 links with collision geometry are placed by
    ``robot_state_publisher`` out of ``rg6_finger_joint`` (directly and through the five ``<mimic>`` joints).  Without
    the joint they have NO TF, and ``move_group``'s ``shape_mask`` then cannot mask them out of the depth cloud: the
    whole hand falls out of the octomap self filter and its own fingers become obstacle voxels in front of a camera
    that sits on the gripper.  Measured on the a200-0553 on 2026-09-01 with the arm unpowered: ten ``Missing transform
    for shape mesh`` lines per cloud (handles 14...23, the ten collision meshes of the eight links) and, because the
    updater waits ``shape_transform_cache_lookup_wait_time`` = 0.3 s PER link, an occupancy update every 2.44 s instead
    of the 5 Hz the feed delivers.

    WHY THE LAST MEASURED VALUE FIRST:  the state this guards against is an unpowered tool connector, and an unpowered
    gripper does not move.  The last measurement is therefore the physically correct angle, not a guess -- unlike the
    open stop, which merely is the pose with the largest envelope and thus the conservative one for a clearance check.

    WHY THE OPEN STOP AS THE FALLBACK, and why it is not written as a literal:  ``q_min`` is where the linkage table
    starts, and the table is generated from the URDF (R19).  A hard 0.0 here would be a second place that has to be
    right about the same geometry -- the exact construction model and driver already drifted apart on once.

    What this does NOT do is claim a measurement.  ``status_payload`` is untouched, so the manipulator diagnostics
    still reads ``status: -1``/``safety_failed: true`` and reports the outage; ``state=None`` (the endpoint did not
    answer at all) additionally leaves ``bridge_state`` silent, which is that path's own signal.
    """
    if state is not None and state.readable:
        return float(linkage.angle_from_width(state.width_m)), True
    return (linkage.q_min if last_measured_rad is None else float(last_measured_rad)), False


# ---------------------------------------------------------------------------
# Selftest -- without ROS, so that it runs on the workstation too.
# ---------------------------------------------------------------------------
def _spawn_fake_urcap():
    """Local XML-RPC stand-in; returns ``(server, thread, url, log)``.

    It reproduces the two quirks of the real endpoint that hide a bug: int arguments are a fault -501, and the width
    comes back in millimeters.
    """
    from xmlrpc.server import SimpleXMLRPCServer

    log = []
    state = {"width_mm": 103.26, "busy": False, "grip": False, "target_mm": 103.26, "phases": []}

    def rg_grip(tool, width, force):
        if not isinstance(width, float) or not isinstance(force, float):
            raise xmlrpc.client.Fault(-501, "expected double")
        log.append(("grip", tool, width, force))
        # Modeled on the measurement from 2026-08-19 (65 ─▶ 20 mm): after the command ``busy`` stays false for about
        # 0.4 s, THEN the hand moves for about 1.2 s, and only at the end does the new width stand. ``rg_grip`` itself
        # returns immediately -- it acknowledges the acceptance, not the result.
        state["target_mm"] = width
        state["phases"] = ["idle", "idle", "moving", "moving", "moving"]
        return 0

    def rg_get_busy(tool):
        if not state["phases"]:
            return state["busy"]
        phase = state["phases"].pop(0)
        if not state["phases"]:
            state["width_mm"] = state["target_mm"]  # travel finished
        return phase == "moving"

    def rg_stop(tool):
        log.append(("stop", tool))
        return 0

    srv = SimpleXMLRPCServer(("127.0.0.1", 0), logRequests=False, allow_none=True)
    srv.register_function(rg_grip, "rg_grip")
    srv.register_function(rg_stop, "rg_stop")
    srv.register_function(lambda t: state["width_mm"], "rg_get_width")
    srv.register_function(rg_get_busy, "rg_get_busy")
    srv.register_function(lambda t: state["grip"], "rg_get_grip_detected")
    srv.register_function(lambda t: 0, "rg_get_status")
    srv.register_function(lambda t: False, "rg_get_safety_failed")
    srv.register_function(time.sleep, "delay")
    thread = threading.Thread(target=srv.serve_forever, daemon=True)
    thread.start()
    host, port = srv.server_address
    return srv, thread, f"http://{host}:{port}/", log


def selftest() -> int:
    srv, _thread, url, log = _spawn_fake_urcap()
    try:
        cli = Rg6Client(url)

        # 1. The width goes out in millimeters and comes back in meters.
        #    It is read AFTER the travel -- why, see 5a.
        cli.grip(0.100, 60.0)
        assert log[-1][2] == 100.0, log[-1]
        assert abs(await_settled(cli, poll_s=0.0).width_m - 0.100) < 1e-9

        # 2. Both numbers are floats -- the stand-in faults otherwise.
        cli.grip(0.0, 60.0)
        assert isinstance(log[-1][2], float) and isinstance(log[-1][3], float)

        # 3. Clamped to the device range, not passed through.
        cli.grip(0.500, 999.0)
        assert log[-1][2] == 160.0 and log[-1][3] == 120.0, log[-1]

        # 4. A dead endpoint is an Rg6Error, not a hang.
        dead = Rg6Client("http://127.0.0.1:9/", timeout_s=0.5)
        try:
            dead.state()
        except Rg6Error:
            pass
        else:  # pragma: no cover
            raise AssertionError("dead endpoint should have raised Rg6Error")

        slow = Rg6Client(url, timeout_s=0.01)
        try:
            slow._call("delay", 0.1)
        except Rg6Error as exc:
            assert isinstance(exc.__cause__, TimeoutError), exc
        else:
            raise AssertionError("an unresponsive endpoint should time out")

        # 5a. A result read IMMEDIATELY after rg_grip reports the width from
        #     BEFORE.  Measured over the wire on 2026-08-19: commanded 60 mm,
        #     driven to 64.96 mm, reported 2.8 mm -- the starting value.
        #     ``rg_grip`` acknowledges the acceptance, not the result, and with
        #     ``width_m`` wrong ``grasped`` would be worthless too.
        cli.grip(0.020, 40.0)
        assert abs(cli.state().width_m - 0.020) > 0.001, "read too early -> stale value"

        # ... and with the wait it is right.  BOTH edges are waited for: the
        #     gripper stands still for about 0.4 s after the command
        #     (measured), a plain "while busy" returns immediately.
        cli.grip(0.045, 40.0)
        settled = await_settled(cli, poll_s=0.0)
        assert abs(settled.width_m - 0.045) < 1e-9, settled

        # If the gripper never starts moving (it already stands at the target), the wait answers after the start window
        # -- not never.
        stalled = await_settled(cli, start_timeout_s=0.05, poll_s=0.0)
        assert abs(stalled.width_m - 0.045) < 1e-9, stalled

        # 5c. The MoveIt path: GripperCommand commands a JOINT VALUE.
        #     MoveIt never needs the gripper on the controller_manager -- it
        #     needs this action, and that runs in an ordinary executor, not in
        #     the 8 ms cycle of the CB3.
        kin = FingerKinematics(str(pathlib.Path(__file__).with_name("rg6_finger_kinematics.json")))
        width, force = goal_to_grip(kin.angle_from_width(0.100), 55.0, kin, 40.0, (25.0, 120.0))
        assert abs(width - 0.100) < 2e-4, width  # table resolution
        assert force == 55.0, force
        # An empty max_effort means "take what fits" -- not "zero force".
        assert goal_to_grip(0.0, 0.0, kin, 40.0, (25.0, 120.0))[1] == 40.0
        # ... and an oversized request is clamped, not passed through.
        assert goal_to_grip(0.0, 999.0, kin, 40.0, (25.0, 120.0))[1] == 120.0

        st_closed = Rg6State(width_m=0.0605, busy=False, grip_detected=True, status=0, safety_failed=False)
        res = goal_result(st_closed, 0.060, 40.0, kin, tolerance_m=0.008)
        assert res["reached_goal"] is True and res["stalled"] is True, res
        assert res["effort"] == 40.0, res
        # Far off is far off, even when the gripper stands still.
        assert goal_result(st_closed, 0.100, 40.0, kin, tolerance_m=0.008)["reached_goal"] is False

        # 5b. The status message carries the DEVICE STATE, not the command --
        #     that is exactly what the manipulator diagnostics judges.
        st = Rg6State(width_m=0.1032, busy=False, grip_detected=True, status=0, safety_failed=False)
        status = status_payload(st, COMMAND_GRIP)
        assert status["width_m"] == st.width_m, status
        assert status["grip_detected"] is True, status
        assert status["last_command"] == COMMAND_GRIP, status
        # It must survive json.dumps -- it goes onto the wire as a string.
        assert json.loads(json.dumps(status)) == status, status
        # AI2/AI3 do NOT belong in it (see the docstring): a second source for the same number, and the worse one.
        assert "width_raw" not in status and "force_raw" not in status, status

        # 5c. An ANSWER is not yet a MEASUREMENT (Rg6State.readable).  The
        #     values are the ones read on the a200-0553 on 2026-08-24, with the
        #     arm at POWER_OFF.
        assert st.readable, "a real measurement must pass"
        dead = Rg6State(width_m=-0.999, busy=True, grip_detected=True, status=-1, safety_failed=True)
        assert not dead.readable, "-999 mm / status -1 is not a measurement"
        # Without the guard this would become a FULLY CLOSED gripper -- the clamp does not extrapolate, it snaps to the
        # stop.
        assert abs(kin.angle_from_width(dead.width_m) - kin.q_max) < 1e-9
        # Both halves of the check bite on their own: an error status alone is enough, and so is a width beyond the
        # nominal range.
        assert not Rg6State(width_m=0.060, busy=False, grip_detected=False, status=-1, safety_failed=False).readable
        assert not Rg6State(width_m=0.400, busy=False, grip_detected=False, status=0, safety_failed=False).readable
        # The nominal limits themselves are still valid -- the device reports up to 5 mm above the jaw measurement
        # (R19), which must not count as dead.
        assert Rg6State(width_m=0.160, busy=False, grip_detected=False, status=0, safety_failed=False).readable
        assert Rg6State(width_m=0.0, busy=False, grip_detected=False, status=0, safety_failed=False).readable

        # 6. Width ─▶ finger joint comes from the GENERATED table, not from a
        #    formula and not from robot_contract (R19; the robot must not need
        #    the private contract).
        # Monotonic: wider open ─▶ SMALLER joint value (0 = fully open).
        assert kin.angle_from_width(0.100) < kin.angle_from_width(0.045)
        # A round trip meets itself, within the table resolution. min_width_m instead of 0.0: the model only closes to
        # 0.4 mm, where the pads touch.  A round trip over 0.0 would test the clamp, not the interpolation.
        for w in (kin.min_width_m, 0.020, 0.060, 0.100, kin.max_width_m):
            assert abs(kin.width_from_angle(kin.angle_from_width(w)) - w) < 2e-4, w
        # The WIDTH is clamped, not extrapolated: beyond the stop the stop applies, otherwise a negative width would
        # silently sit behind the closed position.
        assert kin.angle_from_width(-0.05) == kin.angle_from_width(kin.min_width_m)
        assert kin.angle_from_width(0.300) == kin.angle_from_width(kin.max_width_m)
        # The table ends BEFORE the point where the fingers pass through each other in the model -- otherwise the
        # inversion would be ambiguous.
        assert kin.q_max < 1.30, kin.q_max
        assert kin.max_width_m > 0.15 and kin.min_width_m < 0.002

        # 7. Two threads on ONE ServerProxy -- in the node those are the grip
        #    worker and the finger joint poller.  Without the lock in _call
        #    their requests interleave on the shared socket; that shows up as
        #    ResponseNotReady/BadStatusLine.
        errors = []

        def _hammer():
            try:
                for _ in range(25):
                    cli.state()
            except Exception as exc:
                errors.append(exc)

        threads = [threading.Thread(target=_hammer) for _ in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        assert not errors, errors

        # 8. The finger angle that goes onto joint_states -- INCLUDING the case where nothing measures (R43).  A
        #    missing rg6_finger_joint costs the eight movable RG6 links their TF, and move_group then drops the whole
        #    hand out of the octomap self filter; measured at the robot on 2026-09-01 as ten shape_mask lines per
        #    cloud (handles 14...23) and an occupancy update rate of 0.4 Hz instead of 5.
        measured = Rg6State(width_m=0.100, busy=False, grip_detected=False, status=0, safety_failed=False)
        q, is_measured = finger_angle_to_publish(measured, kin, None)
        assert is_measured and abs(q - kin.angle_from_width(0.100)) < 1e-12, q

        # The sentinel of an unpowered tool connector: with a measurement behind it, that one is held -- the gripper
        # cannot move without power, so the last measured value is the physically correct one.
        sentinel = Rg6State(width_m=-0.999, busy=True, grip_detected=True, status=-1, safety_failed=True)
        q, is_measured = finger_angle_to_publish(sentinel, kin, 0.7)
        assert not is_measured and q == 0.7, q

        # Without one -- the bridge came up before the arm -- the OPEN stop applies, and it comes from the table
        # rather than from a literal: q_min is where the table starts, and that is the widest the DEVICE reaches
        # (0.038 rad at 151.1 mm), not the 0.0 the SRDF's group_state 'open' carries -- the model opens wider than
        # the hardware does.  That difference is the known 'Deviation in joint rg6_finger_joint: [0] != [0.038]'.
        q, is_measured = finger_angle_to_publish(sentinel, kin, None)
        assert not is_measured and q == kin.q_min, q
        assert abs(kin.width_from_angle(kin.q_min) - kin.max_width_m) < 1e-12, kin.q_min

        # An unreachable endpoint (Rg6Error -> no state at all) breaks the TF the same way, so it takes the same path.
        q, is_measured = finger_angle_to_publish(None, kin, None)
        assert not is_measured and q == kin.q_min, q
    finally:
        srv.shutdown()

    print(
        "rg6_grip_bridge selftest: OK (units, float coercion, clamping, "
        "timeout, status message, linkage table, concurrency, finger angle fallback)"
    )
    return 0


# ---------------------------------------------------------------------------
# ROS node -- rclpy is imported ONLY HERE so that --selftest runs on the
# workstation too, where there is no rclpy.
# ---------------------------------------------------------------------------
def run(argv) -> int:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init(args=argv)
    node = Node("rg6_grip_bridge")
    log = node.get_logger()

    node.declare_parameter("endpoint_url", DEFAULT_URL)
    node.declare_parameter("tool_index", 0)
    node.declare_parameter("timeout_s", 3.0)
    # Names and limits as PARAMETERS, not from a profile: they are the only thing this node needs to know about its
    # environment, and putting a private Python package on the robot for them would make the bridge undeployable.
    node.declare_parameter("manipulators_ns", "/a200_0553/manipulators")
    node.declare_parameter("driver_joint", "rg6_finger_joint")
    node.declare_parameter("action_name", "")  # empty = derived from manipulators_ns
    node.declare_parameter("default_force_n", 40.0)
    node.declare_parameter("force_range_n", [25.0, 120.0])
    node.declare_parameter("kinematics_file", "")  # empty = next to the script
    # Waiting for the end of the travel (see await_settled).  As parameters, because the numbers come from a measurement
    # on ONE gripper: 0.4 s start-up, 1.2 s travel over 45 mm.  The 1.0 s start window is at the same time the wait for
    # a command that has nothing to do.
    node.declare_parameter("settle_start_timeout_s", 1.0)
    node.declare_parameter("settle_motion_timeout_s", 10.0)
    node.declare_parameter("settle_poll_s", 0.05)

    def _p(name):
        return node.get_parameter(name).value

    client = Rg6Client(_p("endpoint_url"), int(_p("tool_index")), float(_p("timeout_s")))

    kin_file = _p("kinematics_file") or str(pathlib.Path(__file__).with_name("rg6_finger_kinematics.json"))
    linkage = FingerKinematics(kin_file)
    log.info(f"linkage table: {kin_file} ({linkage.max_width_m * 1000:.1f} mm open, q up to {linkage.q_max:.5f} rad)")

    # -- finger joint ------------------------------------------------------
    # Nothing else publishes rg6_finger_joint into /joint_states (measured
    # 2026-08-17).  Without it move_group sees the gripper in its DEFAULT
    # pose, and every clearance check around the hand computes against a pose
    # it does not have -- the same class as R15, only movable.  This node has
    # the measured width anyway.
    #
    # And it publishes in EVERY pass, measurement or not: a joint that drops
    # out is worse than the default pose, because then robot_state_publisher
    # emits no TF at all for the eight movable links and move_group's shape
    # mask stops masking the hand out of the depth cloud (R43).
    from sensor_msgs.msg import JointState

    node.declare_parameter("joint_state_rate_hz", 5.0)
    manip_ns = str(_p("manipulators_ns")).rstrip("/")
    finger_joint = str(_p("driver_joint"))
    joints = node.create_publisher(JointState, f"{manip_ns}/endeffectors/joint_states", 10)
    # The same poll carries the state for the manipulator diagnostics.  It reads it here -- <ns>/rg6/state has no
    # publisher.
    states = node.create_publisher(String, f"{manip_ns}/rg6/bridge_state", 10)
    last_command = [COMMAND_NONE]

    def _poll_joint() -> None:
        """The finger value from the MEASURED width, not from the command -- and never nothing at all.

        The angle itself comes from ``finger_angle_to_publish``:  measured while the gripper measures, the last
        measured value while it does not, the open stop of the table until there has been a first measurement.  The
        topic must not fall silent, see there (R43).

        An own thread and NO ROS timer:  ``client.state()`` is a blocking XML-RPC call.  In a timer callback it would
        hang on the executor -- 3 s every 200 ms with a dead endpoint, and the grip command would not get through in
        that same time.

        The conversion width ─▶ joint is done by the linkage geometry of the profile (R19), not by this node.
        """
        period = 1.0 / float(_p("joint_state_rate_hz"))
        # Edge-triggered, NOT per iteration: the loop runs at joint_state_rate_hz, and an outage that logged every
        # pass would bury the moment it began under thousands of identical lines.  One line when it starts, one
        # when it comes back -- and the duration on the second, which is the number an operator actually wants.
        outage_since: float | None = None
        # The same edge trigger for the SUBSTITUTED joint value (R43).  It is a different event from the outage above
        # -- an answering endpoint with an unpowered tool connector raises nothing -- and it is the one an operator
        # needs when TF shows a hand that nobody is measuring.
        substitute_since: float | None = None
        last_measured_rad: float | None = None
        while rclpy.ok():
            state = None
            try:
                state = client.state()
            except Rg6Error as exc:
                # The STATUS TOPIC stays silent, deliberately: the silence is itself the signal, and the diagnostics
                # judges the age of the last status and reports the outage from it.  The LOG is a different
                # channel and lies to nobody -- without it the moment an outage began is unrecoverable.
                if outage_since is None:
                    outage_since = time.monotonic()
                    log.warning(f"gripper status unavailable: {exc}")
            else:
                if outage_since is not None:
                    log.info(f"gripper status back after {time.monotonic() - outage_since:.1f} s")
                    outage_since = None
                # The status message goes out even when the endpoint ANSWERS without having measured
                # (state.readable, see there): the raw answer (status -1, safety_failed) is exactly what the
                # manipulator diagnostics needs to report the outage.  Were it silent too, the diagnostics would
                # only see ageing and could not tell "bridge dead" from "gripper unpowered".
                states.publish(String(data=json.dumps(status_payload(state, last_command[0]))))

            # The JOINT, unlike the status, goes out in every pass -- with the measured angle where there is one and
            # otherwise with the held value or the open stop.  What must never happen is what -999 mm did before the
            # readable check existed: run through the clamp and arrive as a fully closed gripper, a number move_group
            # takes at face value (R19).  What must equally not happen is the joint DROPPING OUT, which costs the
            # eight movable RG6 links their TF and the whole hand its place in the octomap self filter (R43).
            q_rad, measured = finger_angle_to_publish(state, linkage, last_measured_rad)
            if measured:
                last_measured_rad = q_rad
                if substitute_since is not None:
                    log.info(f"rg6_finger_joint measured again after {time.monotonic() - substitute_since:.1f} s")
                    substitute_since = None
            elif substitute_since is None:
                substitute_since = time.monotonic()
                held = "no measurement yet, open stop" if last_measured_rad is None else "last measured value held"
                log.warning(f"rg6_finger_joint substituted ({held}): {q_rad:.5f} rad")
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = [finger_joint]
            msg.position = [q_rad]
            joints.publish(msg)
            time.sleep(period)

    threading.Thread(target=_poll_joint, daemon=True).start()

    # ONE command at a time.  A second one during the travel is REJECTED, not queued: on 2026-08-17 ten goals stacked on
    # top of each other jammed the gripper in busy=true with the reported width at the range end.
    inflight = threading.Lock()

    # -- MoveIt ------------------------------------------------------------
    # Second entrance to the same device: the GripperCommand action.  Without
    # it the controller entry in moveit.yaml points at nothing, and a grip
    # command from RViz or MoveGroupInterface runs into a timeout on ``real``
    # instead of into a "cannot do that".  In the mock, gripper_sim serves
    # the same name -- the bridge runs onboard only, so there are never two
    # servers.
    #
    # The gripper does NOT hang off the controller_manager: an action runs in
    # the executor, not in the 8 ms cycle of the CB3.  A blocking XML-RPC call
    # (measured 1.33 s until standstill) would be the end of every arm control
    # loop there.
    from control_msgs.action import GripperCommand
    from rclpy.action import ActionServer
    from rclpy.callback_groups import ReentrantCallbackGroup

    node.declare_parameter("goal_tolerance_m", 0.008)

    def on_action(goal_handle):
        cmd = goal_handle.request.command
        width_m, force_n = goal_to_grip(
            cmd.position, cmd.max_effort, linkage, _p("default_force_n"), _p("force_range_n")
        )
        result = GripperCommand.Result()
        if not inflight.acquire(blocking=False):
            log.warn("GripperCommand rejected: a grip command is still running")
            goal_handle.abort()
            return result
        try:
            last_command[0] = COMMAND_GRIP
            client.grip(width_m, force_n)
            state = await_settled(
                client,
                float(_p("settle_start_timeout_s")),
                float(_p("settle_motion_timeout_s")),
                float(_p("settle_poll_s")),
            )
        except Rg6Error as exc:
            log.error(f"GripperCommand failed: {exc}")
            goal_handle.abort()
            return result
        finally:
            inflight.release()
        fields = goal_result(state, width_m, force_n, linkage, float(_p("goal_tolerance_m")))
        result.position = fields["position"]
        result.effort = fields["effort"]
        result.stalled = fields["stalled"]
        result.reached_goal = fields["reached_goal"]
        log.info(
            f"GripperCommand {width_m * 1000:.0f} mm @ {force_n:.0f} N ─▶ "
            f"{state.width_m * 1000:.1f} mm "
            f"reached={result.reached_goal} stalled={result.stalled}"
        )
        goal_handle.succeed()
        return result

    action_name = str(_p("action_name") or f"{manip_ns}/rg6_gripper_controller/gripper_cmd")
    ActionServer(node, GripperCommand, action_name, on_action, callback_group=ReentrantCallbackGroup())

    log.info(f"rg6_grip_bridge ready: {client.url} ◀─ {action_name}")
    # MultiThreaded because on_action blocks until the hand stands still (about 1.3 s).  Single threaded, that one call
    # would stall /motion/gripper_cmd and every further delivery for the same time.
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--selftest" in argv:
        return selftest()
    return run(argv)


if __name__ == "__main__":
    raise SystemExit(main())
