"""Independent fixed-point and lifecycle oracle for the closed-loop engine.

Checks only the external handshake and numerical contract, not RTL FSM states.
The controller is P-only: the serial planner advances the reference and a
dedicated single-cycle proportional stage publishes the clamped command.
"""
from pathlib import Path
import os
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import Timer
import pytest

VELOCITIES = (205, 410, 1024, 2048, 4096, 5120, 6144, 8191)
ACCELS = (1, 2, 4, 10, 20, 41, 102, 205)
LATENCY_LIMIT = 2000


def signed(signal):
    raw = int(signal.value)
    return raw - (1 << len(signal)) if raw & (1 << (len(signal)-1)) else raw


def clamp(value, limit):
    return min(limit, max(-limit, value))


def advance(position, velocity, stopping, target, vmax, acceleration, negative):
    # Q11 position (scale 2048) and Q11 velocity, so pos += v is exact.
    position += velocity
    # Integer finish: a sub-step residue is snapped exactly onto the target.
    if abs(target - position) < 2048:
        return target, 0, 0
    remaining, speed = abs(target - position), abs(velocity)
    # Exact Q11 brake sum S(s) = s + S(s-a). A candidate is accepted only when
    # its brake sum fits in the remaining distance, so the reference cannot
    # overshoot the target.
    candidate = min(speed + acceleration, vmax)
    # S only grows when the speed actually changes (holding at the limit keeps
    # the same brake sum).
    grown = stopping if candidate == speed else stopping + candidate
    if grown <= remaining:
        speed, stopping = candidate, grown
    elif stopping <= remaining:
        pass
    else:
        stopping -= speed
        if speed < acceleration:
            speed, stopping = 0, 0
        else:
            speed -= acceleration
    return position, (-speed if negative else speed), stopping


class PControl:
    """Integer-error proportional controller: command = ff + Kp*error, clamped."""

    def __init__(self):
        self.command = self.saturated = 0

    def update(self, position, measured, velocity, vmax, kp):
        # Integer-only error: (pos_int - measured) << 11 (Q11). Gain codes
        # 00..11 are 1/8..1, so the Q11 P term is error >> (3 - code).
        error = (position // 2048 - measured) * 2048
        total = velocity + error // (8, 4, 2, 1)[kp]
        self.command = clamp(total, vmax)
        self.saturated = int(total != self.command)


async def clocks(count=1):
    await Timer(10*count, unit="ns")


class Harness:
    def __init__(self, dut):
        self.dut = dut
        self.stable = os.environ["SHARED_STABLE"] == "1"
        self.p = PControl()
        self.ref = (0, 0, 0)
        self.code = self.goal = 0
        self.stopping = 0
        self.negative = False
        self.maximum_latency = 0

    def put(self, **values):
        for name, value in values.items():
            signal = getattr(self.dut, name)
            signal.value = value & ((1 << len(signal))-1)

    def command_output(self):
        return signed(self.dut.velocity_command), int(self.dut.saturated.value)

    def reference_output(self):
        return (signed(self.dut.reference_position), signed(self.dut.reference_velocity),
                int(self.dut.planner_busy.value))

    async def reset(self):
        self.put(rst_n=0, enable=1, control_enable=1, clear_control=0,
                 start=0, tick=0, start_position=0, target_position=0,
                 plan_code=0, kp_select=0)
        await clocks(2)
        self.put(rst_n=1)
        await clocks()
        self.p = PControl()
        self.ref = (0, 0, 0)
        assert self.reference_output() == self.ref
        assert self.command_output() == (0, 0)
        assert not int(self.dut.calculating.value)

    async def launch(self, origin, goal, code):
        self.put(start_position=origin, target_position=goal, plan_code=code,
                 start=1, tick=0, clear_control=0, control_enable=1, enable=1)
        await clocks()
        self.put(start=0)
        self.ref = (origin*2048, 0, int(origin != goal))
        self.goal, self.code, self.p = goal*2048, code, PControl()
        self.stopping = 0
        self.negative = goal < origin
        assert self.reference_output() == self.ref
        assert self.command_output() == (0, 0)
        assert int(self.dut.planner_done.value) == int(origin == goal)
        await clocks()
        assert not int(self.dut.planner_done.value)

    async def tick(self, measured, kp, disturb=True, control=True):
        old_ref, old_command = self.ref, self.command_output()
        vmax, accel = VELOCITIES[self.code & 7], ACCELS[self.code >> 3]
        if control:
            self.p.update(old_ref[0], measured, old_ref[1], vmax, kp)
        else:
            self.p = PControl()
        if old_ref[2]:
            expected_position, expected_velocity, self.stopping = advance(
                old_ref[0], old_ref[1], self.stopping, self.goal, vmax, accel,
                self.negative)
            expected_ref = (expected_position, expected_velocity,
                            int(expected_position != self.goal))
        else:
            expected_ref = old_ref
        self.put(start_position=measured, kp_select=kp,
                 control_enable=int(control), tick=1)
        await clocks()
        self.put(tick=0)
        if disturb:
            # The measured sample is consumed at the acceptance edge. Live
            # configuration is only allowed for STABLE_CONFIG=1.
            self.put(start_position=4095 if measured != 4095 else -4096)
            if not self.stable:
                self.put(kp_select=3-kp, target_position=~(self.goal//2048),
                         plan_code=self.code ^ 63)
        elapsed = 1
        while int(self.dut.calculating.value):
            # The single-cycle P stage has already published; the serial
            # planner must not disturb it while it works.
            assert self.command_output() == (self.p.command, self.p.saturated)
            # Pulse for one clock early in work; holding a tick through
            # completion would legitimately start a fresh operation.
            if disturb and elapsed == 1:
                self.put(tick=1)
                await clocks()
                self.put(tick=0)
                elapsed += 1
            else:
                await clocks(10)
                elapsed += 10
            assert elapsed < LATENCY_LIMIT, (self.code, self.ref, "calculation timeout")
        self.put(tick=0)
        self.maximum_latency = max(self.maximum_latency, elapsed)
        self.ref = expected_ref
        assert self.reference_output() == expected_ref, (self.code, old_ref, self.reference_output(), expected_ref)
        assert self.command_output() == (self.p.command, self.p.saturated), (
            self.code, kp, measured, old_ref, self.command_output(), self.p.__dict__)
        # No tick means no new publication or queued operation.
        await clocks(3)
        assert not int(self.dut.calculating.value)
        assert not int(self.dut.planner_done.value)
        assert self.reference_output() == self.ref
        assert self.command_output() == (self.p.command, self.p.saturated)


@cocotb.test()
async def shared_numerical_contract(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns", impl="gpi").start())
    await Timer(1, unit="ns")
    h = Harness(dut)
    await h.reset()
    rng = random.Random(0x18051)
    # Every velocity/acceleration profile, both directions, and every Kp code.
    cases = [(0, 3, code) for code in range(64)] + [(2, -1, code) for code in range(64)]
    cases += [(-4096, 4095, 63), (4095, -4096, 63), (4095, 4094, 0),
              (-4096, -4095, 7), (0, 0, 0)]
    for origin, target, code in cases:
        await h.launch(origin, target, code)
        for index in range(20000):
            # Integer feedback around the reference exercises fractional errors
            # and saturation without observing RTL internals.
            measured = clamp((h.ref[0] // 2048) + rng.choice((-8, -1, 0, 0, 1, 7)), 4095)
            measured = max(-4096, measured)
            kp = (index // 4 + code) % 4
            await h.tick(measured, kp)
            if not h.ref[2]:
                # Integer-only finish: frac <1 step error accepted per numeric-spec.
                assert h.ref[0] == target*2048 and h.ref[1] == 0, (h.ref, target)
                # Disturbance recovery must retain the completed reference.
                for offset in (-4096, 4095, target, target):
                    await h.tick(offset, 3)
                break
        else:
            raise AssertionError(("move timeout", origin, target, code))
    dut._log.info("Maximum observed engine latency: %d clocks", h.maximum_latency)


async def wait_idle(dut, limit=LATENCY_LIMIT):
    for _ in range(limit//10):
        if not int(dut.calculating.value):
            return
        await clocks(10)
    raise AssertionError("engine stalled")


@cocotb.test()
async def shared_cancellation_contract(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns", impl="gpi").start())
    await Timer(1, unit="ns")
    h = Harness(dut)
    await h.reset()
    # A one-clock clear anywhere in the planner pass must zero the command,
    # preserve the pending planner advance, and recover on the next valid tick.
    for moving in (False, True):
        for abort in ("clear_control", "control_enable"):
            for offset in range(0, 128, 4):
                await h.reset()
                await h.launch(-2, 20 if moving else -2, 63)
                await h.tick(-3, 2)
                old_ref = h.ref
                if moving:
                    moved = advance(old_ref[0], old_ref[1], h.stopping, h.goal,
                                    VELOCITIES[7], ACCELS[7], h.negative)
                    expected = (moved[0], moved[1], int(moved[0] != h.goal))
                    expected_stopping = moved[2]
                else:
                    expected, expected_stopping = old_ref, h.stopping
                # Start a tick, optionally let the P publication happen, then
                # abort regardless of whether the planner is still busy.
                h.put(start_position=-4, kp_select=1, tick=1)
                await clocks()
                h.put(tick=0)
                if offset:
                    await clocks(offset)
                h.put(**{abort: 1 if abort == "clear_control" else 0})
                await clocks()
                assert h.command_output() == (0, 0), (abort, offset)
                # Release after one edge; the command must stay zero until a
                # fresh accepted tick.
                h.put(clear_control=0, control_enable=1)
                for _ in range(LATENCY_LIMIT//10):
                    assert h.command_output() == (0, 0), (abort, offset, "stale publication")
                    if not int(dut.calculating.value):
                        break
                    await clocks(10)
                else:
                    raise AssertionError((abort, offset, "cancellation stalled"))
                assert h.reference_output() == expected, (abort, offset, moving,
                                                          h.reference_output(), expected)
                h.ref, h.p, h.stopping = expected, PControl(), expected_stopping
                await h.tick(-2, 1)
    # Global disable/reset aborts the move and clears the command.
    for abort in ("enable", "rst_n"):
        await h.reset()
        await h.launch(-2, 20, 63)
        await h.tick(-3, 2)
        h.put(start_position=-4, tick=1, kp_select=1)
        await clocks()
        h.put(tick=0)
        await clocks()
        assert int(dut.calculating.value)
        h.put(**{abort: 0})
        await clocks()
        assert h.command_output() == (0, 0)
        assert not int(dut.pi_calculating.value)
        assert not int(dut.calculating.value)
        assert not int(dut.planner_busy.value)
        assert signed(dut.reference_velocity) == 0
        h.put(enable=1, rst_n=1)
        await clocks(LATENCY_LIMIT)
        assert not int(dut.calculating.value)
        assert h.command_output() == (0, 0)
        assert signed(dut.reference_velocity) == 0
    # At a completed reference, feedback recovery is P-only and clear cancels
    # the correction without moving the reference.
    await h.reset()
    await h.launch(3, 3, 63)
    h.put(start_position=2, kp_select=3, tick=1)
    await clocks()
    h.put(tick=0)
    await wait_idle(dut)
    assert h.command_output() != (0, 0)
    h.put(clear_control=1)
    await clocks()
    assert h.command_output() == (0, 0)
    h.put(clear_control=0)
    await clocks(LATENCY_LIMIT)
    assert not int(dut.calculating.value)
    assert h.reference_output() == (3*2048, 0, 0)
    assert h.command_output() == (0, 0)
    h.p = PControl()
    await h.tick(2, 3)
    # A disabled controller may continue planning without producing P output.
    await h.reset()
    await h.launch(0, 1, 63)
    for _ in range(5):
        await h.tick(0, 3, control=False)
    assert h.command_output() == (0, 0)
    await h.tick(0, 3)


@pytest.mark.parametrize("stable_config", [0, 1])
def test_shared_controller(tmp_path, stable_config):
    from cocotb_tools.runner import get_runner
    root = Path(__file__).resolve().parents[1]
    runner = get_runner("icarus")
    runner.build(sources=[root / "src" / "shared_motion_controller.v",
                          root / "src" / "motion_engine.v",
                          root / "src" / "pi_p_controller.v"],
                 includes=[root / "src"], hdl_toplevel="shared_motion_controller",
                 parameters={"STABLE_CONFIG": stable_config},
                 build_dir=tmp_path / "shared", timescale=("1ns", "1ps"), always=True)
    runner.test(hdl_toplevel="shared_motion_controller", test_module="test_shared_controller",
                test_dir=Path(__file__).parent, results_xml=str(tmp_path / "results.xml"),
                extra_env={"SHARED_STABLE": str(stable_config)})
