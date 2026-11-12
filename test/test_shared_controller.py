"""Independent fixed-point and lifecycle oracle for the shared control engine.

Checks only the external handshake and numerical contract, not RTL FSM states.
"""
from functools import lru_cache
from pathlib import Path
import os
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import Timer
import pytest

VELOCITIES = (6554, 13107, 32768, 65536, 131072, 262144, 458752, 655360)
ACCELS = (33, 66, 131, 328, 655, 1311, 3277, 6554)
LATENCY_LIMIT = 2000


def signed(signal):
    raw = int(signal.value)
    return raw - (1 << len(signal)) if raw & (1 << (len(signal)-1)) else raw


def clamp(value, limit):
    return min(limit, max(-limit, value))


@lru_cache(maxsize=32768)
def stopping_distance(speed, acceleration):
    return sum(range(speed, 0, -acceleration))


def advance(position, velocity, target, vmax, acceleration):
    position += velocity
    if position == target:
        return position, 0, 0
    remaining, speed = abs(target-position), abs(velocity)
    next_speed = max(speed-acceleration, 0)
    for candidate in (min(speed+acceleration, vmax), speed, next_speed):
        if stopping_distance(candidate, acceleration) <= remaining:
            next_speed = candidate
            break
    if speed <= acceleration and remaining <= min(acceleration, vmax):
        next_speed = remaining
    return position, next_speed if target > position else -next_speed, 1


class PI:
    def __init__(self):
        self.integral = self.command = self.saturated = 0

    def update(self, position, measured, velocity, vmax, kp, ki):
        error = position - measured*65536
        total = velocity + error // (8, 4, 2, 1)[kp] + self.integral
        self.command = clamp(total, vmax)
        self.saturated = int(total != self.command)
        if (total > vmax and error > 0) or (total < -vmax and error < 0):
            self.integral = clamp(self.integral, vmax)
        else:
            self.integral = clamp(self.integral + error // (8, 4, 2, 1)[ki], vmax)


async def clocks(count=1):
    await Timer(10*count, unit="ns")


class Harness:
    def __init__(self, dut):
        self.dut = dut
        self.stable = os.environ["SHARED_STABLE"] == "1"
        self.pi = PI()
        self.ref = (0, 0, 0)
        self.code = self.goal = 0
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
                 plan_code=0, kp_select=0, ki_select=0)
        await clocks(2)
        self.put(rst_n=1)
        await clocks()
        self.pi = PI()
        self.ref = (0, 0, 0)
        assert self.reference_output() == self.ref
        assert self.command_output() == (0, 0)
        assert not int(self.dut.calculating.value)

    async def launch(self, origin, goal, code):
        self.put(start_position=origin, target_position=goal, plan_code=code,
                 start=1, tick=0, clear_control=0, control_enable=1, enable=1)
        await clocks()
        self.put(start=0)
        self.ref = (origin*65536, 0, int(origin != goal))
        self.goal, self.code, self.pi = goal*65536, code, PI()
        assert self.reference_output() == self.ref
        assert self.command_output() == (0, 0)
        assert int(self.dut.planner_done.value) == int(origin == goal)
        await clocks()
        assert not int(self.dut.planner_done.value)

    async def tick(self, measured, kp, ki, disturb=True, control=True):
        old_ref, old_command = self.ref, self.command_output()
        vmax, accel = VELOCITIES[self.code & 7], ACCELS[self.code >> 3]
        if control:
            self.pi.update(old_ref[0], measured, old_ref[1], vmax, kp, ki)
        else:
            self.pi = PI()
        expected_ref = advance(*old_ref[:2], self.goal, vmax, accel) if old_ref[2] else old_ref
        self.put(start_position=measured, kp_select=kp, ki_select=ki,
                 control_enable=int(control), tick=1)
        await clocks()
        self.put(tick=0)
        if disturb:
            # These must be captured at the acceptance edge. Configuration may
            # be live only for STABLE_CONFIG=1, as specified by the module.
            self.put(start_position=4095 if measured != 4095 else -4096)
            if not self.stable:
                self.put(kp_select=3-kp, ki_select=3-ki,
                         target_position=~(self.goal//65536), plan_code=self.code ^ 63)
        elapsed = 1
        while int(self.dut.calculating.value):
            if int(self.dut.pi_calculating.value):
                assert self.command_output() == old_command
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
        assert self.command_output() == (self.pi.command, self.pi.saturated), (
            self.code, kp, ki, measured, old_ref, self.command_output(), self.pi.__dict__)
        # No tick means no new publication or queued operation.
        await clocks(3)
        assert not int(self.dut.calculating.value)
        assert not int(self.dut.planner_done.value)
        assert self.reference_output() == self.ref
        assert self.command_output() == (self.pi.command, self.pi.saturated)


@cocotb.test()
async def shared_numerical_contract(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns", impl="gpi").start())
    await Timer(1, unit="ns")
    h = Harness(dut)
    await h.reset()
    rng = random.Random(0x18051)
    # Every velocity/acceleration profile, both directions, and every gain pair.
    cases = [(0, 3, code) for code in range(64)] + [(2, -1, code) for code in range(64)]
    cases += [(-4096, 4095, 63), (4095, -4096, 63), (4095, 4094, 0),
              (-4096, -4095, 7), (0, 0, 0)]
    for origin, target, code in cases:
        await h.launch(origin, target, code)
        for index in range(20000):
            # Integer feedback around the reference exercises fractional errors,
            # saturation, anti-windup and sign changes without observing RTL internals.
            measured = clamp((h.ref[0] // 65536) + rng.choice((-8, -1, 0, 0, 1, 7)), 4095)
            measured = max(-4096, measured)
            kp, ki = (index // 4 + code) % 4, (index + code) % 4
            await h.tick(measured, kp, ki)
            if not h.ref[2]:
                assert h.ref == (target*65536, 0, 0)
                # PI-only disturbance recovery must safely reuse scratch storage
                # after planner completion, retaining the completed reference.
                for offset in (-4096, 4095, target, target):
                    await h.tick(offset, 3, 0)
                break
        else:
            raise AssertionError(("move timeout", origin, target, code))
    dut._log.info("Maximum observed shared-engine latency: %d clocks", h.maximum_latency)


@cocotb.test()
async def shared_cancellation_contract(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns", impl="gpi").start())
    await Timer(1, unit="ns")
    h = Harness(dut)
    await h.reset()
    # Cancel in both PI and planner phases. Global disable/reset aborts the move;
    # PI clear/disable cancels only PI and preserves the pending planner update.
    for phase in ("pi_calculating", "planner_calculating"):
        for abort in ("clear_control", "control_enable", "enable", "rst_n"):
            await h.reset()
            await h.launch(-2, 20, 63)
            await h.tick(-3, 2, 3)
            old_ref = h.ref
            self_expected = advance(*old_ref[:2], h.goal, VELOCITIES[7], ACCELS[7])
            h.put(start_position=-4, tick=1)
            await clocks()
            h.put(tick=0)
            for _ in range(LATENCY_LIMIT):
                if (int(dut.pi_calculating.value) if phase == "pi_calculating" else
                        int(dut.calculating.value) and not int(dut.pi_calculating.value)):
                    break
                await clocks()
            else:
                raise AssertionError(("phase not reached", phase))
            await clocks(7)
            assert int(dut.calculating.value)
            assert int(dut.pi_calculating.value) == int(phase == "pi_calculating")
            h.put(**{abort: 1 if abort == "clear_control" else 0})
            await clocks()
            assert h.command_output() == (0, 0)
            if abort in ("enable", "rst_n"):
                assert not int(dut.pi_calculating.value)
                assert not int(dut.calculating.value)
                assert not int(dut.planner_busy.value)
                assert signed(dut.reference_velocity) == 0
                h.put(enable=1, rst_n=1)
                await clocks(LATENCY_LIMIT)
                assert not int(dut.calculating.value)
                assert h.command_output() == (0, 0)
                assert signed(dut.reference_velocity) == 0
            else:
                for _ in range(LATENCY_LIMIT//10):
                    if not int(dut.calculating.value):
                        break
                    await clocks(10)
                else:
                    raise AssertionError("PI cancellation stalled planner")
                assert h.reference_output() == self_expected
                assert h.command_output() == (0, 0)
                h.ref, h.pi = self_expected, PI()
                h.put(clear_control=0, control_enable=1)
                await clocks(20)
                assert h.reference_output() == self_expected
                assert h.command_output() == (0, 0)
                # A fresh sample must remain correct after cancellation.
                await h.tick(-2, 1, 2)
    # At a completed reference, feedback may return to the target during
    # PI-only work. Clear must cancel this correction without moving reference
    # or leaving temporary error bits exposed as a feedforward velocity.
    await h.reset()
    await h.launch(3, 3, 63)
    h.put(start_position=2, kp_select=3, ki_select=3, tick=1)
    await clocks()
    h.put(tick=0)
    await clocks(53)
    assert int(dut.pi_calculating.value)
    h.put(start_position=3, clear_control=1)
    await clocks()
    assert h.command_output() == (0, 0)
    h.put(clear_control=0)
    await clocks(LATENCY_LIMIT)
    assert not int(dut.calculating.value)
    assert h.reference_output() == (3*65536, 0, 0)
    assert h.command_output() == (0, 0)
    await h.tick(2, 3, 3)
    # Sweep every edge across the initial serial error subtraction and PI
    # sum. A one-clock cancellation pulse must stay latched until any in-place
    # register rotation is restored, preserving both active and idle references.
    for moving in (False, True):
        for abort in ("clear_control", "control_enable"):
            for offset in range(96):
                await h.reset()
                await h.launch(-2, 20 if moving else -2, 63)
                await h.tick(-3, 2, 3)
                old_ref = h.ref
                expected = (advance(*old_ref[:2], h.goal, VELOCITIES[7], ACCELS[7])
                            if moving else old_ref)
                h.put(start_position=-4, tick=1)
                await clocks()
                h.put(tick=0)
                if offset:
                    await clocks(offset)
                assert int(dut.pi_calculating.value), (offset, "expected unfinished PI")
                h.put(**{abort: 1 if abort == "clear_control" else 0})
                await clocks()
                assert h.command_output() == (0, 0), (abort, offset)
                # Release after one edge. The cancellation must persist while
                # scratch storage finishes restoring its ordinary bit order.
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
                h.ref, h.pi = expected, PI()
                await h.tick(-2, 1, 2)
    # A disabled controller may continue planning without producing PI output.
    await h.reset()
    await h.launch(0, 1, 63)
    for _ in range(5):
        await h.tick(0, 3, 3, control=False)
    assert h.command_output() == (0, 0)
    await h.tick(0, 3, 3)


@pytest.mark.parametrize("stable_config", [0, 1])
def test_shared_controller(tmp_path, stable_config):
    from cocotb_tools.runner import get_runner
    root = Path(__file__).resolve().parents[1]
    runner = get_runner("icarus")
    runner.build(sources=[root / "src" / "shared_motion_controller.v"],
                 includes=[root / "src"], hdl_toplevel="shared_motion_controller",
                 parameters={"STABLE_CONFIG": stable_config},
                 build_dir=tmp_path / "shared", timescale=("1ns", "1ps"), always=True)
    runner.test(hdl_toplevel="shared_motion_controller", test_module="test_shared_controller",
                test_dir=Path(__file__).parent, results_xml=str(tmp_path / "results.xml"),
                extra_env={"SHARED_STABLE": str(stable_config)})
