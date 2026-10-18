"""Discrete planner model and independent invariants: pytest test/test_planner.py."""
from pathlib import Path
import random
from functools import lru_cache
import pytest

import cocotb
from cocotb.triggers import Timer


async def clock(dut, tick=0):
    dut.tick.value = tick
    dut.clk.value = 0
    await Timer(5, unit="ns")
    dut.clk.value = 1
    await Timer(5, unit="ns")


def signed(signal):
    raw = int(signal.value)
    return raw - (1 << len(signal)) if raw & (1 << (len(signal) - 1)) else raw


def scale(dut):
    return 1 << (len(dut.reference_position) - 13)


def state(dut):
    return (signed(dut.reference_position), signed(dut.reference_velocity),
            int(dut.busy.value), int(dut.done.value))


async def reset(dut):
    dut.enable.value = 1
    dut.start.value = 0
    dut.tick.value = 0
    dut.start_position.value = 0
    dut.target_position.value = 0
    dut.max_velocity.value = 0
    dut.acceleration.value = 0
    dut.rst_n.value = 0
    await clock(dut)
    dut.rst_n.value = 1
    assert state(dut) == (0, 0, 0, 0)


async def command(dut, origin, target, vmax, accel):
    dut.start_position.value = origin & 8191
    dut.target_position.value = target & 8191
    dut.max_velocity.value = vmax
    dut.acceleration.value = accel
    dut.start.value = 1
    await clock(dut)
    dut.start.value = 0
    immediate = origin == target or vmax == 0 or accel == 0
    assert state(dut) == (origin * scale(dut), 0, not immediate, immediate)
    # Preparation deliberately sees tick and changing command inputs. A bounded
    # serial setup must preserve the accepted origin/zero velocity and busy.
    cycles = 0
    while int(dut.preparing.value):
        dut.start_position.value = (-origin) & 8191
        dut.target_position.value = (-target) & 8191
        dut.max_velocity.value = 0
        dut.acceleration.value = 0
        before = state(dut)
        await clock(dut, tick=1)
        assert state(dut) == before
        cycles += 1
        assert cycles <= 3 * len(dut.max_velocity) + 2


@lru_cache(maxsize=32768)
def stop_sum(speed, accel):
    # Independent oracle deliberately sums individual discrete deceleration
    # intervals rather than using the RTL's closed-form quotient/products.
    return sum(range(speed, 0, -accel))


def model_next(pos, speed, goal, vmax, accel):
    pos += speed
    direction = 1 if goal >= pos else -1
    remaining = abs(goal - pos)
    if not remaining:
        return pos, 0, 0, 1
    magnitude = abs(speed)
    candidates = (min(magnitude + accel, vmax), magnitude,
                  max(magnitude - accel, 0))
    next_speed = candidates[-1]
    for candidate in candidates:
        if stop_sum(candidate, accel) <= remaining:
            next_speed = candidate
            break
    if magnitude <= accel and remaining <= min(accel, vmax):
        next_speed = remaining
    return pos, direction * next_speed, 1, 0


async def move(dut, origin, target, vmax, accel, disturb=False):
    await command(dut, origin, target, vmax, accel)
    if origin == target or vmax == 0 or accel == 0:
        await clock(dut)
        assert state(dut) == (origin * scale(dut), 0, 0, 0)
        return []
    goal = target * scale(dut)
    previous = state(dut)
    velocities = []
    max_ticks = abs(goal - previous[0]) // max(1, min(accel, vmax)) + 5000
    for index in range(max_ticks):
        if disturb and index % 7 == 0:
            # All command inputs may change while busy; the active move is latched.
            dut.start.value = 1
            dut.start_position.value = (-target) & 8191
            dut.target_position.value = (-origin) & 8191
            dut.max_velocity.value = 0
            dut.acceleration.value = 0
        else:
            dut.start.value = 0
        if index % 5 == 0:
            await clock(dut, tick=0)
            assert state(dut) == previous
        expected = model_next(previous[0], previous[1], goal, vmax, accel)
        await clock(dut, tick=1)
        actual = state(dut)
        assert actual == expected, (origin, target, vmax, accel, index, actual, expected)
        # Independent physical invariants, not derived from model_next.
        assert actual[0] - previous[0] == previous[1]
        assert min(origin * scale(dut), goal) <= actual[0] <= max(origin * scale(dut), goal)
        assert abs(actual[1]) <= vmax
        assert abs(actual[1] - previous[1]) <= accel
        assert actual[1] * (target - origin) >= 0
        assert abs(goal - actual[0]) <= abs(goal - previous[0])
        velocities.append(abs(actual[1]))
        previous = actual
        if actual[3]:
            assert actual == (goal, 0, 0, 1)
            dut.start.value = 0
            await clock(dut, tick=0)
            assert state(dut) == (goal, 0, 0, 0)
            return velocities
    raise AssertionError(f"move did not finish: {(origin, target, vmax, accel)}")


@cocotb.test()
async def planner_contract(dut):
    await reset(dut)
    if scale(dut) == 256:
        # A long move reaches the ceiling, cruises, then decelerates.
        trace = await move(dut, 0, 100, 1024, 128, disturb=True)
        assert trace.count(1024) > 3
        assert any(a < b for a, b in zip(trace, trace[1:]))
        assert any(a > b for a, b in zip(trace, trace[1:]))
        cases = ((100, -100, 1000, 73), (0, 1, 65535, 65535),
                 (0, -1, 127, 256), (4095, -4096, 65535, 65535),
                 (-4096, 4095, 65535, 1), (-4096, -4095, 1, 1),
                 (4095, 4094, 65535, 319), (50, 50, 1, 1),
                 (0, 10, 0, 50), (10, -10, 50, 0), (0, 1, 0, 0))
    else:
        # At 1 kHz, Q16 raw 33 is 503.54 steps/s^2. This precision
        # prevents the configured 500 steps/s^2 floor rounding to zero.
        trace = await move(dut, 0, 100, 6554, 33, disturb=True)
        assert trace.count(6554) > 3  # Approximately 100 steps/s cruise.
        assert max(trace) == 6554
        # Full signed travel in both directions, highest supported physical
        # speed (10,000 steps/s), extreme raw limits, and fractional tails.
        cases = ((-4096, 4095, 655360, 6554),
                 (4095, -4096, 655360, 6554),
                 (-4096, 4095, 655360, 33),
                 (0, 1, 1048575, 1048575), (0, -1, 127, 256),
                 (4095, 4094, 655360, 319), (50, 50, 1, 1),
                 (0, 10, 0, 50), (10, -10, 50, 0), (0, 1, 0, 0))
    for case in cases:
        await move(dut, *case)
    rng = random.Random(0x180)
    for _ in range(55):
        origin = rng.randint(-4096, 4095)
        target = max(-4096, min(4095, origin + rng.randint(-30, 30)))
        if scale(dut) == 256:
            vmax = rng.randint(1, 65535)
            accel = rng.choice((1, 3, 255, 256, 1000, 65535))
        else:
            vmax = rng.randint(6554, 655360)
            accel = rng.choice((33, 66, 255, 1000, 6554, 1048575))
        await move(dut, origin, target, vmax, accel, disturb=True)

    # A serial setup can be aborted before its cached stopping limit exists.
    if int(dut.SERIAL_SETUP.value):
        dut.start_position.value = 0
        dut.target_position.value = 100
        dut.max_velocity.value = 655360
        dut.acceleration.value = 33
        dut.start.value = 1
        await clock(dut)
        dut.start.value = 0
        assert int(dut.preparing.value)
        dut.enable.value = 0
        await clock(dut, tick=1)
        assert state(dut) == (0, 0, 0, 0)
        assert not int(dut.preparing.value)
        dut.enable.value = 1
        # Reset must also cancel preparation, with no stale cached cap reused.
        dut.max_velocity.value = 655360
        dut.acceleration.value = 33
        dut.start.value = 1
        await clock(dut)
        dut.start.value = 0
        assert int(dut.preparing.value)
        await reset(dut)
        assert not int(dut.preparing.value)
        # A later command must calculate its own stopping cap.
        await move(dut, 1, -3, 6554, 65)

    # Abort is immediate without tick, never falsely reports completion.
    await command(dut, -100, 100, 512, 100)
    for _ in range(8):
        await clock(dut, tick=1)
    pos = signed(dut.reference_position)
    dut.enable.value = 0
    dut.start.value = 1
    await clock(dut, tick=0)
    assert state(dut) == (pos, 0, 0, 0)
    await clock(dut, tick=1)
    assert state(dut) == (pos, 0, 0, 0)
    dut.start.value = 0
    dut.enable.value = 1
    await clock(dut, tick=1)
    assert state(dut) == (pos, 0, 0, 0)
    await command(dut, 1, 5, 500, 100)
    await clock(dut, tick=1)
    dut.rst_n.value = 0
    await clock(dut, tick=0)
    assert state(dut) == (0, 0, 0, 0)


@pytest.mark.parametrize("fraction_bits,limit_width,serial_setup", [(8, 16, 0), (16, 20, 0), (16, 20, 1)])
def test_planner(tmp_path, fraction_bits, limit_width, serial_setup):
    from cocotb_tools.runner import get_runner
    root = Path(__file__).resolve().parents[1]
    runner = get_runner("icarus")
    runner.build(sources=[root / "src" / "motion_planner.v"],
                 hdl_toplevel="motion_planner",
                 parameters={"FRACTION_BITS": fraction_bits, "LIMIT_WIDTH": limit_width, "SERIAL_SETUP": serial_setup},
                 build_dir=tmp_path / "planner_build",
                 timescale=("1ns", "1ps"), always=True)
    runner.test(hdl_toplevel="motion_planner", test_module="test_planner",
                test_dir=Path(__file__).parent,
                results_xml=str(tmp_path / "planner_results.xml"))
