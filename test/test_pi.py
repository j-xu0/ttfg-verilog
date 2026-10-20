"""PI numeric contract, saturation, anti-windup, and sequential controls."""
from pathlib import Path
import random
import os

import pytest

import cocotb
from cocotb.triggers import Timer


async def clock(dut):
    dut.clk.value = 0
    await Timer(5, unit="ns")
    dut.clk.value = 1
    await Timer(5, unit="ns")


def signed(signal):
    raw = int(signal.value)
    return raw - (1 << len(signal)) if raw & (1 << (len(signal) - 1)) else raw


def clamp(value, limit):
    return max(-limit, min(limit, value))


class Model:
    def __init__(self):
        self.integral = self.command = self.saturated = 0

    def update(self, p):
        if not p["rst_n"] or p["clear"] or not p["enable"]:
            self.integral = self.command = self.saturated = 0
        elif p["tick"]:
            error = p["reference_position"] - p["measured_position"] * 65536
            # Floor division is independent of the RTL arithmetic-shift operator.
            proportional = error // (8, 4, 2, 1)[p["kp_select"]]
            delta = error // (8, 4, 2, 1)[p["ki_select"]]
            total = p["feedforward_velocity"] + proportional + self.integral
            limit = p["max_velocity"]
            self.command = clamp(total, limit)
            self.saturated = int(total != self.command)
            if (total > limit and error > 0) or (total < -limit and error < 0):
                self.integral = clamp(self.integral, limit)
            else:
                self.integral = clamp(self.integral + delta, limit)


@cocotb.test()
async def pi_contract(dut):
    p = dict(rst_n=0, enable=1, clear=0, tick=0,
             reference_position=0, measured_position=0,
             feedforward_velocity=0, max_velocity=655360, kp_select=0, ki_select=0)
    model = Model()
    serial = os.environ.get("PI_SERIAL", "0") == "1"
    stable = os.environ.get("PI_STABLE", "0") == "1"

    async def step(**changes):
        p.update(changes)
        for name, value in p.items():
            signal = getattr(dut, name)
            signal.value = value & ((1 << len(signal)) - 1)
        previous = model.command, model.saturated
        model.update(p)
        await clock(dut)
        if serial and p["rst_n"] and p["enable"] and not p["clear"] and p["tick"]:
            assert int(dut.calculating.value) == 1
            # Captured numeric inputs must be immune to changes during work;
            # extra ticks while busy are ignored rather than queuing samples.
            for name in ("reference_position", "measured_position", "feedforward_velocity",
                         "max_velocity", "kp_select", "ki_select"):
                if not stable or name not in ("max_velocity", "kp_select", "ki_select"):
                    getattr(dut, name).value = 0
            for cycle in range(450):
                assert (signed(dut.velocity_command), int(dut.saturated.value)) == previous
                await clock(dut)
                if not int(dut.calculating.value):
                    break
            else:
                raise AssertionError("serial calculation exceeded latency bound")
        actual = signed(dut.velocity_command), int(dut.saturated.value)
        assert actual == (model.command, model.saturated), (p.copy(), actual, model.__dict__)
        assert signed(dut.integral) == model.integral, (p.copy(), model.__dict__)
        if p["rst_n"] and p["enable"] and not p["clear"] and p["tick"]:
            assert abs(actual[0]) <= p["max_velocity"]
            assert abs(signed(dut.integral)) <= p["max_velocity"]
        return actual

    await step()
    await step(rst_n=1)
    # All gain pairs, signed fractions (including negative sub-LSBs), and
    # measured integer subtraction. Each case starts with zero integral.
    for kp in range(4):
        for ki in range(4):
            for error in (0, 1, -1, 7, -7, 65536, -65536, 131071, -131071):
                await step(clear=1, tick=0)
                await step(clear=0, tick=1, kp_select=kp, ki_select=ki,
                           reference_position=3 * 65536 + error, measured_position=3,
                           feedforward_velocity=300)
                for _ in range(4):
                    await step()
    # Widened subtraction at both signed position endpoints must saturate in
    # the correct direction instead of wrapping back through zero.
    for reference, measured, direction in (((1 << 28) - 1, -4096, 1),
                                            (-(1 << 28), 4095, -1)):
        await step(clear=1)
        actual = await step(clear=0, reference_position=reference,
                            measured_position=measured, kp_select=3, ki_select=3)
        assert actual == (direction * 655360, 1)
        assert signed(dut.integral) == 0
        for _ in range(20):
            await step()
        # No stored windup: removing the error immediately gives feedforward.
        await step(reference_position=0, measured_position=0, feedforward_velocity=0)
        assert signed(dut.velocity_command) == 0

    # Reach integral limit without command windup by offsetting feedforward;
    # Ki > Kp also exercises candidate clamp before saturation of output.
    for direction in (1, -1):
        await step(clear=1)
        await step(clear=0, reference_position=direction * 65536, measured_position=0,
                   kp_select=0, ki_select=3, max_velocity=100000,
                   feedforward_velocity=-direction * 90000)
        for _ in range(8):
            await step()
        assert signed(dut.integral) == direction * 100000
        await step(max_velocity=1000, feedforward_velocity=0)
        assert signed(dut.integral) == direction * 1000
        # Opposite error integrates out of saturation instead of remaining stuck.
        await step(reference_position=-direction * 8)
        assert signed(dut.integral) == direction * 992

    # Feedforward endpoint clipping, zero ceiling, and exact ceiling equality.
    for ff in (-(1 << 20), (1 << 20) - 1, -655360, 655360):
        await step(clear=1)
        await step(clear=0, reference_position=0, measured_position=0,
                   max_velocity=655360, feedforward_velocity=ff)
        assert signed(dut.velocity_command) == clamp(ff, 655360)
    await step(max_velocity=0)
    assert signed(dut.velocity_command) == 0

    # Tick gates all state. Clear and disable act on the very next edge.
    await step(clear=1)
    await step(clear=0, tick=1, reference_position=65536, max_velocity=655360,
               feedforward_velocity=0)
    previous = (model.integral, model.command, model.saturated)
    await step(tick=0, reference_position=-(1 << 28), max_velocity=0)
    assert (model.integral, model.command, model.saturated) == previous
    await step(enable=0)
    assert signed(dut.velocity_command) == signed(dut.integral) == 0
    await step(enable=1, tick=1, reference_position=65536, max_velocity=655360)
    await step(clear=1, tick=0)
    assert signed(dut.velocity_command) == signed(dut.integral) == 0

    rng = random.Random(0x18091)
    for _ in range(1200):
        await step(clear=int(rng.randrange(40) == 0), enable=int(rng.randrange(20) != 0),
                   tick=int(rng.randrange(4) != 0),
                   reference_position=rng.choice((rng.randrange(-(1 << 28), 1 << 28),
                                                   rng.randrange(-200000, 200000))),
                   measured_position=rng.choice((0, rng.randrange(-4096, 4096))),
                   feedforward_velocity=rng.randrange(-(1 << 20), 1 << 20),
                   max_velocity=rng.choice((0, 1, 655360, (1 << 20) - 1,
                                            rng.randrange(1 << 20))),
                   kp_select=rng.randrange(4), ki_select=rng.randrange(4))

    if serial:
        # Both clear and disable cancel partially evaluated samples. No stale
        # result can reappear after re-enabling without a fresh tick.
        for abort in ("clear", "enable", "rst_n"):
            await step(clear=1, tick=0)
            await step(clear=0, enable=1, rst_n=1, tick=0)
            dut.tick.value = 1
            await clock(dut)
            dut.tick.value = 0
            for _ in range(53):
                await clock(dut)
            assert int(dut.calculating.value)
            getattr(dut, abort).value = 1 if abort == "clear" else 0
            await clock(dut)
            assert not int(dut.calculating.value)
            assert signed(dut.velocity_command) == signed(dut.integral) == 0
            dut.clear.value = 0
            dut.enable.value = 1
            dut.rst_n.value = 1
            for _ in range(450):
                await clock(dut)
                assert signed(dut.velocity_command) == signed(dut.integral) == 0

    # Reset is asynchronous, independent of both clock and tick.
    await step(clear=0, enable=1, tick=1, reference_position=65536,
               measured_position=0, max_velocity=655360, feedforward_velocity=100)
    dut.clk.value = 0
    dut.rst_n.value = 0
    await Timer(1, unit="ns")
    assert signed(dut.velocity_command) == signed(dut.integral) == int(dut.saturated.value) == 0


@pytest.mark.parametrize("serial,stable", [(0, 0), (1, 0), (1, 1)])
def test_pi(tmp_path, serial, stable):
    from cocotb_tools.runner import get_runner
    root = Path(__file__).resolve().parents[1]
    runner = get_runner("icarus")
    runner.build(sources=[root / "src" / "pi_controller.v"],
                 hdl_toplevel="pi_controller", build_dir=tmp_path / "pi_build",
                 timescale=("1ns", "1ps"), parameters={"SERIAL": serial, "STABLE_CONFIG": stable}, always=True)
    runner.test(hdl_toplevel="pi_controller", test_module="test_pi",
                test_dir=Path(__file__).parent,
                results_xml=str(tmp_path / "pi_results.xml"),
                extra_env={"PI_SERIAL": str(serial), "PI_STABLE": str(stable)})
