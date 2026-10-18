"""Pulse timing, DDS rate and abort invariants: pytest test/test_step_dir.py."""
from pathlib import Path
import json
import os

import cocotb
from cocotb.triggers import Timer
import pytest


async def cycle(dut):
    dut.clk.value = 0
    await Timer(5, unit="ns")
    dut.clk.value = 1
    await Timer(5, unit="ns")
    return int(dut.step.value), int(dut.dir.value)


class Monitor:
    def __init__(self, dut, params):
        self.dut, self.p = dut, params
        self.time = 0
        self.previous = 0
        self.direction = int(dut.dir.value)
        self.rise = self.fall = self.dir_change = None
        self.rises = []

    async def run(self, count):
        for _ in range(count):
            step, direction = await cycle(self.dut)
            self.time += 1
            if direction != self.direction:
                assert not step and not self.previous
                if self.fall is not None:
                    assert self.time - self.fall >= self.p["DIR_HOLD_CYCLES"]
                self.dir_change = self.time
            if step and not self.previous:
                if self.fall is not None:
                    assert self.time - self.fall >= self.p["STEP_LOW_CYCLES"]
                if self.dir_change is not None:
                    assert self.time - self.dir_change >= self.p["DIR_SETUP_CYCLES"]
                assert self.time >= self.p["ENABLE_SETUP_CYCLES"]
                self.rise = self.time
                self.rises.append((self.time, direction))
            if self.previous and not step:
                assert self.time - self.rise == self.p["STEP_HIGH_CYCLES"]
                self.fall = self.time
            self.previous, self.direction = step, direction


async def reset(dut):
    dut.clk.value = 0
    dut.enable.value = 0
    dut.fault.value = 0
    dut.velocity_command.value = 0
    dut.rst_n.value = 0
    await cycle(dut)
    dut.rst_n.value = 1
    dut.enable.value = 1


def command(dut, value):
    dut.velocity_command.value = value & ((1 << 21) - 1)


@cocotb.test()
async def pulse_contract(dut):
    p = json.loads(os.environ["STEP_TEST_PARAMS"])
    for velocity in (65536, -65536, 98305, -98305, -(1 << 20), (1 << 20)-1):
        await reset(dut)
        command(dut, velocity)
        monitor = Monitor(dut, p)
        await monitor.run(1500)
        before = len(monitor.rises)
        await monitor.run(12000)
        observed = len(monitor.rises) - before
        capped = min(abs(velocity), p["MAX_STEP_HZ"] * 65536 // p["CONTROL_HZ"])
        expected = 12000 * capped * p["CONTROL_HZ"] / (p["CLOCK_HZ"] * 65536)
        assert abs(observed - expected) <= 1, (velocity, observed, expected)
        assert all(direction == (velocity > 0) for _, direction in monitor.rises)
        assert int(dut.rate_limited.value) == (abs(velocity) > capped)

    # Reverse during an active pulse; every pulse must retain its direction,
    # and fall-to-direction and direction-to-rise gaps must satisfy the contract.
    await reset(dut)
    command(dut, 655360)
    monitor = Monitor(dut, p)
    while not monitor.previous:
        await monitor.run(1)
    command(dut, -655360)
    await monitor.run(250)
    assert any(direction == 0 for _, direction in monitor.rises)
    command(dut, 98305)
    await monitor.run(600)
    assert monitor.rises[-1][1] == 1

    # Stop during a pulse: finish that pulse and never produce catch-up events.
    while not monitor.previous:
        await monitor.run(1)
    command(dut, 0)
    before = len(monitor.rises)
    await monitor.run(500)
    assert len(monitor.rises) == before and not monitor.previous
    command(dut, 65536)
    before = len(monitor.rises)
    await monitor.run(80)  # At 1kHz the DDS period is 100 clocks.
    assert len(monitor.rises) == before
    await monitor.run(250)
    assert len(monitor.rises) > before

    # Each asynchronous output gate must suppress a pulse before the next edge.
    # These explicit safety aborts are allowed to truncate an active pulse.
    for signal, value, restore in (("enable", 0, 1), ("fault", 1, 0), ("rst_n", 0, 1)):
        await reset(dut)
        command(dut, 655360)
        for _ in range(100):
            if (await cycle(dut))[0]:
                break
        else:
            assert False, "No pulse before abort"
        getattr(dut, signal).value = value
        await Timer(1, unit="ns")
        assert not int(dut.step.value) and not int(dut.driver_enable.value)
        await cycle(dut)
        command(dut, 0)
        getattr(dut, signal).value = restore
        monitor = Monitor(dut, p)
        await monitor.run(100)
        assert not monitor.rises
        assert int(dut.driver_enable.value)
        command(dut, -65536)
        await monitor.run(300)
        assert monitor.rises and monitor.rises[-1][1] == 0


@pytest.mark.parametrize("single_cycle", [False, True])
def test_step_dir(tmp_path, single_cycle):
    from cocotb_tools.runner import get_runner
    root = Path(__file__).resolve().parents[1]
    params = dict(CLOCK_HZ=100000, CONTROL_HZ=1000, MAX_STEP_HZ=10000,
                  STEP_HIGH_CYCLES=1 if single_cycle else 3,
                  STEP_LOW_CYCLES=1 if single_cycle else 4,
                  DIR_SETUP_CYCLES=1 if single_cycle else 5,
                  DIR_HOLD_CYCLES=1 if single_cycle else 6,
                  ENABLE_SETUP_CYCLES=1 if single_cycle else 7)
    runner = get_runner("icarus")
    runner.build(sources=[root / "src" / "step_dir_generator.v"],
                 hdl_toplevel="step_dir_generator", parameters=params,
                 build_dir=tmp_path / "step_build", timescale=("1ns", "1ps"), always=True)
    runner.test(hdl_toplevel="step_dir_generator", test_module="test_step_dir",
                test_dir=Path(__file__).parent, extra_env={"STEP_TEST_PARAMS": json.dumps(params)},
                results_xml=str(tmp_path / "step_results.xml"))
