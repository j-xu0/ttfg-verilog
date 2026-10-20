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
                test_filter="pulse_contract",
                test_dir=Path(__file__).parent, extra_env={"STEP_TEST_PARAMS": json.dumps(params)},
                results_xml=str(tmp_path / "step_results.xml"))


class UnreducedNCO:
    """Original full-unit phase algorithm, independent of RTL's GCD reduction."""
    def __init__(self, params):
        self.p = params
        self.reset()

    def reset(self):
        self.pulse = self.direction = self.phase = self.high_left = 0
        self.low_age = max(self.p["STEP_LOW_CYCLES"], self.p["DIR_HOLD_CYCLES"]) - 1
        self.dir_age = self.enable_age = self.timing_limited = 0

    def outputs(self, velocity, rst_n, enable, fault):
        active = bool(rst_n and enable and not fault)
        clamp = abs(velocity) > 65536 * self.p["MAX_STEP_HZ"] // self.p["CONTROL_HZ"]
        return (int(active and self.pulse), self.direction, int(active),
                int(active and (clamp or self.timing_limited)))

    def edge(self, velocity, rst_n, enable, fault):
        if not rst_n or not enable or fault:
            self.reset()
            return
        p = self.p
        old_pulse, old_low, old_dir, old_enable = (
            self.pulse, self.low_age, self.dir_age, self.enable_age)
        self.enable_age = min(old_enable + 1, p["ENABLE_SETUP_CYCLES"] - 1)
        self.dir_age = min(old_dir + 1, p["DIR_SETUP_CYCLES"] - 1)
        if not old_pulse:
            self.low_age = min(old_low + 1, max(p["STEP_LOW_CYCLES"], p["DIR_HOLD_CYCLES"]) - 1)
        self.timing_limited = 0
        if old_pulse:
            if self.high_left:
                self.high_left -= 1
            else:
                self.pulse = 0
                self.low_age = 0
        if not velocity:
            self.phase = 0
        elif self.direction != (velocity > 0):
            self.phase = 0
            if not old_pulse and old_low >= p["DIR_HOLD_CYCLES"] - 1:
                self.direction = int(velocity > 0)
                self.dir_age = 0
        elif old_dir < p["DIR_SETUP_CYCLES"] - 1 or old_enable < p["ENABLE_SETUP_CYCLES"] - 1:
            self.phase = 0
        else:
            magnitude = min(abs(velocity), 65536 * p["MAX_STEP_HZ"] // p["CONTROL_HZ"])
            # This intentionally retains the original unreduced multiplier and
            # threshold, never reading the optimized RTL's arithmetic constants.
            self.phase += magnitude * p["CONTROL_HZ"]
            threshold = 65536 * p["CLOCK_HZ"]
            if self.phase >= threshold:
                self.phase -= threshold
                if not old_pulse and old_low >= p["STEP_LOW_CYCLES"] - 1:
                    self.pulse = 1
                    self.high_left = p["STEP_HIGH_CYCLES"] - 1
                else:
                    self.timing_limited = 1


@cocotb.test()
async def nco_old_algorithm_equivalence(dut):
    import math
    import random
    p = json.loads(os.environ["STEP_TEST_PARAMS"])
    model = UnreducedNCO(p)
    rng = random.Random(0x180DD5)
    factor = math.gcd(65536 * p["CLOCK_HZ"], p["CONTROL_HZ"])
    threshold = 65536 * p["CLOCK_HZ"] // factor
    assert len(dut.phase) == max(1, (threshold - 1).bit_length())
    dut.clk.value = 0
    dut.rst_n.value = 0
    dut.enable.value = 0
    dut.fault.value = 0
    command(dut, 0)
    await cycle(dut)

    def outputs():
        return tuple(int(getattr(dut, name).value) for name in
                     ("step", "dir", "driver_enable", "rate_limited"))

    vectors = [(0, 0, 0, 0, 2)]
    # Stable fractional phases, in-flight rate changes, reversal and zero.
    for velocity in (98305, 700001, 1, 655359, 0, -98305, -(1 << 20), 0):
        vectors.append((velocity, 1, 1, 0, 4500))
    for _ in range(100):
        velocity = rng.choice((0, 1, -1, 65536, -65536, 98305,
                               -(1 << 20), (1 << 20)-1,
                               rng.randint(-(1 << 20), (1 << 20)-1)))
        rst_n = int(rng.randrange(25) != 0)
        enable = int(rng.randrange(9) != 0)
        fault = int(rng.randrange(11) == 0)
        vectors.append((velocity, rst_n, enable, fault, rng.randint(1, 1600)))

    for velocity, rst_n, enable, fault, count in vectors:
        command(dut, velocity)
        dut.rst_n.value = rst_n
        dut.enable.value = enable
        dut.fault.value = fault
        if not rst_n:
            model.reset()
        await Timer(1, unit="ns")
        assert outputs() == model.outputs(velocity, rst_n, enable, fault), "asynchronous gate mismatch"
        for _ in range(count):
            model.edge(velocity, rst_n, enable, fault)
            await cycle(dut)
            assert outputs() == model.outputs(velocity, rst_n, enable, fault)
            assert int(dut.phase.value) * factor == model.phase


    # Force each asynchronous abort while STEP is actually high. This also
    # exercises reset between edges and checks the original phase on restart.
    for signal in ("enable", "fault", "rst_n"):
        velocity, rst_n, enable, fault = 655360, 1, 1, 0
        command(dut, velocity)
        dut.rst_n.value = 1
        dut.enable.value = 1
        dut.fault.value = 0
        for _ in range(10000):
            model.edge(velocity, rst_n, enable, fault)
            await cycle(dut)
            assert outputs() == model.outputs(velocity, rst_n, enable, fault)
            assert int(dut.phase.value) * factor == model.phase
            if int(dut.step.value):
                break
        else:
            assert False, "failed to reach active pulse for safety abort"
        if signal == "enable":
            enable = 0
        elif signal == "fault":
            fault = 1
        else:
            rst_n = 0
            model.reset()
        getattr(dut, signal).value = {"enable": enable, "fault": fault, "rst_n": rst_n}[signal]
        await Timer(1, unit="ns")
        assert outputs() == model.outputs(velocity, rst_n, enable, fault)
        model.edge(velocity, rst_n, enable, fault)
        await cycle(dut)
        assert outputs() == model.outputs(velocity, rst_n, enable, fault)
        assert int(dut.phase.value) * factor == model.phase


@pytest.mark.parametrize("configuration", ["production", "coprime", "one_cycle", "blocked", "unit_threshold", "small_split"])
def test_step_dir_exact_nco(tmp_path, configuration):
    from cocotb_tools.runner import get_runner
    root = Path(__file__).resolve().parents[1]
    params = dict(CLOCK_HZ=10000000, CONTROL_HZ=1000, MAX_STEP_HZ=10000,
                  STEP_HIGH_CYCLES=50, STEP_LOW_CYCLES=50,
                  DIR_SETUP_CYCLES=50, DIR_HOLD_CYCLES=50, ENABLE_SETUP_CYCLES=100)
    if configuration == "coprime":
        params.update(CLOCK_HZ=100003, CONTROL_HZ=997,
                      STEP_HIGH_CYCLES=3, STEP_LOW_CYCLES=4,
                      DIR_SETUP_CYCLES=5, DIR_HOLD_CYCLES=6, ENABLE_SETUP_CYCLES=7)
    elif configuration in ("one_cycle", "blocked"):
        params.update(CLOCK_HZ=100000, STEP_HIGH_CYCLES=1, STEP_LOW_CYCLES=1,
                      DIR_SETUP_CYCLES=1, DIR_HOLD_CYCLES=1, ENABLE_SETUP_CYCLES=1)
        if configuration == "blocked":
            params.update(STEP_HIGH_CYCLES=13, STEP_LOW_CYCLES=11)
    elif configuration == "small_split":
        # Reduced threshold 10 = 5*2 exercises the split path with a one-bit
        # fractional word, maximum increment equal to its modulus, and an
        # upper modulus different from production's 625.
        params.update(CLOCK_HZ=10, CONTROL_HZ=65536, MAX_STEP_HZ=2,
                      STEP_HIGH_CYCLES=1, STEP_LOW_CYCLES=1,
                      DIR_SETUP_CYCLES=1, DIR_HOLD_CYCLES=1, ENABLE_SETUP_CYCLES=1)
    elif configuration == "unit_threshold":
        params.update(CLOCK_HZ=1, CONTROL_HZ=65536, MAX_STEP_HZ=1,
                      STEP_HIGH_CYCLES=1, STEP_LOW_CYCLES=1,
                      DIR_SETUP_CYCLES=1, DIR_HOLD_CYCLES=1, ENABLE_SETUP_CYCLES=1)
    runner = get_runner("icarus")
    runner.build(sources=[root / "src" / "step_dir_generator.v"],
                 hdl_toplevel="step_dir_generator", parameters=params,
                 build_dir=tmp_path / "nco_build", timescale=("1ns", "1ps"), always=True)
    runner.test(hdl_toplevel="step_dir_generator", test_module="test_step_dir",
                test_filter="nco_old_algorithm_equivalence",
                test_dir=Path(__file__).parent, extra_env={"STEP_TEST_PARAMS": json.dumps(params)},
                results_xml=str(tmp_path / "nco_results.xml"))
