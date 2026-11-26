"""Clock-enable timing contract: python -m pytest test/test_timebase.py."""
import os
from pathlib import Path

import cocotb
from cocotb.triggers import Timer
import pytest


async def tick(dut):
    dut.clk.value = 0
    await Timer(50, unit="ns")
    dut.clk.value = 1
    await Timer(50, unit="ns")
    return int(dut.control_ce.value)


@cocotb.test()
async def timebase_contract(dut):
    divisor = int(os.environ["TIMEBASE_DIVISOR"])
    dut.clk.value = 0
    dut.enable.value = 1
    dut.rst_n.value = 0
    assert await tick(dut) == 0
    dut.rst_n.value = 1

    # Check every clock across three full periods, including the actual
    # 10000-cycle default. Divisor 1 intentionally enables every clock.
    for cycle in range(1, 3 * divisor + 2):
        assert await tick(dut) == int(cycle % divisor == 0), cycle

    # Reset must discard a partial interval and produce a full first period.
    dut.rst_n.value = 0
    assert await tick(dut) == 0
    dut.rst_n.value = 1
    partial = divisor // 2
    for _ in range(partial):
        assert await tick(dut) == 0

    # Disable freezes phase, suppresses CE, and emits no delayed stale pulse.
    dut.enable.value = 0
    for _ in range(5):
        assert await tick(dut) == 0
    dut.enable.value = 1
    for cycle in range(1, divisor - partial + 1):
        assert await tick(dut) == int(cycle == divisor - partial)

    # Disable immediately after a pulse must clear it on the next edge.
    dut.enable.value = 0
    for _ in range(3):
        assert await tick(dut) == 0
    dut.rst_n.value = 0
    assert await tick(dut) == 0
    dut.rst_n.value = 1
    for _ in range(3):
        assert await tick(dut) == 0
    dut.enable.value = 1
    for cycle in range(1, divisor + 1):
        assert await tick(dut) == int(cycle == divisor)

    # Active-low reset clears even while the clock is stationary.
    dut.rst_n.value = 0
    await Timer(1, unit="ns")
    assert int(dut.control_ce.value) == 0


@pytest.mark.parametrize("clock_hz,control_hz", [(10000000, 1000), (7, 1), (1, 1)])
def test_timebase(tmp_path, clock_hz, control_hz):
    from cocotb_tools.runner import get_runner

    root = Path(__file__).resolve().parents[1]
    runner = get_runner("icarus")
    runner.build(
        sources=[root / "src" / "control_timebase.v"],
        hdl_toplevel="control_timebase",
        parameters={"CLOCK_HZ": clock_hz, "CONTROL_HZ": control_hz},
        build_dir=tmp_path / "timebase_build",
        timescale=("1ns", "1ps"),
        always=True,
    )
    runner.test(
        hdl_toplevel="control_timebase",
        test_module="test_timebase",
        test_dir=Path(__file__).parent,
        extra_env={"TIMEBASE_DIVISOR": str(clock_hz // control_hz)},
        results_xml=str(tmp_path / "timebase_results.xml"),
    )
