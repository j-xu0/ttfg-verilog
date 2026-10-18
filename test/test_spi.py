"""Run with: python -m pytest test/test_spi.py (Icarus Verilog required)."""
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer


async def cycles(dut, count=1):
    for _ in range(count):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")


async def reset(dut):
    dut.cs_n.value = 1
    dut.sclk.value = 0
    dut.mosi.value = 0
    dut.busy.value = 0
    dut.fault.value = 0
    dut.at_target.value = 0
    dut.rst_n.value = 0
    await cycles(dut, 5)
    dut.rst_n.value = 1
    await cycles(dut, 5)


async def begin(dut):
    dut.cs_n.value = 0
    await cycles(dut, 5)


async def bits(dut, word, count=16):
    response = 0
    # Four system-clock periods per half-cycle, independent of clock phase.
    # Change MOSI on falling SCLK; inspect MISO at physical rising SCLK,
    # before the oversampling logic has observed the rising edge.
    for i in range(count):
        dut.mosi.value = (word >> (count - i - 1)) & 1
        await Timer(40, unit="ns")
        dut.sclk.value = 1
        await Timer(1, unit="ns")
        response = (response << 1) | int(dut.miso.value)
        await Timer(39, unit="ns")
        dut.sclk.value = 0
    await Timer(40, unit="ns")
    return response


async def finish(dut, writes=0, rejects=0):
    dut.cs_n.value = 1
    observed_writes = observed_rejects = observed_targets = 0
    for _ in range(8):
        await cycles(dut)
        observed_writes += int(dut.write_pulse.value)
        observed_rejects += int(dut.reject_pulse.value)
        observed_targets += int(dut.target_write_pulse.value)
        assert not int(dut.target_write_pulse.value) or int(dut.write_pulse.value)
    assert (observed_writes, observed_rejects) == (writes, rejects)
    assert int(dut.miso.value) == 0
    return observed_targets


async def frame(dut, word, count=16, writes=0, rejects=0):
    await begin(dut)
    reply = await bits(dut, word, count)
    target_writes = await finish(dut, writes, rejects)
    assert target_writes == int(bool(writes) and ((word >> 13) & 3) == 0)
    return reply


def registers(dut):
    return (int(dut.target.value), int(dut.control.value), int(dut.plan.value))


@cocotb.test()
async def spi_contract(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset(dut)
    assert registers(dut) == (0, 0, 0)
    assert int(dut.miso.value) == 0

    # Exercise signed endpoints and register isolation/masking.
    for payload in (0, 1, 4095, 4096, 8191):
        await begin(dut)
        old = registers(dut)
        await bits(dut, 0x8000 | payload)
        assert registers(dut) == old, "must wait for CS deassertion"
        assert await finish(dut, writes=1) == 1
        assert registers(dut) == (payload, 0, 0)
    await frame(dut, 0xA000 | 0x1FFA, writes=1)
    assert registers(dut) == (8191, 10, 0)
    await frame(dut, 0xC000 | 0x1FD5, writes=1)
    assert registers(dut) == (8191, 10, 21)
    saved = registers(dut)

    # Reserved writes, every truncated length, and long frames cannot write.
    await frame(dut, 0xFFFF, rejects=1)
    for count in range(1, 16):
        await frame(dut, 0xFFFF, count=count, rejects=1)
        assert registers(dut) == saved
    for count in (17, 32, 48, 80):
        await frame(dut, (1 << count) - 1, count=count, rejects=1)
        assert registers(dut) == saved
    await frame(dut, 0, count=0)

    # Read response is a frame-start snapshot, unaffected by later changes.
    dut.at_target.value = 1
    dut.fault.value = 1
    await begin(dut)
    dut.at_target.value = 0
    dut.fault.value = 0
    assert await bits(dut, 0x7FFF) == 0x3000
    await finish(dut)
    assert registers(dut) == saved

    # A busy pulse poisons the frame even if it ends before commit.
    await begin(dut)
    await bits(dut, 0x80, count=8)
    dut.busy.value = 1
    await cycles(dut, 2)
    dut.busy.value = 0
    await bits(dut, 0x12, count=8)
    await finish(dut, rejects=1)
    assert registers(dut) == saved

    # Busy at frame start and busy arriving just before CS completion.
    dut.busy.value = 1
    assert await frame(dut, 0x8001, rejects=1) == 0x4000
    dut.busy.value = 0
    await begin(dut)
    await bits(dut, 0x8002)
    dut.busy.value = 1
    await finish(dut, rejects=1)
    dut.busy.value = 0
    assert registers(dut) == saved
    await frame(dut, 0x8123, writes=1)
    assert int(dut.target.value) == 0x123

    # Minimum half-period transactions at varied phase offsets, including
    # status MSBs which would expose shifting MISO on the wrong clock edge.
    for offset in (1, 3, 6, 9):
        await Timer(offset, unit="ns")
        dut.at_target.value = 1
        dut.fault.value = 1
        await begin(dut)
        await Timer(offset, unit="ns")
        assert await bits(dut, 0x8000 | offset) == 0x3000
        await finish(dut, writes=1)
        assert int(dut.target.value) == offset

    # Reset while CS stays low must not manufacture a new frame. The host
    # must deassert CS and assert it anew before a write can be accepted.
    dut.cs_n.value = 0
    dut.rst_n.value = 0
    await cycles(dut, 5)
    dut.rst_n.value = 1
    await cycles(dut, 5)
    await bits(dut, 0x8123)
    await finish(dut)
    assert registers(dut) == (0, 0, 0)
    await frame(dut, 0x8009, writes=1)
    assert registers(dut) == (9, 0, 0)

    # Reset aborts an in-flight frame and clears all persistent registers.
    await begin(dut)
    await bits(dut, 0xFF, count=8)
    await reset(dut)
    assert registers(dut) == (0, 0, 0)
    assert int(dut.write_pulse.value) == 0
    assert int(dut.reject_pulse.value) == 0
    await frame(dut, 0x8007, writes=1)
    assert registers(dut) == (7, 0, 0)


def test_spi(tmp_path):
    from cocotb_tools.runner import get_runner

    root = Path(__file__).resolve().parents[1]
    runner = get_runner("icarus")
    runner.build(
        sources=[root / "src" / "stepper_spi.v"],
        hdl_toplevel="stepper_spi",
        build_dir=tmp_path / "spi",
        always=True,
        timescale=("1ns", "1ps"),
    )
    runner.test(hdl_toplevel="stepper_spi", test_module="test_spi")
