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
    dut.at_target.value = 0
    dut.fault_code.value = 0
    dut.fault_event.value = 0
    dut.motion_event.value = 0
    dut.motion_disabled.value = 1
    dut.rst_n.value = 0
    await cycles(dut, 5)
    dut.rst_n.value = 1
    await cycles(dut, 5)


async def begin(dut):
    dut.cs_n.value = 0
    await cycles(dut, 5)


async def one_bit(dut, value):
    dut.mosi.value = value
    await Timer(40, unit="ns")
    dut.sclk.value = 1
    await Timer(40, unit="ns")
    dut.sclk.value = 0
    await Timer(40, unit="ns")


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


async def finish(dut, writes=0):
    dut.cs_n.value = 1
    observed_writes = observed_targets = 0
    for _ in range(8):
        await cycles(dut)
        observed_writes += int(dut.write_pulse.value)
        observed_targets += int(dut.target_write_pulse.value)
        assert not int(dut.target_write_pulse.value) or int(dut.write_pulse.value)
    assert observed_writes == writes
    assert int(dut.miso.value) == 0
    return observed_targets


async def frame(dut, word, count=16, writes=0):
    await begin(dut)
    reply = await bits(dut, word, count)
    target_writes = await finish(dut, writes)
    assert target_writes == int(bool(writes) and ((word >> 13) & 3) == 0)
    return reply


def registers(dut):
    return (int(dut.target.value), int(dut.plan.value))


@cocotb.test()
async def spi_contract(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset(dut)
    assert registers(dut) == (0, 0)
    assert int(dut.miso.value) == 0
    assert int(dut.int_pending.value) == 0
    assert int(dut.frame_holds_motion.value) == 0

    # Exercise signed endpoints and register isolation/masking.
    for payload in (0, 1, 4095, 4096, 8191):
        await begin(dut)
        old = registers(dut)
        await bits(dut, 0x8000 | payload)
        assert registers(dut) == old, "must wait for CS deassertion"
        assert await finish(dut, writes=1) == 1
        assert registers(dut) == (payload, 0)
    # Addr 1 is reserved after CONTROL removal: silently ignored.
    await frame(dut, 0xA000 | 0x1FFA)
    assert registers(dut) == (8191, 0)
    await frame(dut, 0xC000 | 0x1FD5, writes=1)
    assert registers(dut) == (8191, 5)
    saved = registers(dut)

    # Reserved writes, every truncated length, and long frames cannot write.
    await frame(dut, 0xFFFF)
    await frame(dut, 0xBFFF)
    for count in range(1, 16):
        await frame(dut, 0xFFFF, count=count)
        assert registers(dut) == saved
    for count in (17, 32, 48, 80):
        await frame(dut, (1 << count) - 1, count=count)
        assert registers(dut) == saved
    await frame(dut, 0, count=0)

    # Read response is a frame-start snapshot, unaffected by later changes.
    dut.at_target.value = 1
    dut.fault_code.value = 3
    await begin(dut)
    dut.at_target.value = 0
    dut.fault_code.value = 0
    assert await bits(dut, 0x7FFF) == 0x2003
    await finish(dut)
    assert registers(dut) == saved

    # BUSY at frame start is the verdict: the write is dropped, and the host
    # already has that same bit. Bit 12 stays zero now that fault is |codes.
    dut.busy.value = 1
    assert await frame(dut, 0x8001) == 0x4000
    assert registers(dut) == saved
    dut.busy.value = 0
    # Busy arriving mid-frame no longer drops the write: the top-level
    # frame_holds_motion gate prevents motion from starting inside a frame,
    # so this can only be a testbench artefact at this level.
    await begin(dut)
    await bits(dut, 0x80, count=8)
    dut.busy.value = 1
    await cycles(dut, 2)
    dut.busy.value = 0
    await bits(dut, 0x12, count=8)
    await finish(dut, writes=1)
    assert registers(dut) == (0x12, 5)
    saved = registers(dut)

    # frame_holds_motion treats a frame as a write until its first bit.
    await begin(dut)
    assert int(dut.frame_holds_motion.value) == 1
    await one_bit(dut, 0)
    assert int(dut.frame_holds_motion.value) == 0
    for _ in range(15):
        await one_bit(dut, 0)
    await finish(dut)
    assert int(dut.frame_holds_motion.value) == 0
    await begin(dut)
    await one_bit(dut, 1)
    assert int(dut.frame_holds_motion.value) == 1
    for _ in range(15):
        await one_bit(dut, 1)
    await finish(dut)
    assert registers(dut) == saved
    assert int(dut.frame_holds_motion.value) == 0

    # Minimum half-period transactions at varied phase offsets, including
    # status MSBs which would expose shifting MISO on the wrong clock edge.
    for offset in (1, 3, 6, 9):
        await Timer(offset, unit="ns")
        dut.at_target.value = 1
        dut.fault_code.value = 1
        await begin(dut)
        await Timer(offset, unit="ns")
        assert await bits(dut, 0x8000 | offset) == 0x2001
        await finish(dut, writes=1)
        assert int(dut.target.value) == offset
    dut.at_target.value = 0
    dut.fault_code.value = 0

    # Reset while CS stays low must not manufacture a new frame. The host
    # must deassert CS and assert it anew before a write can be accepted.
    dut.cs_n.value = 0
    dut.rst_n.value = 0
    await cycles(dut, 5)
    dut.rst_n.value = 1
    await cycles(dut, 5)
    await bits(dut, 0x8123)
    await finish(dut)
    assert registers(dut) == (0, 0)
    await frame(dut, 0x8009, writes=1)
    assert registers(dut) == (9, 0)

    # Reset aborts an in-flight frame and clears all persistent registers.
    await begin(dut)
    await bits(dut, 0xFF, count=8)
    await reset(dut)
    assert registers(dut) == (0, 0)
    assert int(dut.write_pulse.value) == 0
    assert int(dut.int_pending.value) == 0
    await frame(dut, 0x8007, writes=1)
    assert registers(dut) == (7, 0)


@cocotb.test()
async def recovery_commands(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset(dut)

    async def transaction(word, count=16, event=None):
        await begin(dut)
        reply = await bits(dut, word, count)
        if event is not None:
            event.value = 1
            await cycles(dut)
            event.value = 0
        dut.cs_n.value = 1
        clears = zeros = 0
        for _ in range(8):
            await cycles(dut)
            clears += int(dut.fault_clear_pulse.value)
            zeros += int(dut.zero_position_pulse.value)
        return reply, clears, zeros

    dut.fault_code.value = 3
    assert await transaction(0xC002) == (0x0003, 0, 0)
    assert int(dut.int_pending.value) == 0
    # Truncated reads cannot acknowledge (a 15-bit reply drops the low bit).
    assert await transaction(0, count=15) == ((0x0003 >> 1), 0, 0)
    # An event during the read frame makes it stale: acknowledge nothing and
    # keep the interrupt asserted so the host polls again.
    assert await transaction(0, event=dut.fault_event) == (0x0003, 0, 0)
    assert int(dut.int_pending.value) == 1
    assert await transaction(0) == (0x0003, 1, 0)
    assert int(dut.int_pending.value) == 0

    # A completed move is also an interrupt event.
    assert await transaction(0, event=dut.motion_event) == (0x0003, 0, 0)
    assert int(dut.int_pending.value) == 1
    assert await transaction(0) == (0x0003, 1, 0)
    assert int(dut.int_pending.value) == 0

    dut.busy.value = 1
    assert await transaction(0) == (0x4003, 1, 0)
    # A frame that read BUSY drops its write without side effects.
    assert await transaction(0xE000) == (0x4003, 0, 0)
    dut.busy.value = 0
    dut.motion_disabled.value = 0
    assert await transaction(0xE000) == (0x0003, 0, 0)
    dut.motion_disabled.value = 1
    assert await transaction(0xE001) == (0x0003, 0, 0)
    assert await transaction(0xE000) == (0x0003, 0, 1)
    assert int(dut.target.value) == 0

    # Disable must be held from the sampled CS assertion through commit, not
    # just present at commit time: asserting it mid-frame is too late, and
    # releasing it mid-frame is not undone by re-asserting before CS rises.
    async def finish_zeroing():
        dut.cs_n.value = 1
        clears = zeros = 0
        for _ in range(8):
            await cycles(dut)
            clears += int(dut.fault_clear_pulse.value)
            zeros += int(dut.zero_position_pulse.value)
        return clears, zeros

    # A nonzero TARGET makes a spurious zero visible.
    assert await transaction(0x8000 | 17) == (0x0003, 0, 0)
    assert int(dut.target.value) == 17

    dut.motion_disabled.value = 0
    await begin(dut)
    await bits(dut, 0xE0, count=8)
    dut.motion_disabled.value = 1
    await cycles(dut)
    await bits(dut, 0x00, count=8)
    assert await finish_zeroing() == (0, 0)

    await begin(dut)
    await bits(dut, 0xE0, count=8)
    dut.motion_disabled.value = 0
    await cycles(dut)
    dut.motion_disabled.value = 1
    await bits(dut, 0x00, count=8)
    assert await finish_zeroing() == (0, 0)
    assert registers(dut) == (17, 2)


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
