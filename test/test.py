# SPDX-License-Identifier: Apache-2.0
"""Top-level port and integration checks at 10 MHz clk / 500 kHz SPI."""
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ReadOnly, Timer


_pins = 2


async def initialize(dut):
    global _pins
    _pins = 2
    dut.ena.value = 1
    dut.ui_in.value = 2  # CS idle high
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    cocotb.start_soon(Clock(dut.clk, 100, unit="ns").start())
    await Timer(731, unit="ns")
    dut.rst_n.value = 1
    await Timer(1000, unit="ns")


def set_input(dut, bit, value):
    global _pins
    _pins = (_pins & ~(1 << bit)) | (value << bit)
    dut.ui_in.value = _pins


async def spi_frame(dut, word, count=16):
    set_input(dut, 1, 0)
    await Timer(1000, unit="ns")
    response = 0
    for bit in range(count - 1, -1, -1):
        set_input(dut, 2, (word >> bit) & 1)
        await Timer(1000, unit="ns")
        # A mode-0 master samples at the physical rising SCLK edge.
        set_input(dut, 0, 1)
        response = (response << 1) | ((int(dut.uo_out.value) >> 6) & 1)
        await Timer(1000, unit="ns")
        set_input(dut, 0, 0)
    await Timer(1000, unit="ns")
    set_input(dut, 1, 1)
    await Timer(1000, unit="ns")
    return response


@cocotb.test()
async def ports_status_and_reset(dut):
    await initialize(dut)
    assert int(dut.uo_out.value) == 0
    assert int(dut.uio_oe.value) == 0
    assert int(dut.uio_out.value) == 0
    assert await spi_frame(dut, 0) == 0

    # All motor/status outputs below fault stay inactive, even with enable high
    # and arbitrary commands. The bidirectional pins remain inputs.
    set_input(dut, 5, 1)
    for command in (0x8FFF, 0xA00F, 0xC03F):
        await spi_frame(dut, command)
        assert int(dut.uo_out.value) & 0xBF == 0
        assert int(dut.uio_oe.value) == 0

    # A legal cycle has no fault; a diagonal transition reports a sticky fault.
    for phase in (1, 3, 2, 0):
        set_input(dut, 3, phase >> 1)
        set_input(dut, 4, phase & 1)
        await Timer(1000, unit="ns")
        assert int(dut.uo_out.value) & 0x20 == 0
    set_input(dut, 3, 1)
    set_input(dut, 4, 1)
    await Timer(1000, unit="ns")
    assert int(dut.uo_out.value) == 0x20
    assert await spi_frame(dut, 0) == 0x1000

    dut.ena.value = 0
    await Timer(1000, unit="ns")
    assert int(dut.uo_out.value) == 0
    assert await spi_frame(dut, 0) == 0
    dut.ena.value = 1
    await Timer(1000, unit="ns")
    assert int(dut.uo_out.value) == 0x20

    # Assert reset away from a clock edge; fault disappears immediately.
    await FallingEdge(dut.clk)
    await Timer(7, unit="ns")
    dut.rst_n.value = 0
    await Timer(10, unit="ns")
    await ReadOnly()
    assert int(dut.uo_out.value) == 0
    await Timer(1000, unit="ns")
    dut.rst_n.value = 1
    await Timer(1000, unit="ns")
    assert int(dut.uo_out.value) == 0  # stationary AB=11 startup is valid
    assert await spi_frame(dut, 0) == 0


@cocotb.test(skip=os.environ.get("GATES") == "yes")
async def rtl_register_and_encoder_integration(dut):
    """Check internal connections; excluded from flattened gate-level tests."""
    await initialize(dut)
    spi = dut.user_project.spi
    for command, expected in ((0x9000, (4096, 0, 0)),
                              (0xBFFA, (4096, 10, 0)),
                              (0xDFD5, (4096, 10, 21))):
        await spi_frame(dut, command)
        assert (int(spi.target.value), int(spi.control.value),
                int(spi.plan.value)) == expected
    await spi_frame(dut, 0xFFFF, count=7)
    assert int(spi.target.value) == 4096
    assert int(spi.control.value) == 10
    assert int(spi.plan.value) == 21
    for phase in (1, 3, 2, 0):
        set_input(dut, 3, phase >> 1)
        set_input(dut, 4, phase & 1)
        await Timer(1000, unit="ns")
    assert int(dut.user_project.encoder.position.value) == 4
