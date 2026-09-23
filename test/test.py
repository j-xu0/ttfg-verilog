# SPDX-FileCopyrightText: © 2026 Jason Xu
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer


async def rising_edge(dut):
    """Wait for a rising edge and allow nonblocking assignments to settle."""
    await RisingEdge(dut.clk)
    await Timer(1, unit="ns")


@cocotb.test()
async def test_counter(dut):
    """Exercise asynchronous reset, load, count, wrap, and output enable."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 1

    # Assert reset between clock edges and check it before the next rising edge.
    await FallingEdge(dut.clk)
    dut.rst_n.value = 0
    await Timer(1, unit="ns")
    assert dut.uio_out.value == 0
    assert dut.uio_oe.value == 0x00
    assert dut.uo_out.value == 0x00

    dut.rst_n.value = 1

    # A synchronous load must not change the count until a rising clock edge.
    dut.uio_in.value = 0x42
    dut.ui_in.value = 0x01  # load=1, output_enable=0
    await Timer(1, unit="us")
    assert dut.uio_out.value == 0x00
    assert dut.uio_oe.value == 0x00
    await rising_edge(dut)
    assert dut.uio_out.value == 0x42

    # With load deasserted, the value increments once per rising edge.
    dut.ui_in.value = 0x02
    await Timer(1, unit="ns")
    assert dut.uio_oe.value == 0xFF
    await rising_edge(dut)
    assert dut.uio_out.value == 0x43
    await rising_edge(dut)
    assert dut.uio_out.value == 0x44

    # Loading 0xff and counting once verifies modulo-256 wraparound.
    dut.uio_in.value = 0xFF
    dut.ui_in.value = 0x01
    await rising_edge(dut)
    assert dut.uio_out.value == 0xFF
    dut.ui_in.value = 0x02
    await Timer(1, unit="ns")
    await rising_edge(dut)
    assert dut.uio_out.value == 0x00

    # Releasing the pins does not stop the internal counter.
    dut.ui_in.value = 0x00
    await Timer(1, unit="ns")
    assert dut.uio_oe.value == 0x00
    await rising_edge(dut)
    assert dut.uio_out.value == 0x01

    # Asynchronous reset must also clear a nonzero running count.
    await FallingEdge(dut.clk)
    dut.rst_n.value = 0
    await Timer(1, unit="ns")
    assert dut.uio_out.value == 0x00
