"""Independent quadrature contract tests: python -m pytest test/test_encoder.py."""
from pathlib import Path

import cocotb
from cocotb.triggers import Timer


async def tick(dut, cycles=1):
    for _ in range(cycles):
        dut.clk.value = 0
        await Timer(5, unit="ns")
        dut.clk.value = 1
        await Timer(5, unit="ns")


def position(dut):
    raw = int(dut.position.value)
    return raw if raw < 4096 else raw - 8192


async def set_phase(dut, phase):
    dut.a.value = phase >> 1
    dut.b.value = phase & 1
    await tick(dut, 6)


async def reset(dut, phase=0):
    dut.rst_n.value = 0
    dut.a.value = phase >> 1
    dut.b.value = phase & 1
    await tick(dut, 2)
    dut.rst_n.value = 1
    await tick(dut, 8)
    assert position(dut) == 0
    assert int(dut.fault.value) == 0


@cocotb.test()
async def quadrature_contract(dut):
    # Every stationary starting phase, including 11, must initialize to zero.
    for initial in range(4):
        await reset(dut, initial)
        await tick(dut, 12)
        assert position(dut) == 0
        assert int(dut.fault.value) == 0

    await reset(dut)
    positive = (1, 3, 2, 0)
    negative = (2, 3, 1, 0)
    expected = 0
    for _ in range(3):
        for phase in positive:
            await set_phase(dut, phase)
            expected += 1
            assert position(dut) == expected
            assert int(dut.fault.value) == 0
    for _ in range(4):
        for phase in negative:
            await set_phase(dut, phase)
            expected -= 1
            assert position(dut) == expected
    await tick(dut, 20)
    assert position(dut) == expected

    # Both-bit jumps in either diagonal are rejected, including after recovery.
    for before, after in ((0, 3), (3, 0), (1, 2), (2, 1)):
        await reset(dut, before)
        await set_phase(dut, after)
        assert position(dut) == 0
        assert int(dut.fault.value) == 1
        next_phase = {0: 1, 1: 3, 3: 2, 2: 0}[after]
        await set_phase(dut, next_phase)
        assert position(dut) == 1
        assert int(dut.fault.value) == 1
        await tick(dut, 10)
        assert position(dut) == 1
        assert int(dut.fault.value) == 1

    # Check each edge across the signed boundary, then reverse across it.
    await reset(dut)
    for edge in range(4096):
        await set_phase(dut, positive[edge % 4])
        expected = ((edge + 1 + 4096) % 8192) - 4096
        assert position(dut) == expected
    assert position(dut) == -4096
    await set_phase(dut, 2)
    assert position(dut) == 4095
    assert int(dut.fault.value) == 0
    await reset(dut, 3)


def test_encoder(tmp_path):
    from cocotb_tools.runner import get_runner

    root = Path(__file__).resolve().parents[1]
    runner = get_runner("icarus")
    runner.build(
        sources=[root / "src" / "quadrature_decoder.v"],
        hdl_toplevel="quadrature_decoder",
        build_dir=tmp_path / "encoder_build",
        timescale=("1ns", "1ps"),
        always=True,
    )
    runner.test(
        hdl_toplevel="quadrature_decoder",
        test_module="test_encoder",
        test_dir=Path(__file__).parent,
        results_xml=str(tmp_path / "encoder_results.xml"),
    )
