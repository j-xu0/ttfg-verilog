# SPDX-License-Identifier: Apache-2.0
"""Top-level port and integration checks at 10 MHz clk / 500 kHz SPI."""
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import Edge, FallingEdge, ReadOnly, RisingEdge, Timer
from cocotb.utils import get_sim_time


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

    # Configuration while disabled cannot drive motor pins. Bidirectional pins
    # remain inputs; TARGET is queued until motion enable is asserted.
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


class MotorModel:
    """One legal x4 encoder count per STEP rising edge, with optional lost steps."""
    def __init__(self, dut):
        self.dut = dut
        self.position = 0
        self.phase = 0
        self.drop_next = False
        self.steps = 0
        self.rises = []
        self.widths = []
        self.rise = None
        self.previous = int(dut.uo_out.value)

    def advance(self, delta):
        self.position += delta
        self.phase = (self.phase + delta) % 4
        phase = (0, 1, 3, 2)[self.phase]
        set_input(self.dut, 3, phase >> 1)
        set_input(self.dut, 4, phase & 1)

    async def run(self):
        while True:
            await Edge(self.dut.uo_out)
            await Timer(1, unit="ns")
            pins = int(self.dut.uo_out.value)
            now = get_sim_time(unit="ns")
            if pins & 1 and not self.previous & 1:
                assert pins & 4, "STEP without ENABLE"
                self.rise = now
                self.steps += 1
                self.rises.append(now)
                if self.drop_next:
                    self.drop_next = False
                else:
                    # Encoder response is asynchronous to clk and arrives well
                    # before the next possible pulse at the 10 kHz rate cap.
                    direction = 1 if pins & 2 else -1
                    await Timer(500, unit="ns")
                    self.advance(direction)
            elif self.previous & 1 and not pins & 1:
                self.widths.append(now - self.rise)
            self.previous = pins


async def wait_settled(dut, timeout_us=150000):
    for _ in range(timeout_us // 100):
        pins = int(dut.uo_out.value)
        assert not pins & 0x20, "unexpected feedback fault"
        if pins & 0x10:
            assert not pins & 0x09, "settled with busy/STEP asserted"
            return
        await Timer(100, unit="us")
    raise AssertionError("move did not settle within motor-model timeout")


@cocotb.test()
async def closed_loop_moves_and_disturbance(dut):
    await initialize(dut)
    motor = MotorModel(dut)
    cocotb.start_soon(motor.run())
    await spi_frame(dut, 0xA000)  # Kp=Ki=1/8
    await spi_frame(dut, 0xC03F)  # 10000 steps/s, ~100000 steps/s^2
    await spi_frame(dut, 0x8004)  # configure target while disabled
    await Timer(100, unit="us")
    assert int(dut.uo_out.value) & 0x1F == 0
    motor.drop_next = True
    set_input(dut, 5, 1)
    await Timer(1, unit="us")
    assert int(dut.uo_out.value) & 8
    assert await spi_frame(dut, 0) & 0x4000
    await spi_frame(dut, 0x8007)  # rejected while busy, must still land at 4
    await wait_settled(dut)
    assert motor.position == 4
    assert motor.steps >= 5, "missed step was not corrected"
    assert await spi_frame(dut, 0) == 0x2000

    # Regression: accepting a new TARGET after settle must launch a new plan,
    # rather than interpreting the new target as drift from the old reference.
    await spi_frame(dut, 0x8000 | ((-3) & 0x1FFF))
    await Timer(1, unit="us")
    assert int(dut.uo_out.value) & 8
    await wait_settled(dut)
    assert motor.position == -3
    steps_before = motor.steps
    motor.advance(1)  # external shaft disturbance after settling
    await Timer(1, unit="us")
    assert not int(dut.uo_out.value) & 0x10
    await wait_settled(dut)
    assert motor.position == -3
    assert motor.steps > steps_before

    # Same-value TARGET commits are commands, too (not change detection).
    await spi_frame(dut, 0x8000 | ((-3) & 0x1FFF))
    await wait_settled(dut)
    assert motor.position == -3
    assert motor.widths and min(motor.widths) >= 5000, motor.widths
    assert int(dut.uio_oe.value) == 0
    assert int(dut.uio_out.value) == 0


@cocotb.test()
async def motion_abort_and_fault(dut):
    await initialize(dut)
    await spi_frame(dut, 0xC03F)
    await spi_frame(dut, 0x800A)
    set_input(dut, 5, 1)
    await Timer(4, unit="ms")
    assert int(dut.uo_out.value) & 8
    # A disable does not resume the aborted command on the next enable.
    set_input(dut, 5, 0)
    await Timer(10, unit="ns")
    assert int(dut.uo_out.value) & 7 == 0
    await Timer(1, unit="us")
    set_input(dut, 5, 1)
    await Timer(2, unit="ms")
    assert int(dut.uo_out.value) & 0x1F == 0
    await spi_frame(dut, 0x800A)  # explicit rewrite required
    await Timer(2, unit="us")
    assert int(dut.uo_out.value) & 8
    # Illegal two-bit encoder transition latches a fault and stops outputs.
    set_input(dut, 3, 1)
    set_input(dut, 4, 1)
    await Timer(1, unit="us")
    assert int(dut.uo_out.value) & 0x27 == 0x20
    assert await spi_frame(dut, 0) == 0x1000
    await spi_frame(dut, 0x8001)
    await Timer(2, unit="ms")
    assert int(dut.uo_out.value) & 0x3F == 0x20
    dut.rst_n.value = 0
    await Timer(1, unit="us")
    assert int(dut.uo_out.value) == 0
    dut.rst_n.value = 1
    await Timer(2, unit="ms")
    assert int(dut.uo_out.value) == 0


@cocotb.test()
async def absolute_encoder_range_fault(dut):
    await initialize(dut)
    # Deliberately cross +4095 -> -4096 using legal transitions while disabled.
    # Port-level test also runs against a flattened gate-level netlist.
    for index in range(4096):
        phase = (1, 3, 2, 0)[index % 4]
        set_input(dut, 3, phase >> 1)
        set_input(dut, 4, phase & 1)
        await Timer(1, unit="us")
    assert int(dut.uo_out.value) == 0x20
    assert await spi_frame(dut, 0) == 0x1000

    # Negative endpoint itself is valid; one further decrement must fault.
    dut.rst_n.value = 0
    await Timer(1, unit="us")
    dut.rst_n.value = 1
    await Timer(1, unit="us")
    for index in range(4096):
        phase = (2, 3, 1, 0)[index % 4]
        set_input(dut, 3, phase >> 1)
        set_input(dut, 4, phase & 1)
        await Timer(1, unit="us")
    assert int(dut.uo_out.value) == 0
    set_input(dut, 3, 1)  # 00 -> 10 decrements once more and wraps
    await Timer(1, unit="us")
    assert int(dut.uo_out.value) == 0x20
    set_input(dut, 5, 1)
    await spi_frame(dut, 0x8001)
    await Timer(1, unit="ms")
    assert int(dut.uo_out.value) == 0x20


@cocotb.test()
async def full_speed_and_default_limits(dut):
    await initialize(dut)
    motor = MotorModel(dut)
    cocotb.start_soon(motor.run())
    await spi_frame(dut, 0xC03F)
    await spi_frame(dut, 0x8000 | 1200)
    set_input(dut, 5, 1)
    await wait_settled(dut, timeout_us=400000)
    assert motor.position == 1200
    gaps = [b-a for a, b in zip(motor.rises, motor.rises[1:])]
    assert min(gaps) >= 100000, "exceeded 10 kHz cap"
    assert gaps.count(100000) > 5, "did not exercise the 10 kHz ceiling"
    # Lowest velocity/acceleration codes exercise the Q16 low-acceleration path
    # through SPI, planning, PI and actual encoder feedback.
    await spi_frame(dut, 0xC000)
    await spi_frame(dut, 0x8000 | 1201)
    await wait_settled(dut, timeout_us=300000)
    assert motor.position == 1201
    assert motor.widths and min(motor.widths) >= 5000


@cocotb.test(skip=os.environ.get("GATES") == "yes")
async def serial_reference_sampling(dut):
    """PI snapshots a completed reference before planner workspace is reused."""
    await initialize(dut)
    await spi_frame(dut, 0xC03F)
    await spi_frame(dut, 0x8004)
    set_input(dut, 5, 1)
    top = dut.user_project

    def signed(signal):
        raw = int(signal.value)
        return raw - (1 << len(signal)) if raw >> (len(signal)-1) else raw

    integral = 0
    for _ in range(5):
        # The first interval also includes enable synchronization and phase.
        for _ in range(10020):
            await FallingEdge(dut.clk)
            if int(top.control_ce.value):
                break
        else:
            raise AssertionError("control tick missing")
        assert not int(top.planner_calculating.value)
        assert not int(top.pi_calculating.value)
        reference = signed(top.reference_position)
        feedback = signed(top.position)
        feedforward = signed(top.reference_velocity)
        error = reference - feedback*65536
        total = feedforward + error//8 + integral
        command = max(-655360, min(655360, total))
        saturated = int(command != total)
        if not ((total > 655360 and error > 0) or
                (total < -655360 and error < 0)):
            integral = max(-655360, min(655360, integral + error//8))
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        assert not int(top.planner_calculating.value)
        assert int(top.pi_calculating.value)
        assert int(top.controller.work.value) & 8191 == feedback & 8191
        for _ in range(2000):
            if not int(top.pi_calculating.value):
                break
            await RisingEdge(dut.clk)
            await Timer(1, unit="ns")
        else:
            raise AssertionError("PI exceeded shared-engine deadline")
        assert signed(top.velocity_command) == command
        assert int(top.pi_saturated.value) == saturated
        assert int(top.planner_calculating.value)
        for _ in range(2000):
            if not int(top.engine_calculating.value):
                break
            await RisingEdge(dut.clk)
            await Timer(1, unit="ns")
        else:
            raise AssertionError("planner exceeded shared-engine deadline")
        assert not int(top.planner_calculating.value)
        assert not int(top.pi_calculating.value)


@cocotb.test(skip=os.environ.get("GATES") == "yes")
async def shared_engine_abort_phases(dut):
    """Raw disable/reset inhibits pins in either engine phase without restart."""
    await initialize(dut)
    top = dut.user_project
    for abort in ("disable", "reset"):
        for phase in ("pi_calculating", "planner_calculating"):
            await spi_frame(dut, 0xC03F)
            await spi_frame(dut, 0x8004)
            set_input(dut, 5, 1)
            for _ in range(12000):
                await FallingEdge(dut.clk)
                if int(getattr(top, phase).value):
                    break
            else:
                raise AssertionError(f"engine never reached {phase}")
            if abort == "disable":
                set_input(dut, 5, 0)
            else:
                dut.rst_n.value = 0
            # Check the physical pins before the next clock can clear state.
            await Timer(1, unit="ns")
            assert int(dut.uo_out.value) & 7 == 0
            await Timer(1000, unit="ns")
            assert not int(top.engine_calculating.value)
            assert not int(top.planner_busy.value)
            assert int(top.velocity_command.value) == 0
            dut.rst_n.value = 1
            set_input(dut, 5, 1)
            # A cancelled move needs a fresh TARGET, even after two ticks.
            await Timer(2100, unit="us")
            assert int(dut.uo_out.value) & 0x1F == 0
            set_input(dut, 5, 0)
            await Timer(1000, unit="ns")
