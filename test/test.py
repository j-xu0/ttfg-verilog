# SPDX-License-Identifier: Apache-2.0
"""Top-level port and integration checks at 10 MHz clk / 500 kHz SPI."""
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import Edge, FallingEdge, ReadOnly, RisingEdge, Timer, with_timeout
from cocotb.utils import get_sim_time


_pins = 2

# Status word returned by every SPI frame: {0, BUSY, AT_TARGET, 11'b0, codes}.
STATUS_BUSY = 0x4000
STATUS_AT_TARGET = 0x2000
STATUS_ENC_FAULT = 0x0001
STATUS_RANGE_FAULT = 0x0002
STATUS_FAULT = STATUS_ENC_FAULT | STATUS_RANGE_FAULT
INT_PIN = 0x08
MOTOR_PINS = 0x07


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


async def spi_begin(dut):
    set_input(dut, 1, 0)
    await Timer(1000, unit="ns")


async def spi_bits(dut, word, count=16):
    """Clock `count` bits of `word`, MSB first, returning the reply bits."""
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
    return response


async def spi_end(dut):
    set_input(dut, 1, 1)
    await Timer(1000, unit="ns")


async def spi_frame(dut, word, count=16):
    await spi_begin(dut)
    response = await spi_bits(dut, word, count)
    await spi_end(dut)
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
        assert int(dut.uo_out.value) & 0x7F == 0
        assert int(dut.uio_oe.value) == 0

    # A legal cycle has no fault; a diagonal transition latches one and holds
    # the interrupt until a read acknowledges it.
    for phase in (1, 3, 2, 0):
        set_input(dut, 3, phase >> 1)
        set_input(dut, 4, phase & 1)
        await Timer(1000, unit="ns")
        assert not int(dut.uo_out.value) & INT_PIN
    set_input(dut, 3, 1)
    set_input(dut, 4, 1)
    await Timer(1000, unit="ns")
    assert int(dut.uo_out.value) & INT_PIN
    assert await spi_frame(dut, 0) == STATUS_ENC_FAULT
    assert not int(dut.uo_out.value) & INT_PIN

    dut.ena.value = 0
    await Timer(1000, unit="ns")
    assert int(dut.uo_out.value) == 0
    assert await spi_frame(dut, 0) == 0
    dut.ena.value = 1
    await Timer(1000, unit="ns")
    assert int(dut.uo_out.value) == 0

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
    for command, expected in ((0x9000, (4096, 0)),
                              (0xBFFA, (4096, 0)),
                              (0xDFD5, (4096, 5))):
        await spi_frame(dut, command)
        assert (int(spi.target.value),
                int(spi.plan.value)) == expected
    await spi_frame(dut, 0xFFFF, count=7)
    assert int(spi.target.value) == 4096
    assert int(spi.plan.value) == 5
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
            now = get_sim_time(unit="step")  # Integer picoseconds (tb.v precision).
            if pins & 1 and not self.previous & 1:
                assert pins & 4, "STEP without ENABLE"
                self.rise = now
                self.steps += 1
                self.rises.append(now)
                if self.drop_next:
                    self.drop_next = False
                else:
                    # Encoder response is asynchronous to clk and arrives well
                    # before the next possible pulse at the 4 kHz rate cap.
                    direction = 1 if pins & 2 else -1
                    await Timer(500, unit="ns")
                    self.advance(direction)
            elif self.previous & 1 and not pins & 1:
                self.widths.append(now - self.rise)
            self.previous = pins


async def wait_settled(dut, timeout_us=150000):
    for _ in range(timeout_us // 100):
        word = await spi_frame(dut, 0)
        assert not word & STATUS_FAULT, "unexpected feedback fault"
        if word & STATUS_AT_TARGET:
            assert not word & STATUS_BUSY, "settled while busy"
            assert not int(dut.uo_out.value) & 0x01, "settled with STEP asserted"
            return
        await Timer(100, unit="us")
    raise AssertionError("move did not settle within motor-model timeout")


@cocotb.test()
async def closed_loop_moves_and_disturbance(dut):
    await initialize(dut)
    motor = MotorModel(dut)
    cocotb.start_soon(motor.run())
    await spi_frame(dut, 0xA000)  # addr 1 reserved (ex-CONTROL), rejected, no effect
    await spi_frame(dut, 0xC03F)  # 10000 steps/s, ~100000 steps/s^2
    await spi_frame(dut, 0x8004)  # configure target while disabled
    await Timer(100, unit="us")
    assert int(dut.uo_out.value) & MOTOR_PINS == 0
    assert not int(dut.uo_out.value) & INT_PIN
    motor.drop_next = True
    set_input(dut, 5, 1)
    await Timer(1, unit="us")
    assert int(dut.uo_out.value) & 0x04
    assert await spi_frame(dut, 0) & STATUS_BUSY
    # A write whose own returned status word says BUSY is dropped; the frame
    # freezes motion starts, so that bit is the exact verdict.
    await spi_frame(dut, 0x8007)
    await wait_settled(dut)
    assert motor.position == 4
    assert motor.steps >= 5, "missed step was not corrected"
    assert await spi_frame(dut, 0) == STATUS_AT_TARGET

    # Regression: accepting a new TARGET after settle must launch a new plan,
    # rather than interpreting the new target as drift from the old reference.
    await spi_frame(dut, 0x8000 | ((-3) & 0x1FFF))
    await Timer(1, unit="us")
    assert int(dut.uo_out.value) & 0x04
    await wait_settled(dut)
    assert motor.position == -3
    steps_before = motor.steps
    motor.advance(1)  # external shaft disturbance after settling
    await Timer(1, unit="us")
    assert await spi_frame(dut, 0) & STATUS_BUSY
    await wait_settled(dut)
    assert motor.position == -3
    assert motor.steps > steps_before

    # Same-value TARGET commits are commands, too (not change detection).
    await spi_frame(dut, 0x8000 | ((-3) & 0x1FFF))
    await wait_settled(dut)
    assert motor.position == -3
    assert motor.widths and min(motor.widths) >= 5_000_000, motor.widths
    assert int(dut.uio_oe.value) == 0
    assert int(dut.uio_out.value) == 0


@cocotb.test(skip=os.environ.get("GATES") == "yes")
async def write_frame_freezes_queued_launch(dut):
    """A queued TARGET cannot start inside a write frame, so that frame's own
    BUSY bit is the exact write verdict."""
    await initialize(dut)
    top = dut.user_project
    motor = MotorModel(dut)
    cocotb.start_soon(motor.run())
    await spi_frame(dut, 0xC03F)
    await spi_frame(dut, 0x8004)  # queue +4 while disabled
    await Timer(10, unit="us")
    assert int(top.command_pending.value) == 1
    assert int(top.busy.value) == 0

    await spi_begin(dut)
    high = await spi_bits(dut, (0x8000 | 8) >> 8, count=8)
    # Motion enable rises with the write frame still in flight. Without the
    # gate this is the instant the queued +4 would have armed.
    set_input(dut, 5, 1)
    await Timer(5, unit="us")
    assert int(top.active.value) == 0, "queued move started inside write frame"
    assert int(top.busy.value) == 0, "BUSY rose inside write frame"
    low = await spi_bits(dut, (0x8000 | 8) & 0xFF, count=8)
    await spi_end(dut)
    assert (high << 8 | low) & STATUS_BUSY == 0
    await Timer(10, unit="us")
    # The in-frame write won: the queued +4 was replaced by +8.
    assert int(top.target.value) == 8
    await wait_settled(dut)
    assert motor.position == 8
    assert await spi_frame(dut, 0) == STATUS_AT_TARGET


@cocotb.test()
async def motion_abort_and_fault(dut):
    await initialize(dut)
    await spi_frame(dut, 0xC03F)
    await spi_frame(dut, 0x800A)
    set_input(dut, 5, 1)
    await Timer(4, unit="ms")
    assert int(dut.uo_out.value) & 0x04
    # A disable does not resume the aborted command on the next enable.
    set_input(dut, 5, 0)
    await Timer(10, unit="ns")
    assert int(dut.uo_out.value) & MOTOR_PINS == 0
    await Timer(1, unit="us")
    set_input(dut, 5, 1)
    await Timer(2, unit="ms")
    # The abort holds the interrupt until a read consumes the status word.
    assert int(dut.uo_out.value) & INT_PIN
    assert await spi_frame(dut, 0) == 0
    assert not int(dut.uo_out.value) & INT_PIN
    await spi_frame(dut, 0x800A)  # explicit rewrite required
    await Timer(2, unit="us")
    assert int(dut.uo_out.value) & 0x04
    # Illegal two-bit encoder transition latches a fault and stops outputs.
    set_input(dut, 3, 1)
    set_input(dut, 4, 1)
    await Timer(1, unit="us")
    assert int(dut.uo_out.value) & MOTOR_PINS == 0
    assert int(dut.uo_out.value) & INT_PIN
    assert await spi_frame(dut, 0) == STATUS_ENC_FAULT
    await Timer(2, unit="us")
    assert int(dut.uo_out.value) & MOTOR_PINS == 0
    assert not int(dut.uo_out.value) & INT_PIN
    await spi_frame(dut, 0x8001)
    await Timer(2, unit="us")
    assert int(dut.uo_out.value) & 0x04
    dut.rst_n.value = 0
    await Timer(1, unit="us")
    assert int(dut.uo_out.value) == 0
    dut.rst_n.value = 1
    await Timer(2, unit="ms")
    assert int(dut.uo_out.value) == 0


@cocotb.test()
async def subcycle_disable_during_step(dut):
    await initialize(dut)
    motor = MotorModel(dut)
    cocotb.start_soon(motor.run())
    await spi_frame(dut, 0xC007)
    await spi_frame(dut, 0x8040)
    set_input(dut, 5, 1)

    async def wait_step():
        while True:
            await Edge(dut.uo_out)
            if int(dut.uo_out.value) & 1:
                return

    await with_timeout(wait_step(), 20, "ms")
    await FallingEdge(dut.clk)
    await Timer(10, unit="ns")
    assert int(dut.uo_out.value) & 0x05 == 0x05, "expected STEP with the driver enabled"
    # The 20 ns low pulse fits between rising clocks, inside the high STEP.
    set_input(dut, 5, 0)
    await Timer(10, unit="ns")
    assert int(dut.uo_out.value) & MOTOR_PINS == 0, "disable did not inhibit motor pins"
    await Timer(10, unit="ns")
    set_input(dut, 5, 1)
    await Timer(10, unit="ns")
    assert int(dut.uo_out.value) & MOTOR_PINS == 0, "STEP or driver pins reasserted before cleanup"
    await Timer(5, unit="ms")
    assert int(dut.uo_out.value) & MOTOR_PINS == 0
    # The abort is a busy edge, so it raises the interrupt just as a completed
    # move would; only a status read acknowledges it.
    assert int(dut.uo_out.value) & INT_PIN
    assert motor.steps == 1, "aborted move emitted another STEP"

    # A fresh command still works after the abort.
    await spi_frame(dut, 0x8004)
    await wait_settled(dut)
    assert motor.position == 4


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
    assert int(dut.uo_out.value) & INT_PIN
    assert await spi_frame(dut, 0) == STATUS_RANGE_FAULT

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
    assert not int(dut.uo_out.value) & INT_PIN
    set_input(dut, 3, 1)  # 00 -> 10 decrements once more and wraps
    await Timer(1, unit="us")
    assert int(dut.uo_out.value) & INT_PIN
    set_input(dut, 5, 1)
    await spi_frame(dut, 0x8001)  # writes never acknowledge
    await Timer(1, unit="ms")
    assert int(dut.uo_out.value) & INT_PIN


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
    assert min(gaps) >= 250_000_000, "exceeded 4 kHz cap"
    assert gaps.count(250_000_000) > 5, "did not exercise the 4 kHz ceiling"
    # Lowest velocity/acceleration codes exercise the Q11 low-acceleration path
    # through SPI, planning, P feedback and actual encoder feedback.
    await spi_frame(dut, 0xC000)
    await spi_frame(dut, 0x8000 | 1201)
    await wait_settled(dut, timeout_us=300000)
    assert motor.position == 1201
    assert motor.widths and min(motor.widths) >= 5_000_000


@cocotb.test(skip=os.environ.get("GATES") == "yes")
async def serial_reference_sampling(dut):
    """P feedback snapshots the completed reference before the planner reuses it."""
    await initialize(dut)
    await spi_frame(dut, 0xA03F)  # addr 1 reserved (ex-CONTROL), rejected, no effect
    await spi_frame(dut, 0xC03F)
    await spi_frame(dut, 0x8004)
    set_input(dut, 5, 1)
    top = dut.user_project

    def signed(signal):
        raw = int(signal.value)
        return raw - (1 << len(signal)) if raw >> (len(signal)-1) else raw

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
        # P-only fixed-Kp path: integer error scaled by 2048 (Q11), then the
        # fixed 1/4 gain. No velocity feedforward. The clamp is the fixed
        # 8191 Q11 velocity limit set by the current engine experiment.
        vmax = 8191
        error = (reference // 2048 - feedback) * 2048
        total = error // 4
        command = max(-vmax, min(vmax, total))
        saturated = int(command != total)
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        assert not int(top.planner_calculating.value) or int(top.engine_calculating.value)
        assert int(top.pi_calculating.value)
        assert signed(top.velocity_command) == command
        assert int(top.pi_saturated.value) == saturated
        # The command is registered at the tick edge: a later encoder sample
        # must not change this tick's published value.
        set_input(dut, 3, 1)
        await Timer(1000, unit="ns")
        assert signed(top.velocity_command) == command
        for _ in range(2000):
            if not int(top.pi_calculating.value):
                break
            await RisingEdge(dut.clk)
            await Timer(1, unit="ns")
        else:
            raise AssertionError("P strobe exceeded deadline")
        assert signed(top.velocity_command) == command
        assert int(top.planner_calculating.value)
        for _ in range(2000):
            if not int(top.engine_calculating.value):
                break
            await RisingEdge(dut.clk)
            await Timer(1, unit="ns")
        else:
            raise AssertionError("planner exceeded engine deadline")
        assert not int(top.planner_calculating.value)
        assert not int(top.pi_calculating.value)
        assert signed(top.velocity_command) == command


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
            assert int(dut.uo_out.value) & MOTOR_PINS == 0
            set_input(dut, 5, 0)
            await Timer(1000, unit="ns")


@cocotb.test()
async def spi_init_short_move(dut):
    """Directed waveform: SPI setup writes, then one tiny accel/decel move.

    PLAN is written while motion is disabled, then TARGET queues
    a 64-step move. Enabling starts it. Sixty-four steps is far below the
    profile's accel+decel distance, so the planner never reaches the velocity
    limit and there is no cruise phase: the STEP spacing falls then rises.
    """
    await initialize(dut)
    motor = MotorModel(dut)
    cocotb.start_soon(motor.run())

    await spi_frame(dut, 0xA004)  # addr 1 reserved (ex-CONTROL), rejected, no effect
    await spi_frame(dut, 0xC03F)  # PLAN: 10000 steps/s, ~100000 steps/s^2
    await spi_frame(dut, 0x8040)  # TARGET: +64 steps, queued while disabled
    await Timer(100, unit="us")
    assert int(dut.uo_out.value) & MOTOR_PINS == 0, "disabled setup drove pins"

    set_input(dut, 5, 1)  # motion enable launches the queued TARGET
    await wait_settled(dut, timeout_us=100000)
    assert motor.position == 64
    assert motor.steps == 64, f"expected one STEP per step, saw {motor.steps}"
    assert await spi_frame(dut, 0) == STATUS_AT_TARGET

    gaps = [b - a for a, b in zip(motor.rises, motor.rises[1:])]
    assert len(gaps) >= 4, f"too few STEP pulses to show a profile: {gaps}"
    middle = min(range(len(gaps)), key=gaps.__getitem__)
    assert 0 < middle < len(gaps) - 1, f"no accel/decel split: {gaps}"
    assert gaps[0] > gaps[middle] and gaps[-1] > gaps[middle], gaps


@cocotb.test()
async def fault_acknowledge_and_zero_position(dut):
    await initialize(dut)
    for phase in (1, 3, 2, 0):
        set_input(dut, 3, phase >> 1)
        set_input(dut, 4, phase & 1)
        await Timer(1, unit="us")
    set_input(dut, 3, 1)
    set_input(dut, 4, 1)
    await Timer(1, unit="us")
    # Writes and truncated reads cannot acknowledge. Fault report is bit-coded.
    assert await spi_frame(dut, 0xC001) == STATUS_ENC_FAULT
    await spi_frame(dut, 0, count=15)
    assert int(dut.uo_out.value) & INT_PIN
    assert await spi_frame(dut, 0) == STATUS_ENC_FAULT
    assert not int(dut.uo_out.value) & INT_PIN
    # Fault acknowledgement preserves the four legal counts.
    set_input(dut, 5, 1)
    await spi_frame(dut, 0x8004)
    await wait_settled(dut)
    # Zero is rejected while enabled, even if settled.
    await spi_frame(dut, 0xE000)
    assert await spi_frame(dut, 0) == STATUS_AT_TARGET
    set_input(dut, 5, 0)
    await Timer(1, unit="us")
    await spi_frame(dut, 0xE000)
    set_input(dut, 5, 1)
    await Timer(1, unit="us")
    assert int(dut.uo_out.value) & MOTOR_PINS == 0
    await spi_frame(dut, 0x8000)
    await wait_settled(dut)


@cocotb.test(skip=os.environ.get("GATES") == "yes")
async def zero_position_requires_synchronized_disable(dut):
    await initialize(dut)
    top = dut.user_project
    set_input(dut, 4, 1)  # One legal encoder count, no TARGET queued.
    await Timer(1, unit="us")
    assert int(top.position.value) == 1

    set_input(dut, 5, 1)
    await Timer(1, unit="us")
    await spi_begin(dut)
    assert await spi_bits(dut, 0xE000) == 0
    set_input(dut, 1, 1)
    # CS commits on the third rising edge. Pull enable low after the
    # second edge, too late for the disable to reach the SPI clock domain.
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    await Timer(20, unit="ns")
    set_input(dut, 5, 0)
    await Timer(1, unit="us")
    assert int(top.position.value) == 1, "late raw disable authorized zeroing"

    await spi_begin(dut)
    assert await spi_bits(dut, 0xE000) == 0
    set_input(dut, 5, 1)
    await Timer(1, unit="us")
    set_input(dut, 5, 0)
    await Timer(1, unit="us")
    set_input(dut, 1, 1)
    await Timer(1, unit="us")
    assert int(top.position.value) == 1, "enable during frame authorized zeroing"

    await spi_begin(dut)
    assert await spi_bits(dut, 0xE000) == 0
    set_input(dut, 5, 1)
    set_input(dut, 1, 1)
    await Timer(1, unit="us")
    assert int(top.position.value) == 1, "enable at commit authorized zeroing"

    set_input(dut, 5, 0)
    await Timer(400, unit="ns")
    await spi_frame(dut, 0x8004)
    assert int(top.target.value) == 4
    assert int(top.command_pending.value)
    await spi_frame(dut, 0xE000)
    assert int(top.position.value) == 0
    assert int(top.target.value) == 0
    assert not int(top.command_pending.value)
