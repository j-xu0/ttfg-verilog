"""Independent numerical and control-contract checks for the compact planner."""
from pathlib import Path
from functools import lru_cache
import random
import pytest
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import Timer
from cocotb.utils import get_sim_time

VELOCITIES = (6554,13107,32768,65536,131072,262144,458752,655360)
ACCELS = (33,66,131,328,655,1311,3277,6554)
MAX_CALCULATION_CYCLES = 0

def signed(signal):
    value = int(signal.value)
    return value - (1 << len(signal)) if value >> (len(signal)-1) else value

def snapshot(dut):
    return signed(dut.reference_position), signed(dut.reference_velocity), int(dut.busy.value)

async def cycle(dut, count=1):
    await Timer(10*count, unit='ns')

async def reset(dut):
    dut.enable.value=1; dut.start.value=0; dut.tick.value=0
    dut.start_position.value=0; dut.target_position.value=0; dut.plan_code.value=0
    dut.rst_n.value=0
    await cycle(dut,2)
    dut.rst_n.value=1
    await cycle(dut)
    assert snapshot(dut)==(0,0,0)

@lru_cache(maxsize=32768)
def stop_sum(v,a):
    return sum(range(v,0,-a))

def oracle(pos,vel,goal,vmax,accel):
    pos+=vel
    if pos==goal: return pos,0,0
    rem=abs(goal-pos); speed=abs(vel)
    chosen=max(speed-accel,0)
    for candidate in (min(speed+accel,vmax),speed,chosen):
        if stop_sum(candidate,accel)<=rem:
            chosen=candidate; break
    if speed<=accel and rem<=min(accel,vmax): chosen=rem
    return pos,chosen if goal>pos else -chosen,1

async def command(dut,origin,target,code):
    dut.start_position.value=origin &8191; dut.target_position.value=target&8191
    dut.plan_code.value=code; dut.start.value=1
    await cycle(dut)
    dut.start.value=0
    assert snapshot(dut)==(origin*65536,0,int(origin!=target))
    assert int(dut.done.value)==int(origin==target)

async def move(dut,origin,target,code,disturb=False):
    global MAX_CALCULATION_CYCLES
    await command(dut,origin,target,code)
    if origin==target:
        await cycle(dut); assert not int(dut.done.value); return
    vmax=VELOCITIES[code&7]; accel=ACCELS[code>>3]
    goal=target*65536
    old=snapshot(dut)
    for index in range(20000):
        if disturb:
            dut.start_position.value=(-target)&8191
            if not int(dut.STABLE_CONFIG.value):
                dut.target_position.value=(-origin)&8191
                dut.plan_code.value=(code+23)&63
            dut.start.value=index%2
        # Tick gaps preserve the settled reference.
        if index%9==0:
            await cycle(dut,3); assert snapshot(dut)==old
        calculation_started = get_sim_time(unit='ns')
        dut.tick.value=1
        await cycle(dut)
        dut.tick.value=0
        dut.start.value=0
        assert int(dut.calculating.value)
        # An extra tick while the ALU is occupied cannot queue a second step.
        if index % 11 == 0:
            await cycle(dut,17)
            dut.tick.value=1
            await cycle(dut)
            dut.tick.value=0
        elapsed=0
        while int(dut.calculating.value):
            await cycle(dut,10); elapsed+=10
            assert elapsed<700
        calculation_cycles = (get_sim_time(unit='ns') - calculation_started) // 10
        MAX_CALCULATION_CYCLES = max(MAX_CALCULATION_CYCLES, calculation_cycles)
        assert calculation_cycles < 700
        actual=snapshot(dut)
        expected=oracle(old[0],old[1],goal,vmax,accel)
        assert actual==expected,(origin,target,code,index,actual,expected)
        assert actual[0]-old[0]==old[1]
        assert min(origin*65536,goal)<=actual[0]<=max(origin*65536,goal)
        assert abs(actual[1])<=vmax
        assert abs(actual[1]-old[1])<=accel
        assert abs(goal-actual[0])<=abs(goal-old[0])
        old=actual
        if not actual[2]:
            assert actual==(goal,0,0)
            # done is one clock pulse, and polling may already have passed it.
            dut.start.value=0
            await cycle(dut,2)
            assert not int(dut.done.value)
            return
    raise AssertionError(('timeout',origin,target,code))

@cocotb.test()
async def compact_contract(dut):
    cocotb.start_soon(Clock(dut.clk,10,unit='ns',impl='gpi').start())
    await Timer(1,unit='ns')
    await reset(dut)
    for code in range(64):
        await move(dut,0,3,code,True)
        await move(dut,2,-1,code,True)
    for case in ((-4096,4095,63),(4095,-4096,63),(0,0,0),
                 (4095,4094,0),(-4096,-4095,7)):
        await move(dut,*case,disturb=True)
    rng=random.Random(0x180)
    for _ in range(20):
        origin=rng.randint(-4096,4095)
        target=max(-4096,min(4095,origin+rng.randint(-15,15)))
        await move(dut,origin,target,rng.randrange(64),True)
    # Abort/reset during an unfinished calculation cannot publish stale work.
    for reset_abort in (False,True):
        await command(dut,0,100,7)
        dut.tick.value=1; await cycle(dut); dut.tick.value=0
        await cycle(dut,17)
        assert int(dut.calculating.value)
        if reset_abort: dut.rst_n.value=0
        else: dut.enable.value=0
        await cycle(dut)
        assert not int(dut.busy.value)
        assert signed(dut.reference_velocity)==0
        assert not int(dut.calculating.value)
        assert not int(dut.done.value)
        dut.enable.value=1; dut.rst_n.value=1
        await cycle(dut)
        await move(dut,-2,2,24)
    dut._log.info('Maximum observed calculation latency: %s clocks', MAX_CALCULATION_CYCLES)


@pytest.mark.parametrize("stable_config", [0, 1])
def test_compact_planner(tmp_path, stable_config):
    from cocotb_tools.runner import get_runner
    root=Path(__file__).resolve().parents[1]
    runner=get_runner('icarus')
    runner.build(sources=[root/'src'/'compact_motion_planner.v'],
                 includes=[root/'src'],hdl_toplevel='compact_motion_planner',
                 parameters={'STABLE_CONFIG':stable_config},
                 build_dir=tmp_path/'compact',timescale=('1ns','1ps'),always=True)
    runner.test(hdl_toplevel='compact_motion_planner',test_module='test_compact_planner',
                test_dir=Path(__file__).parent,results_xml=str(tmp_path/'results.xml'))


def test_generated_plan_rom():
    import runpy
    root = Path(__file__).resolve().parents[1]
    script = runpy.run_path(str(root / 'scripts' / 'generate_plan_lut.py'))
    assert (root / 'src' / 'plan_lut.vh').read_text() == script['render']()
