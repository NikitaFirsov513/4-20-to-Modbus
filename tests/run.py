"""Run the actual ARM C implementation with fake RTOS/Flash and mapped registers.
Install test deps: python -m pip install --target build/test-deps unicorn==2.1.4 pyelftools==0.33
The emulator tests register ordering; it does not emulate electrical timing.
"""
import argparse
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'build/test-deps'))
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_WRITE, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0

def execute(elf_path, flash_model=False):
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.mem_map(0x08000000, 0x100000)
    uc.mem_map(0x20000000, 0x10000)
    uc.mem_map(0x1ffff000, 0x1000)
    uc.mem_map(0x40000000, 0x30000)
    uc.mem_map(0xE0000000, 0x100000)
    sentinel = 0x080ffff0
    with elf_path.open('rb') as stream:
        elf = ELFFile(stream)
        for segment in elf.iter_segments():
            if segment['p_type'] == 'PT_LOAD' and segment['p_filesz']:
                uc.mem_write(segment['p_vaddr'], segment.data())
        symbols = {s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
    erases = [0]
    # F1 PR/RLR update handshake is only clocked after the start key.
    # A plain memory model previously missed this boot-time deadlock.
    watchdog_started = [False]
    def watchdog_write(machine, access, address, size, value, user):
        if address == 0x40003000 and value == 0xCCCC:
            watchdog_started[0] = True
        if address in (0x40003004, 0x40003008) and not watchdog_started[0]:
            raise AssertionError('IWDG configured before its peripheral was started')
    uc.hook_add(UC_HOOK_MEM_WRITE,watchdog_write,begin=0x40003000,end=0x4000300f)
    if flash_model:
        # Model F1 unlock, W1C SR flags, page erase and program-error status.
        uc.mem_write(0x0800f800,bytes([255])*2048)
        uc.mem_write(0x0800fc00,bytes.fromhex('01 00 03 00 00 00 01 00 80 64'))
        uc.mem_write(0x40022010, (0x80).to_bytes(4,'little'))
        pending = []
        def read32(address): return int.from_bytes(uc.mem_read(address,4),'little')
        def write_hook(machine, access, address, size, value, user):
            fault = read32(symbols['test_flash_fault'])
            if address == 0x40022004 and value == 0xcdef89ab and fault != 1:
                pending.append((0x40022010, read32(0x40022010) & ~0x80))
            if address == 0x4002200c: pending.append((address, read32(address) & ~value))
            if address == 0x40022010 and value & 0x40:
                erases[0] += 1
                if fault == 2: pending.append((0x4002200c, 4))
                else:
                    page = read32(0x40022014)
                    assert page in (0x0800f800,0x0800fc00)
                    machine.mem_write(page,bytes([255])*1024)
                pending.append((address, value & ~0x40))
            if 0x0800f800 <= address < 0x08010000 and size == 2:
                index = read32(symbols['test_program_index'])
                machine.mem_write(symbols['test_program_index'],(index+1).to_bytes(4,'little'))
                if fault == 3 and index == read32(symbols['test_fail_after']):
                    pending.append((0x4002200c,4))
                    # Program failure leaves this halfword erased.
                    pending.append((address,0xffff,2))
            if address == symbols['test_corrupt_address'] and value:
                old = int.from_bytes(machine.mem_read(value,2),'little')
                pending.append((value,old^1,2))
        def instruction_hook(machine, address, size, user):
            while pending:
                item = pending.pop(0)
                target,value = item[:2]
                size = item[2] if len(item)>2 else 4
                machine.mem_write(target,value.to_bytes(size,'little'))
        uc.hook_add(UC_HOOK_MEM_WRITE,write_hook)
        uc.hook_add(UC_HOOK_CODE,instruction_hook)
    uc.reg_write(UC_ARM_REG_SP, 0x20010000)
    uc.reg_write(UC_ARM_REG_LR, sentinel | 1)
    uc.emu_start(symbols['test_main'] | 1, sentinel, timeout=60_000_000, count=1_000_000_000)
    result = uc.reg_read(UC_ARM_REG_R0)
    checks = int.from_bytes(uc.mem_read(symbols['checks'],4),'little')
    from unicorn.arm_const import UC_ARM_REG_PC
    if uc.reg_read(UC_ARM_REG_PC) != sentinel:
        raise RuntimeError(f'Emulation exceeded budget: PC={uc.reg_read(UC_ARM_REG_PC):#x}, R0={result}, checks={checks}')
    if result:
        source = 'test_flash.c' if flash_model else 'test_firmware.c'
        raise AssertionError(f'C regression failed at tests/{source}:{result} ({checks} checks reached)')
    if flash_model and erases[0] != 20:
        raise AssertionError(f'Unexpected erase count: {erases[0]}')
    return checks

def panic_campaign(elf_path):
    with elf_path.open('rb') as stream:
        elf=ELFFile(stream)
        segments=[(p['p_vaddr'],p.data()) for p in elf.iter_segments() if p['p_type']=='PT_LOAD' and p['p_filesz']]
        symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
    def machine():
        u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
        for start,size in [(0x08000000,0x100000),(0x20000000,0x10000),(0x40000000,0x30000),(0xE0000000,0x100000),(0x1ffff000,0x1000)]:u.mem_map(start,size)
        for address,data in segments:u.mem_write(address,data)
        u.reg_write(UC_ARM_REG_SP,0x20010000);u.reg_write(UC_ARM_REG_LR,0x080ffff1)
        return u
    u=machine();reset=[]
    def write(m,access,address,size,value,user):
        if address==0xE000ED0C and value&4:reset.append(True);m.emu_stop()
    u.hook_add(UC_HOOK_MEM_WRITE,write)
    u.reg_write(UC_ARM_REG_R0,2) # FAULT_HARD
    u.emu_start(symbols['Health_Panic']|1,0x080ffff0,count=10000)
    if not reset or int.from_bytes(u.mem_read(0x40010814,4),'little')!=0x100:
        raise AssertionError('Panic did not release PA8 and request reset')
    bkp=bytes(u.mem_read(0x40006C00,64))
    reboot=machine();reboot.mem_write(0x40006C00,bkp)
    reboot.mem_write(0x40021024,(1<<28).to_bytes(4,'little')) # RCC SFTRSTF
    reboot.emu_start(symbols['health_reboot_check']|1,0x080ffff0,count=100000)
    if reboot.reg_read(UC_ARM_REG_R0):raise AssertionError('Saved fault/reset source lost on reboot')
    print('PASS: fatal error releases DE, requests reset, and preserves the reason across CPU restart')

def powercut_campaign(elf_path):
    """Reset the emulated CPU after erase and after every halfword, retaining
    only the two Flash pages; recover using production boot/load functions."""
    with elf_path.open('rb') as stream:
        elf=ELFFile(stream)
        segments=[(p['p_vaddr'],p.data()) for p in elf.iter_segments() if p['p_type']=='PT_LOAD' and p['p_filesz']]
        symbols={s.name:s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
    def new_machine(pages=None):
        machine=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
        for start,size in [(0x08000000,0x100000),(0x20000000,0x10000),(0x40000000,0x30000)]:machine.mem_map(start,size)
        for address,data in segments:machine.mem_write(address,data)
        machine.mem_write(0x0800f800,pages if pages is not None else bytes([255])*2048)
        machine.mem_write(0x40022010,(128).to_bytes(4,'little'))
        return machine
    def invoke(machine,name):
        machine.reg_write(UC_ARM_REG_SP,0x20010000)
        machine.reg_write(UC_ARM_REG_LR,0x080ffff1)
        machine.emu_start(symbols[name]|1,0x080ffff0,count=2000000)
        return machine.reg_read(UC_ARM_REG_R0)
    def model(machine,cut=None):
        pending=[];state={'operation':-1,'stop':False}
        def read32(address):return int.from_bytes(machine.mem_read(address,4),'little')
        def write(m,access,address,size,value,user):
            if address==0x40022004 and value==0xcdef89ab:pending.append((0x40022010,read32(0x40022010)&~128))
            if address==0x4002200c:pending.append((address,0))
            if address==0x40022010 and value&0x40:
                page=read32(0x40022014)
                m.mem_write(page,bytes([255])*1024)
                pending.append((address,value&~0x40))
                state['operation']+=1
                if state['operation']==cut:state['stop']=True
            if 0x0800f800<=address<0x08010000 and size==2:
                state['operation']+=1
                if state['operation']==cut:state['stop']=True
        def instruction(m,address,size,user):
            while pending:
                target,value=pending.pop(0);m.mem_write(target,value.to_bytes(4,'little'))
            if state['stop']:m.emu_stop()
        machine.hook_add(UC_HOOK_MEM_WRITE,write)
        machine.hook_add(UC_HOOK_CODE,instruction)
    seed=new_machine();model(seed)
    if invoke(seed,'powercut_seed')!=0:raise AssertionError('Power-cut seed failed')
    baseline=bytes(seed.mem_read(0x0800f800,2048))
    for cut in range(17):
        machine=new_machine(baseline);model(machine,cut)
        invoke(machine,'powercut_update')
        interrupted=bytes(machine.mem_read(0x0800f800,2048))
        reboot=new_machine(interrupted)
        result=invoke(reboot,'powercut_check')
        expected=2 if cut==16 else 1
        if result!=expected:raise AssertionError(f'Power cut {cut}: loaded {result}, expected {expected}')
    print('PASS: 17 reset-after-erase/program cases with a fresh CPU boot and preserved calibration')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--toolchain', default=str(Path(os.environ.get('USERPROFILE', '~')) / '.eide/tools/gcc_arm/bin'))
    args = parser.parse_args()
    os.chdir(ROOT)
    out = ROOT / 'build/tests'
    out.mkdir(parents=True, exist_ok=True)
    gcc = Path(args.toolchain) / ('arm-none-eabi-gcc.exe' if os.name == 'nt' else 'arm-none-eabi-gcc')
    flags = ['-mcpu=cortex-m3','-mthumb','-std=c11','-Og','-g','-Wall','-Wextra','-Werror',
             '-ffunction-sections','-fdata-sections','-DSTM32F103xB','-Itests/stubs','-IApp/Inc',
             '-IDrivers/CMSIS/Device/ST/STM32F1xx/Include','-IDrivers/CMSIS/Include',
             '-nostartfiles','--specs=nosys.specs','--specs=nano.specs','-Wl,--gc-sections','-Ttests/link.ld']
    sources = ['tests/test_firmware.c','App/Src/Modbus.c','App/Src/ModbusRegisters.c',
               'App/Src/UART.c','App/Src/Tim.c','App/Src/SensorMath.c','App/Src/Health.c']
    elf_path = out / 'tests.elf'
    subprocess.run([str(gcc),*flags,'-Wl,--undefined=health_reboot_check',*sources,'-o',str(elf_path)],check=True)
    checks = execute(elf_path)
    panic_campaign(elf_path)
    print(f'PASS: {checks} assertions; 20000 malformed/random frames; protocol, configuration, ADC and RS-485/TC checks')
    flash_path = out / 'flash.elf'
    flash_includes = ['-ICore/Inc','-IDrivers/STM32F1xx_HAL_Driver/Inc','-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy']
    subprocess.run([str(gcc),*flags,*flash_includes,'-DUSE_FULL_LL_DRIVER',
                    '-Wl,--undefined=powercut_seed','-Wl,--undefined=powercut_update','-Wl,--undefined=powercut_check',
                    'tests/test_flash.c','App/Src/ConfigStore.c','App/Src/Modbus.c','App/Src/SensorMath.c','App/Src/Health.c','-o',str(flash_path)],check=True)
    checks = execute(flash_path,flash_model=True)
    powercut_campaign(flash_path)
    print(f'PASS: {checks} Flash checks; dual-page journal, legacy migration, calibration preservation, all 16 halfword failures and CRC fallback')

if __name__ == '__main__': main()
