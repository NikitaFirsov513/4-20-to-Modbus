"""Read-only Modbus probe for a Windows COM port; tests adapter direction modes."""
import ctypes as c
from ctypes import wintypes as w
import time
import argparse

class DCB(c.Structure):
    _fields_=[('length',w.DWORD),('baud',w.DWORD),('flags',w.DWORD),('reserved',w.WORD),('xonlim',w.WORD),('xofflim',w.WORD),('bytesize',w.BYTE),('parity',w.BYTE),('stopbits',w.BYTE),('xon',c.c_char),('xoff',c.c_char),('error',c.c_char),('eof',c.c_char),('event',c.c_char),('reserved1',w.WORD)]
class Timeouts(c.Structure):
    _fields_=[(name,w.DWORD) for name in ['interval','readmult','readconst','writemult','writeconst']]
class Comstat(c.Structure):
    _fields_=[('flags',w.DWORD),('inqueue',w.DWORD),('outqueue',w.DWORD)]

def crc16(data):
    crc=0xffff
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc=(crc >> 1)^0xa001 if crc & 1 else crc >> 1
    return crc

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--port',default='COM6')
    parser.add_argument('--measurements',type=int,default=0,
                        help='Read current/ADC/status this many times instead of testing RTS/DTR')
    parser.add_argument('--expected-ma',type=float,default=None)
    args=parser.parse_args()
    k=c.WinDLL('kernel32',use_last_error=True)
    signatures={
        'CreateFileW':([w.LPCWSTR,w.DWORD,w.DWORD,c.c_void_p,w.DWORD,w.DWORD,w.HANDLE],w.HANDLE),
        'CloseHandle':([w.HANDLE],w.BOOL),
        'GetCommState':([w.HANDLE,c.POINTER(DCB)],w.BOOL),
        'SetCommState':([w.HANDLE,c.POINTER(DCB)],w.BOOL),
        'SetCommTimeouts':([w.HANDLE,c.POINTER(Timeouts)],w.BOOL),
        'GetCommTimeouts':([w.HANDLE,c.POINTER(Timeouts)],w.BOOL),
        'PurgeComm':([w.HANDLE,w.DWORD],w.BOOL),
        'ReadFile':([w.HANDLE,c.c_void_p,w.DWORD,c.POINTER(w.DWORD),c.c_void_p],w.BOOL),
        'WriteFile':([w.HANDLE,c.c_void_p,w.DWORD,c.POINTER(w.DWORD),c.c_void_p],w.BOOL),
        'FlushFileBuffers':([w.HANDLE],w.BOOL),
        'ClearCommError':([w.HANDLE,c.POINTER(w.DWORD),c.POINTER(Comstat)],w.BOOL),
        'GetCommModemStatus':([w.HANDLE,c.POINTER(w.DWORD)],w.BOOL),
    }
    for name,(types,result) in signatures.items():
        f=getattr(k,name);f.argtypes=types;f.restype=result
    def check(ok,what):
        if not ok: raise c.WinError(c.get_last_error(),what)
    h=k.CreateFileW('\\\\.\\'+args.port,0xc0000000,0,None,3,0,None)
    if h==w.HANDLE(-1).value: raise c.WinError(c.get_last_error())
    original_dcb=DCB();original_dcb.length=c.sizeof(original_dcb)
    original_timeouts=Timeouts()
    saved_dcb=bool(k.GetCommState(h,c.byref(original_dcb)))
    saved_timeouts=bool(k.GetCommTimeouts(h,c.byref(original_timeouts)))
    readings=[]
    try:
        tx=bytes.fromhex('01 03 01 00 00 04 45 F5') if args.measurements else bytes.fromhex('01 03 00 01 00 01 D5 CA')
        print('TX: '+tx.hex(' ').upper(),flush=True)
        modes=[(0,0)]*args.measurements if args.measurements else [(0,0),(1,0),(3,0),(0,1),(1,1)]
        for rts,dtr in modes:
            dcb=DCB();dcb.length=c.sizeof(dcb)
            check(k.GetCommState(h,c.byref(dcb)),'GetCommState')
            dcb.baud=9600;dcb.bytesize=8;dcb.parity=0;dcb.stopbits=0
            dcb.flags=1|(rts<<12)|(dtr<<4)
            check(k.SetCommState(h,c.byref(dcb)),'SetCommState')
            timeout=Timeouts(0xffffffff,0,20,0,1000)
            check(k.SetCommTimeouts(h,c.byref(timeout)),'SetCommTimeouts')
            check(k.PurgeComm(h,0x0c),'PurgeComm')
            time.sleep(.1)
            written=w.DWORD()
            check(k.WriteFile(h,tx,len(tx),c.byref(written),None),'WriteFile')
            check(k.FlushFileBuffers(h),'FlushFileBuffers')
            rx=bytearray();start=time.monotonic()
            while time.monotonic()-start<1:
                buf=c.create_string_buffer(256);n=w.DWORD()
                check(k.ReadFile(h,buf,256,c.byref(n),None),'ReadFile')
                rx+=buf.raw[:n.value]
                if rx and time.monotonic()-start>.15: break
            errors=w.DWORD();stat=Comstat();modem=w.DWORD()
            check(k.ClearCommError(h,c.byref(errors),c.byref(stat)),'ClearCommError')
            k.GetCommModemStatus(h,c.byref(modem))
            print(f'{args.port} 9600 8N1 RTS={rts} DTR={dtr}: TX={written.value} RX={rx.hex(" ").upper() or "NONE"} errors=0x{errors.value:X} modem=0x{modem.value:X}',flush=True)
            if args.measurements:
                if len(rx)!=13 or rx[:3]!=bytes([1,3,8]) or crc16(rx[:-2])!=int.from_bytes(rx[-2:],'little'):
                    print('Invalid or missing measurement response',flush=True)
                    continue
                ua,mv,raw,status=[int.from_bytes(rx[i:i+2],'big') for i in range(3,11,2)]
                readings.append((ua,mv,raw,status))
                print(f'CRC OK; current={ua/1000:.3f} mA; ADC={mv} mV; raw={raw}; status=0x{status:04X}',flush=True)
        if readings:
            currents=[r[0]/1000 for r in readings]
            mean=sum(currents)/len(currents)
            print(f'Valid responses={len(readings)}/{args.measurements}; mean={mean:.4f} mA; min={min(currents):.3f}; max={max(currents):.3f}',flush=True)
            if args.expected_ma is not None:
                print(f'Deviation from {args.expected_ma:.3f} mA: {mean-args.expected_ma:+.4f} mA',flush=True)
    finally:
        if saved_dcb:k.SetCommState(h,c.byref(original_dcb))
        if saved_timeouts:k.SetCommTimeouts(h,c.byref(original_timeouts))
        k.CloseHandle(h)
if __name__=='__main__':main()
