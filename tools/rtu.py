"""Windows Modbus RTU transport for board acceptance tools (standard library only)."""
import ctypes as c
from ctypes import wintypes as w
import time
from diagnose_serial import DCB, Timeouts, crc16

class SerialRTU:
    def __init__(self, port='COM6',baud=9600,parity=0,stop=1):
        self.k=c.WinDLL('kernel32',use_last_error=True)
        signatures={
            'CreateFileW':([w.LPCWSTR,w.DWORD,w.DWORD,c.c_void_p,w.DWORD,w.DWORD,w.HANDLE],w.HANDLE),
            'CloseHandle':([w.HANDLE],w.BOOL),
            'GetCommState':([w.HANDLE,c.POINTER(DCB)],w.BOOL),
            'SetCommState':([w.HANDLE,c.POINTER(DCB)],w.BOOL),
            'GetCommTimeouts':([w.HANDLE,c.POINTER(Timeouts)],w.BOOL),
            'SetCommTimeouts':([w.HANDLE,c.POINTER(Timeouts)],w.BOOL),
            'PurgeComm':([w.HANDLE,w.DWORD],w.BOOL),
            'ReadFile':([w.HANDLE,c.c_void_p,w.DWORD,c.POINTER(w.DWORD),c.c_void_p],w.BOOL),
            'WriteFile':([w.HANDLE,c.c_void_p,w.DWORD,c.POINTER(w.DWORD),c.c_void_p],w.BOOL),
            'FlushFileBuffers':([w.HANDLE],w.BOOL)}
        for name,(args,result) in signatures.items():
            f=getattr(self.k,name);f.argtypes=args;f.restype=result
        self.handle=self.k.CreateFileW('\\\\.\\'+port,0xc0000000,0,None,3,0,None)
        if self.handle==w.HANDLE(-1).value: raise c.WinError(c.get_last_error())
        self.original=DCB();self.original.length=c.sizeof(self.original)
        self.timeouts=Timeouts()
        try:
            self.check(self.k.GetCommState(self.handle,c.byref(self.original)))
            self.check(self.k.GetCommTimeouts(self.handle,c.byref(self.timeouts)))
            self.configure(baud,parity,stop)
            timeout=Timeouts(0xffffffff,0,20,0,2000)
            self.check(self.k.SetCommTimeouts(self.handle,c.byref(timeout)))
        except BaseException:
            self.k.CloseHandle(self.handle)
            raise
    @staticmethod
    def check(result):
        if not result: raise c.WinError(c.get_last_error())
    def configure(self,baud,parity,stop):
        dcb=DCB();dcb.length=c.sizeof(dcb)
        dcb.baud=baud;dcb.bytesize=8;dcb.parity=parity;dcb.stopbits=0 if stop==1 else 2
        dcb.flags=1 | (2 if parity else 0)
        self.check(self.k.SetCommState(self.handle,c.byref(dcb)))
        self.baud=baud
        time.sleep(.03)
    def exchange(self,payload,timeout=1.0,raw=False):
        tx=bytes(payload)
        if not raw: tx+=crc16(tx).to_bytes(2,'little')
        self.check(self.k.PurgeComm(self.handle,0x0c))
        n=w.DWORD()
        self.check(self.k.WriteFile(self.handle,tx,len(tx),c.byref(n),None))
        if n.value!=len(tx): raise IOError('Short serial write')
        self.check(self.k.FlushFileBuffers(self.handle))
        response=bytearray();deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            buffer=c.create_string_buffer(256)
            self.check(self.k.ReadFile(self.handle,buffer,256,c.byref(n),None))
            response+=buffer.raw[:n.value]
            expected=None
            if len(response)>=2 and response[1]&0x80: expected=5
            elif len(response)>=3 and response[1]==3: expected=5+response[2]
            elif len(response)>=2 and response[1] in (6,16): expected=8
            if expected and len(response)>=expected: break
        if response and (len(response)<5 or crc16(response[:-2])!=int.from_bytes(response[-2:],'little')):
            raise IOError('Bad response CRC: '+response.hex(' '))
        return bytes(response)
    def read(self,address,count,slave=1):
        p=bytes([slave,3])+address.to_bytes(2,'big')+count.to_bytes(2,'big')
        r=self.exchange(p,max(1.0,(count*2+5)*12/self.baud+.2))
        if not r or r[:3]!=bytes([slave,3,count*2]) or len(r)!=5+count*2:
            raise IOError('Read failed: '+r.hex(' '))
        return [int.from_bytes(r[i:i+2],'big') for i in range(3,len(r)-2,2)]
    def save(self,settings,slave=1):
        # settings: address, baud code, parity code, stop count. 0x10 includes SAVE=1.
        values=[1,*settings]
        p=bytes([slave,16,0,0,0,5,10])+b''.join(v.to_bytes(2,'big') for v in values)
        r=self.exchange(p)
        if len(r)!=8 or r[:6]!=bytes([slave,16,0,0,0,5]): raise IOError('Save failed: '+r.hex(' '))
    def stage(self,settings,slave=1):
        p=bytes([slave,16,0,1,0,4,8])+b''.join(v.to_bytes(2,'big') for v in settings)
        r=self.exchange(p)
        if len(r)!=8 or r[:6]!=bytes([slave,16,0,1,0,4]):raise IOError('Stage failed: '+r.hex(' '))
    def close(self):
        self.k.SetCommState(self.handle,c.byref(self.original))
        self.k.SetCommTimeouts(self.handle,c.byref(self.timeouts))
        self.k.CloseHandle(self.handle)
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
