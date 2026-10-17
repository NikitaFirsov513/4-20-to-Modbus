"""Board acceptance without recalibrating. Read-only unless --exercise-settings.
Use --duration-seconds 86400 for an explicit 24-hour run; this tool does not
schedule/background a run. Do not disconnect a settings exercise before restoration.
"""
import argparse,json,time
from pathlib import Path
from rtu import SerialRTU
from diagnose_serial import crc16
BAUD=[1200,2400,4800,9600,19200,38400,57600,115200]
PARITY=[0,2,1] # firmware none/even/odd -> Win32 NOPARITY/EVENPARITY/ODDPARITY

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--port',default='COM6')
    ap.add_argument('--slave',type=int,default=1)
    ap.add_argument('--baud',type=int,default=9600)
    ap.add_argument('--parity',choices=['N','E','O'],default='N')
    ap.add_argument('--stop',type=int,choices=[1,2],default=1)
    ap.add_argument('--duration-seconds',type=float,default=10)
    ap.add_argument('--exercise-settings',action='store_true')
    ap.add_argument('--output',default='build/acceptance.json')
    args=ap.parse_args()
    if args.duration_seconds<=0:ap.error('duration must be positive')
    report={'started':time.strftime('%Y-%m-%dT%H:%M:%S%z'),'samples':0,'failures':0,'status_counts':{},'settings_cases':0}
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
    with SerialRTU(args.port,args.baud,{'N':0,'E':2,'O':1}[args.parity],args.stop) as port:
        slave=args.slave
        version=port.read(0x0200,3,slave)
        if version[:2]!=[1,1]:raise RuntimeError('Expected firmware 1.1.x; update firmware before acceptance')
        report['version']=version;report['calibration']=port.read(0x0030,2,slave)
        if report['calibration']!=[2331,39225]:raise RuntimeError('Unexpected calibration; test does not overwrite it')
        diag=port.read(0x0200,24,slave);report['initial_diagnostics']=diag
        # Protocol rejection checks use no valid writes and never change settings.
        tests=[(bytes([slave,0x7f]),bytes([slave,0xff,1])),
               (bytes([slave,3,1,0,0,0]),bytes([slave,0x83,3])),
               (bytes([slave,6,1,0,0,1]),bytes([slave,0x86,2])),
               (bytes([slave,6,0,2,0,8]),bytes([slave,0x86,3])),
               (bytes([slave,6]),bytes([slave,0x86,3]))]
        for request,expected in tests:
            response=port.exchange(request)
            if response[:3]!=expected:raise AssertionError((request.hex(),response.hex()))
        bad=bytes([slave,3,1,0,0,4,0,0])
        if port.exchange(bad,.25,raw=True):raise AssertionError('Bad CRC received a response')
        if port.exchange(bytes([0,3,1,0,0,4]),.25):raise AssertionError('Broadcast read received a response')
        foreign=slave%247+1
        if port.exchange(bytes([foreign,3,1,0,0,4]),.25):raise AssertionError('Foreign address received a response')
        original=port.read(0x0010,4,slave)
        original_prepared=port.read(0x0001,4,slave)
        if args.exercise_settings:
            if diag[4]&1:raise RuntimeError('SW1 service mode is active; turn it off and reboot before settings exercise')
            trial=original[:];current=original[:]
            try:
                for address in [1,126,247]:
                    for baud in range(8):
                        for parity in range(3):
                            for stop in [1,2]:
                                trial=[address,baud,parity,stop]
                                port.save(trial,slave) # ACK uses previous line parameters.
                                port.configure(BAUD[baud],PARITY[parity],stop);slave=address
                                current=trial[:]
                                time.sleep(.1)
                                if port.read(0x0010,4,slave)!=trial:raise AssertionError('Active settings mismatch')
                                report['settings_cases']+=1
                                print('PASS settings',trial,flush=True)
            finally:
                # Probe both old and new settings if an ACK was lost during switching.
                restored=False
                for candidate in [trial,current,original]:
                    try:
                        port.configure(BAUD[candidate[1]],PARITY[candidate[2]],candidate[3])
                        port.read(0x0010,4,candidate[0])
                        port.save(original,candidate[0]);restored=True;break
                    except (OSError,AssertionError):continue
                if not restored:raise RuntimeError('Could not restore settings: use SW1 ON + power cycle for 1/9600/8N1')
                port.configure(BAUD[original[1]],PARITY[original[2]],original[3]);slave=original[0]
                if port.read(0x0010,4,slave)!=original:raise AssertionError('Restoration verification failed')
                port.stage(original_prepared,slave)
        deadline=time.monotonic()+args.duration_seconds
        last_uptime=None;min_current=65535;max_current=0;max_age=0;next_print=0
        while time.monotonic()<deadline:
            try:
                adc=port.read(0x0100,4,slave)
                diag=port.read(0x0200,24,slave)
                uptime=(diag[8]<<16)|diag[9];age=(diag[10]<<16)|diag[11]
                if last_uptime is not None and uptime<last_uptime:raise AssertionError('Device restarted during polling')
                last_uptime=uptime;max_age=max(max_age,age)
                min_current=min(min_current,adc[0]);max_current=max(max_current,adc[0])
                key=f'0x{adc[3]:04x}';report['status_counts'][key]=report['status_counts'].get(key,0)+1
                if adc[3] & 0x30:raise AssertionError('Stale or failed ADC measurement')
                report['samples']+=1
                if time.monotonic()>next_print:
                    print(f'{report["samples"]} reads; {adc[0]/1000:.3f} mA; status={key}; age={age} ms; min_heap={diag[12]}; stacks={diag[14:17]}',flush=True)
                    next_print=time.monotonic()+5
            except (OSError,AssertionError) as error:
                report['failures']+=1
                report['last_error']=str(error)
            time.sleep(.1)
        report['final_diagnostics']=port.read(0x0200,24,slave)
        report['counters']=port.read(0x0220,26,slave)
        report.update(current_min_ua=min_current,current_max_ua=max_current,max_age_ms=max_age)
        report['passed']=report['samples']>0 and report['failures']==0
    output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2))
    if not report['passed']:raise SystemExit(1)
if __name__=='__main__':main()
