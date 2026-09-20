#!/usr/bin/env python3
"""Exercise compiled SDK worker in --mock mode; no DDS or robot calls."""
import json,os,signal,socket,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
ENV={'HOME':str(Path.home()),'PATH':'/usr/bin:/bin','LD_LIBRARY_PATH':str(Path.home()/'unitree_sdk2-2.0.2/thirdparty/lib/aarch64')}

def trial(label,payload,expected_move):
    path=f'/tmp/elf-nav-mock-{os.getuid()}/nav.sock';log=ROOT/f'log/sdk_{label}.jsonl'
    with log.open('w') as output:
        worker=subprocess.Popen([str(ROOT/'sdk/build/nav_worker'),'eth0','--mock'],env=ENV,stdout=output,stderr=subprocess.STDOUT)
        sock=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
        try:
            end=time.monotonic()+5
            while not Path(path).exists() and time.monotonic()<end:
                assert worker.poll() is None;time.sleep(.02)
            sock.sendto(payload(time.monotonic()).encode(),path);time.sleep(.15)
            # No further commands: independent worker must stop after 250 ms.
            time.sleep(.35)
            # Even new valid commands cannot resume after timeout while moving.
            sock.sendto(f'2 {time.monotonic():.9f} 0.1 0.0 1\n'.encode(),path);time.sleep(.1)
        finally:
            worker.send_signal(signal.SIGINT);worker.wait(timeout=5);sock.close()
    entries=[json.loads(line) for line in log.read_text().splitlines() if line.startswith('{')]
    moves=[i for i,e in enumerate(entries) if e['event']=='move'];assert bool(moves)==expected_move,(label,entries)
    if moves:
        following=entries[moves[-1]+1:];assert any(e['event']=='stop' for e in following)
        assert len(moves)<=13,'Stale command was repeated beyond expiry'
    assert entries[-1]['event']=='stop'
    return {'case':label,'passed':True,'move_calls':len(moves),'mock':True}

report=[trial('timeout',lambda now:f'1 {now:.9f} 0.1 0.0 1\n',True),trial('nonfinite',lambda now:f'1 {now:.9f} nan 0.0 1\n',False),trial('overspeed',lambda now:f'1 {now:.9f} 2.0 0.0 1\n',False)]
(ROOT/'log/sdk_smoke.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
