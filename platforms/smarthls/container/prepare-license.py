#!/usr/bin/env python3
"""Prepare a private runtime copy without changing signed features or host ID."""
import os, pathlib, re, socket, sys
if len(sys.argv) not in (3, 4):
    raise SystemExit('Usage: prepare-license.py INPUT_LICENSE OUTPUT_LICENSE [DAEMON_DIRECTORY]')
source=pathlib.Path(sys.argv[1]); output=pathlib.Path(sys.argv[2])
daemon_dir=pathlib.Path(sys.argv[3] if len(sys.argv)==4 else '/opt/microchip/LicenseDaemons').resolve()
for name in ('lmgrd', 'actlmgrd', 'saltd', 'snpslmd'):
    if not os.access(daemon_dir/name, os.X_OK):
        raise SystemExit(f'Missing executable: {daemon_dir/name}')
text=source.read_text()
server=re.search(r'^SERVER\s+(\S+)\s+([0-9a-fA-F]{12})\s+(\d+)\s*$',text,re.M)
if not server:
    raise SystemExit('Expected an Ethernet-bound SERVER line with explicit port')
macs={p.read_text().strip().replace(':','').lower() for p in pathlib.Path('/sys/class/net').glob('*/address')}
if server[2].lower() not in macs:
    raise SystemExit('License host ID does not match a real host interface; obtain the correct license')
text=text[:server.start()]+f'SERVER {socket.gethostname()} {server[2]} {server[3]}'+text[server.end():]
text=re.sub(r'^(VENDOR|DAEMON)\s+(actlmgrd|saltd|snpslmd)\s+PATH/\S+',
            lambda m:f'{m[1]} {m[2]} {daemon_dir/m[2]}',text,flags=re.M)
output.parent.mkdir(parents=True,exist_ok=True)
# Exclusive creation avoids replacing an existing license configuration.
with os.fdopen(os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), 'w') as f:
    f.write(text)
print('Prepared license runtime copy with verified host binding; feature signatures unchanged.')
