# Libero / SmartHLS 2026.1 on Ubuntu 24.04

Run commands from the repository root. Docker must already work for your user.
The host tested here is Ubuntu 26.04; the tools run in Ubuntu 24.04 user space.
This is a tested local configuration, not a claim of vendor certification of Docker.

## Installed configuration

- Image: `dvrmc-microchip:2026.1-ubuntu24.04`.
- Persistent installation: volume `dvrmc-microchip-2026.1`, mounted at `/opt`.
- Libero, SmartHLS, MSS Configurator and Program/Debug: 2026.1.0.17.
- MegaVault and remote programming server were not selected.
- `run.sh` mounts the installation read-only and runs as the calling user.
- Builds and private runtime files live under the repository's ignored `work/`
  and `platforms/smarthls/hls_output/` directories.

The fresh installer exited zero, reported prerequisite status zero and installed
its maintenance script successfully. Libero Tcl startup, controller SmartHLS
RTL co-simulation and a Libero/Synplify counter synthesis smoke test passed.
Full controller FPGA synthesis and timing closure remain outstanding. The earlier host installation reported
unsupported Ubuntu and cross-device copy errors. It remains untouched.

## Rebuild or install elsewhere

```
docker build -t dvrmc-microchip:2026.1-ubuntu24.04 platforms/smarthls/container
platforms/smarthls/container/install.sh /path/to/Libero_SoC_2026.1_offline_lin
```

The installer directory must contain the shell/binary installer and all offline
archives. The script accepts the vendor license agreements and installs into a
new named volume. It refuses an existing volume. Set `MICROCHIP_VOLUME` to a new
name for another installation. Extraction needs tens of GB of free space.
The dependency list comes from Microchip's Ubuntu 24.04 prerequisite file;
`libgcc-s1` replaces the obsolete `libgcc1` name.

## Licensing

A license must match its licensed host. Do not copy a Mac-bound license to a
Linux host. A reachable floating license server can be used if its license
permits these clients. Libero/SmartHLS, Synplify and Questa use different vendor
daemons. Set all three client variables below.

On this machine the issued `License-1.dat` matches a real Linux host interface.
A private runtime copy resolves its hostname and daemon-path placeholders without
altering host binding or signed features. The license server runs on the host:
Synplify's `snpslmd` failed to open its lock file in Docker, but started normally
on the host. No license server is automatically started by `run.sh`.

The standalone daemon binaries have been copied from the fresh installation to
`~/.local/opt/microchip/2026.1/LicenseDaemons`. To reproduce that copy on another
host, create a temporary container with the installation volume and use
`docker cp` on `/opt/microchip/LicenseDaemons`.

Prepare a private license copy once (the helper refuses to overwrite a file):

```
platforms/smarthls/container/prepare-license.py ~/License-1.dat \
  work/microchip-validation/host-license.dat \
  "$HOME/.local/opt/microchip/2026.1/LicenseDaemons"
```

Start the server in a separate terminal; leave it running during builds:

```
"$HOME/.local/opt/microchip/2026.1/LicenseDaemons/lmgrd" -z \
  -c "$PWD/work/microchip-validation/host-license.dat" \
  -l "$PWD/work/microchip-validation/host-license.log"
```

In the build terminal:

```
export MICROCHIP_HOST_NETWORK=1
export LM_LICENSE_FILE=1702@localhost
export SNPSLMD_LICENSE_FILE=1702@localhost
export SALT_LICENSE_SERVER=1702@localhost
```

For a remote server, use its hostname and port and omit host networking if it
is unnecessary. Its vendor-daemon ports must also be reachable.
Keep issued licenses and runtime copies out of version control.

## Controller validation

```
platforms/smarthls/container/run.sh bash
cd platforms/smarthls
shls init                              # once in a fresh checkout
cp config.tcl.example config.tcl        # representative MPFS250T, 100 MHz
shls -a --user_makefile makefile.user sw
shls -a --user_makefile makefile.user hw
shls -a --user_makefile makefile.user cosim
xvfb-run -a shls -a --user_makefile makefile.user rtl_synth
```

Choose the actual board part before implementation. SmartHLS 2026.1 requires
explicit `--user_makefile`; its generated Makefile alone has an empty source
list. Use `cosim` for this custom function top, not `sim`. The top's software
clear input is named `clear` to avoid colliding with SmartHLS's hardware `reset`.

`MICROCHIP_VOLUME=host` explicitly selects the original `/opt/microchip` bind
mount for diagnostics. Normal runs use the separate volume and do not need it.
USB/JTAG access, interactive desktop display and physical programming were not
validated by this batch workflow.
