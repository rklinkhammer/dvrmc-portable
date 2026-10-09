#!/usr/bin/env bash
# Install from Microchip's extracted, complete 2026.1 offline bundle.
set -euo pipefail
if [[ $# != 1 ]]; then echo "Usage: $0 OFFLINE_INSTALLER_DIRECTORY" >&2; exit 2; fi
installer=$(realpath "$1")
image=${MICROCHIP_IMAGE:-dvrmc-microchip:2026.1-ubuntu24.04}
volume=${MICROCHIP_VOLUME:-dvrmc-microchip-2026.1}
[[ -f $installer/Libero_SoC_2026.1_offline_lin.sh ]]
if docker volume inspect "$volume" >/dev/null 2>&1; then
    echo "Volume $volume already exists; choose a new MICROCHIP_VOLUME for a reinstall." >&2
    exit 1
fi
docker volume create "$volume" >/dev/null
# TMPDIR and destination share the volume to avoid cross-device rename errors.
# Offline archive paths are relative to the installer's working directory.
exec docker run --rm --init \
    --mount "type=volume,src=$volume,dst=/opt" \
    --mount "type=bind,src=$installer,dst=/installer,readonly" \
    -w /installer "$image" bash -c '
        mkdir -p /opt/installer-tmp
        export TMPDIR=/opt/installer-tmp
        ./Libero_SoC_2026.1_offline_lin.sh --root /opt/microchip \
            --accept-licenses --accept-messages --confirm-command --no-shortcuts \
            install Libero_SoC PFSoC_MSS_Configurator Program_Debug SmartHLS
    '
