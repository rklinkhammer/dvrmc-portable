#!/usr/bin/env bash
set -euo pipefail
repo=$(cd "$(dirname "$0")/../../.." && pwd)
image=${MICROCHIP_IMAGE:-dvrmc-microchip:2026.1-ubuntu24.04}
mkdir -p "$repo/work/microchip-home"
args=(--rm --init --user "$(id -u):$(id -g)"
      --mount "type=bind,src=$repo,dst=/workspace"
      --mount "type=bind,src=$repo/work/microchip-home,dst=/home/tool"
      -e HOME=/home/tool -w /workspace)
volume=${MICROCHIP_VOLUME:-dvrmc-microchip-2026.1}
if [[ $volume != host ]]; then
    # Fail on a typo instead of letting Docker create an empty volume.
    docker volume inspect "$volume" >/dev/null
    args+=(--mount "type=volume,src=$volume,dst=/opt,readonly")
else
    args+=(--mount 'type=bind,src=/opt/microchip,dst=/opt/microchip,readonly')
fi
# Host networking is opt-in for a local floating license server. It also allows
# clients to reach a license server on the Linux host via localhost.
if [[ ${MICROCHIP_HOST_NETWORK:-0} == 1 ]]; then args+=(--network host); fi
for name in LM_LICENSE_FILE SNPSLMD_LICENSE_FILE MGLS_LICENSE_FILE ACTEL_LICENSE_FILE SALT_LICENSE_SERVER; do
    if [[ -n ${!name:-} ]]; then args+=(-e "$name"); fi
done
if [[ -t 0 && -t 1 ]]; then args+=(-it); fi
if [[ $# == 0 ]]; then set -- bash; fi
exec docker run "${args[@]}" "$image" "$@"
