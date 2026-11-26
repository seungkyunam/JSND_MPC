#!/usr/bin/env bash
# Regenerates install_mpc_pitwall.sh with the current mpc_pitwall + rviz embedded.
set -e; cd "$(dirname "$(readlink -f "$0")")"
{
cat <<'HEAD'
#!/usr/bin/env bash
# Laptop installer for JSND_MPC pitwall. Run ON THE LAPTOP:
#   ssh jsnd@<JETSON_IP> 'cat JSND_MPC/pitwall/laptop/install_mpc_pitwall.sh' | bash
# Read-only diagnostics + copies 3 files into $HOME. No sudo, no ROS/system changes,
# does not touch ~/.bashrc or the existing JSND cyclonedds-laptop.xml.
set -eo pipefail
JETSON_IP=${JETSON_IP:-__JETSON_IP__}
ok(){ echo "  [OK]   $*"; }; warn(){ echo "  [WARN] $*"; }; bad(){ echo "  [FAIL] $*"; }
echo "== 1. laptop"
. /etc/os-release; echo "  $PRETTY_NAME / $(uname -m) / host $(hostname)"
[[ -n ${DISPLAY:-}${WAYLAND_DISPLAY:-} ]] && ok "display ${DISPLAY:-$WAYLAND_DISPLAY}" || warn "no DISPLAY here (fine over SSH; run mpc_pitwall from the desktop)"
echo "== 2. network to Jetson $JETSON_IP"
netif=$(ip -4 route get "$JETSON_IP" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="dev"){print $(i+1);exit}}')
[[ -n $netif ]] && ok "interface $netif ($(ip -4 -br addr show "$netif" | awk '{print $3}'))" || { bad "no route to $JETSON_IP"; exit 1; }
ping -c 2 -W 2 "$JETSON_IP" >/dev/null && ok "ping" || warn "ping failed"
echo "== 3. ROS"
CONDA_SH=$HOME/miniforge3/etc/profile.d/conda.sh; mode=none
if [[ -r $CONDA_SH ]] && [[ -x $HOME/miniforge3/envs/msnd_pitwall/bin/rviz2 ]]; then
  ok "conda env msnd_pitwall with rviz2 (Jazzy)"; mode=jazzy
  ls "$HOME"/miniforge3/envs/msnd_pitwall/lib/librmw_cyclonedds_cpp.so >/dev/null 2>&1 && ok "rmw_cyclonedds (conda)" || { bad "rmw_cyclonedds_cpp missing in msnd_pitwall"; mode=none; }
else
  warn "conda env msnd_pitwall/rviz2 not found"
fi
if [[ $mode == none ]]; then
  if [[ -r /opt/ros/humble/lib/librmw_cyclonedds_cpp.so && -x /opt/ros/humble/lib/rviz2/rviz2 ]]; then
    ok "fallback: system Humble rviz2 + rmw_cyclonedds"; mode=humble
  else
    bad "neither Conda Jazzy nor Humble rmw_cyclonedds_cpp available"
    echo "        Humble: sudo apt install ros-humble-rmw-cyclonedds-cpp ; then re-run this installer"; exit 1
  fi
fi
echo "== 4. install"
mkdir -p "$HOME/.local/bin" "$HOME/.config/msnd"
put(){ # put <dest> <mode>  (stdin = content), keeps a backup if different
  local tmp; tmp=$(mktemp); cat > "$tmp"
  if [[ -f $1 ]] && ! cmp -s "$tmp" "$1"; then cp -p "$1" "$1.bak.$(date +%Y%m%d_%H%M%S)"; fi
  install -m "$2" "$tmp" "$1"; rm -f "$tmp"; ok "$1"
}
base64 -d <<'B64' | put "$HOME/.local/bin/mpc_pitwall" 755
__MPC_PITWALL__
B64
base64 -d <<'B64' | put "$HOME/.config/msnd/mpc_pitwall.rviz" 644
__RVIZ__
B64
put "$HOME/.config/msnd/cyclonedds-mpc-pitwall.xml" 644 <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!-- mpc_pitwall (domain 174) only. Interface = route to Jetson $JETSON_IP. -->
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General>
      <Interfaces><NetworkInterface name="$netif" multicast="true"/></Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <Discovery><ParticipantIndex>none</ParticipantIndex></Discovery>
  </Domain>
</CycloneDDS>
EOF
if [[ $mode == humble ]]; then
  printf 'MPC_PITWALL_HUMBLE=1\n' | put "$HOME/.config/msnd/mpc_pitwall.env" 644
else
  rm -f "$HOME/.config/msnd/mpc_pitwall.env"
fi
echo "== 5. done (mode: $mode)"
case ":$PATH:" in *":$HOME/.local/bin:"*) echo "  run: mpc_pitwall check   /   mpc_pitwall";;
  *) echo "  ~/.local/bin is not on PATH in this shell; run: ~/.local/bin/mpc_pitwall";; esac
HEAD
} > install_mpc_pitwall.sh.tmp
jip=$(ip -4 -br addr show wlP1p1s0 | awk '{print $3}' | cut -d/ -f1)
python3 - "$jip" <<'EOF'
import base64, sys
s = open('install_mpc_pitwall.sh.tmp').read()
b = lambda f: base64.encodebytes(open(f, 'rb').read()).decode().rstrip()
s = s.replace('__JETSON_IP__', sys.argv[1]).replace('__MPC_PITWALL__', b('mpc_pitwall')).replace('__RVIZ__', b('mpc_pitwall.rviz'))
open('install_mpc_pitwall.sh', 'w').write(s)
EOF
rm install_mpc_pitwall.sh.tmp; chmod 755 install_mpc_pitwall.sh; echo "generated (Jetson IP $jip)"
