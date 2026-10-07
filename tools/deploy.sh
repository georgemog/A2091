#!/bin/sh
# Deploy an A2091 test build to the MiSTer and start it.
#
#   deploy.sh <rbf> <MiSTer-binary> [cfg]
#
# The MiSTer main binary is installed as /media/fat/MiSTer_A2091 and started
# in place of whatever is running (the stock /media/fat/MiSTer is untouched;
# a reboot brings it back). The core is copied to _Computer/ under its own name.
set -e
MISTER=${MISTER:-root@mister}
RBF=$1
BIN=$2
CFG=$3
[ -f "$RBF" ] && [ -f "$BIN" ] || { echo "usage: $0 <rbf> <MiSTer-binary> [cfg]"; exit 1; }

NAME=$(basename "$RBF")
scp -q "$RBF" "$MISTER:/media/fat/_Computer/$NAME"
scp -q "$BIN" "$MISTER:/media/fat/MiSTer_A2091.new"
[ -n "$CFG" ] && scp -q "$CFG" "$MISTER:/media/fat/config/Minimig.cfg"

ssh "$MISTER" "
	killall MiSTer MiSTer_BigMig MiSTer_A2091 2>/dev/null; sleep 1
	mv /media/fat/MiSTer_A2091.new /media/fat/MiSTer_A2091; chmod +x /media/fat/MiSTer_A2091
	cd /media/fat && (nohup /media/fat/MiSTer_A2091 /media/fat/_Computer/$NAME >/tmp/mister_a2091.out 2>&1 &)
	sleep 8; tr '\0' ' ' < /proc/\$(pidof MiSTer_A2091)/cmdline; echo
"
