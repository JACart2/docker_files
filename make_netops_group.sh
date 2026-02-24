#!/bin/bash
# Creates the netops group and grants it password;ess access to specific 
# ip and xhost commands without requiring sudo in the user's she;; session.
# must be run as root (or via sudo)

set -euo pipefail

# -- Config --
GROUP_NAME="netops"
SUDOERS_FILE="/etc/sudoers.d/netops"
IP_BIN="$(command -v ip)"	# resolves to e.g. /sbin/ip or /usr/sbin/ip
XHOST_BIN="$(command -v xhost)"	# resolves to e.g. /usr/bin/xhost

#-- sanity checks --
if [[ $EUID -ne 0 ]]; then
	echo "ERR: this script must be run as root." >&2
	exit 1
fi


