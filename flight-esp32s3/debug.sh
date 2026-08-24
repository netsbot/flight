#!/usr/bin/env bash
set -e

source ~/export-esp.sh

# 1. Kill any existing OpenOCD processes and release USB/JTAG locks
echo "Killing existing OpenOCD instances..."
killall -9 openocd 2>/dev/null || pkill -9 openocd 2>/dev/null || true
sleep 0.5

echo "Flashing via probe-rs..."
cargo espflash flash --chip esp32s3 --non-interactive --bin flight-esp32s3
# 4. Start fresh OpenOCD server in background
echo "Starting OpenOCD..."
openocd -f board/esp32s3-builtin.cfg &

# Give OpenOCD a brief moment to initialize the JTAG tap and bind port 3333
sleep 1