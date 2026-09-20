#!/bin/bash
# Launch FlightGear as the 3D physics harness & visualizer for flight-sitl
#
# Connects with Rust flight-sitl:
#   - Telemetry (FDM v24): FlightGear sends 60 Hz to 127.0.0.1:5500
#   - Controls (Ctrls v27): FlightGear receives 60 Hz on 127.0.0.1:5501

set -euo pipefail

AIRPORT="${1:-KSFO}"

echo "========================================================="
echo "🛩️  Starting FlightGear JSBSim Harness"
echo "   Aircraft:  Rascal110-JSBSim (official FlightGear package)"
echo "   Airport:   $AIRPORT"
echo "   FDM (out): 127.0.0.1:5500 (UDP)"
echo "   Ctrls (in): 127.0.0.1:5501 (UDP)"
echo "========================================================="

exec fgfs \
  --aircraft=Rascal110-JSBSim \
  --airport="$AIRPORT" \
  --native-fdm=socket,out,60,127.0.0.1,5500,udp \
  --native-ctrls=socket,in,60,,5501,udp \
  --disable-ai-traffic \
  --enable-hud \
  --timeofday=noon
