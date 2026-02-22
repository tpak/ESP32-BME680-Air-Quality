#!/bin/bash
# Send fake BME680 data to VictoriaMetrics at a configurable interval.
# Temp walks up and down in a sine wave, pressure drifts slowly,
# and voltage wiggles gently.
#
# Usage: ./test-data.sh [delay]
#   delay  seconds between sends (default: 3)

HOST="localhost"
PORT="8089"
DELAY="${1:-3}"
STEP=0

echo "Sending test data to $HOST:$PORT every ${DELAY}s (Ctrl-C to stop)"

while true; do
  # Temp: sine wave centered at 22 C, amplitude 5
  temp_c=$(awk "BEGIN {printf \"%.2f\", 22 + 5 * sin($STEP * 0.05)}")
  temp_f=$(awk "BEGIN {printf \"%.2f\", $temp_c * 9/5 + 32}")

  # Pressure: very slow drift centered at 1013.25, amplitude 2
  pressure=$(awk "BEGIN {printf \"%.2f\", 1013.25 + 2 * sin($STEP * 0.008)}")

  # Humidity: slow wobble centered at 45%
  humidity=$(awk "BEGIN {printf \"%.2f\", 45 + 8 * sin($STEP * 0.03)}")

  # Gas resistance: slow variation
  gas=$(awk "BEGIN {printf \"%.0f\", 80000 + 15000 * sin($STEP * 0.02)}")

  # Altitude from pressure
  altitude=$(awk "BEGIN {printf \"%.4f\", $pressure / 1013.25}")

  # Voltage: gentle wiggle centered at 3.7V, amplitude 0.15
  voltage=$(awk "BEGIN {printf \"%.2f\", 3.70 + 0.15 * sin($STEP * 0.04)}")
  vraw=$(awk "BEGIN {printf \"%.0f\", ($voltage / 3.3) * 4096 / 2}")

  line="bme680,location=test Temp=${temp_c},TempF=${temp_f},hPa=${pressure},RH=${humidity},VOCOhms=${gas},Altitude=${altitude},Vraw=${vraw},Voltage=${voltage}"

  echo "$line" | nc -u -w0 "$HOST" "$PORT"
  echo "[$STEP] T=${temp_c}C/${temp_f}F  P=${pressure}  V=${voltage}"

  STEP=$((STEP + 1))
  sleep "$DELAY"
done
