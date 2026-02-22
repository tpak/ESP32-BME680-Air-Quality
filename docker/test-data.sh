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

  # Altitude from pressure (hypsometric formula)
  altitude=$(awk "BEGIN {printf \"%.2f\", 44330.0 * (1 - ($pressure / 1013.25) ^ 0.1903)}")

  # Sea level pressure corrected for test altitude of 300m
  sea_level_hpa=$(awk "BEGIN {printf \"%.2f\", $pressure / (1 - 300.0/44330.0) ^ 5.255}")

  # Voltage: gentle wiggle centered at 3.7V, amplitude 0.15
  voltage=$(awk "BEGIN {printf \"%.2f\", 3.70 + 0.15 * sin($STEP * 0.04)}")
  vraw=$(awk "BEGIN {printf \"%.0f\", ($voltage / 3.3) * 4096 / 2}")

  # IAQ: slow variation 25-175
  iaq=$(awk "BEGIN {printf \"%.2f\", 100 + 75 * sin($STEP * 0.015)}")
  iaq_accuracy=$(( (STEP / 50) % 4 ))
  static_iaq=$(awk "BEGIN {printf \"%.2f\", 105 + 70 * sin($STEP * 0.012)}")

  # CO2: slow variation 400-1200 ppm
  co2=$(awk "BEGIN {printf \"%.2f\", 800 + 400 * sin($STEP * 0.01)}")

  # Breath VOC: slow variation 0.5-5.0 ppm
  breath_voc=$(awk "BEGIN {printf \"%.2f\", 2.75 + 2.25 * sin($STEP * 0.018)}")

  # Compensated readings (temp offset -2C, humidity adjusted)
  comp_temp=$(awk "BEGIN {printf \"%.2f\", $temp_c - 2.0}")
  comp_rh=$(awk "BEGIN {printf \"%.2f\", $humidity + 3.0}")

  line="bme680,location=test Temp=${temp_c},TempF=${temp_f},hPa=${pressure},RH=${humidity},VOCOhms=${gas},Altitude=${altitude},SeaLevelHPa=${sea_level_hpa},Vraw=${vraw},Voltage=${voltage},IAQ=${iaq},IAQAccuracy=${iaq_accuracy},StaticIAQ=${static_iaq},CO2=${co2},BreathVOC=${breath_voc},CompTemp=${comp_temp},CompRH=${comp_rh}"

  echo "$line" | nc -u -w0 "$HOST" "$PORT"
  echo "[$STEP] T=${temp_c}C/${temp_f}F  IAQ=${iaq}  CO2=${co2}  VOC=${breath_voc}"

  STEP=$((STEP + 1))
  sleep "$DELAY"
done
