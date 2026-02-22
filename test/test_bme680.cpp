#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "bme680_functions.h"

static int tests_run = 0;
static int tests_passed = 0;

#define RUN_TEST(name, expr) do { \
    tests_run++; \
    if (expr) { \
      tests_passed++; \
      printf("  PASS: %s\n", name); \
    } else { \
      printf("  FAIL: %s\n", name); \
    } \
  } while(0)

#define APPROX(a, b) (fabs((a) - (b)) < 0.01f)

int main() {
  printf("=== Temperature: celsiusToFahrenheit ===\n");
  RUN_TEST("freezing 0C -> 32F",    APPROX(celsiusToFahrenheit(0.0f), 32.0f));
  RUN_TEST("boiling 100C -> 212F",  APPROX(celsiusToFahrenheit(100.0f), 212.0f));
  RUN_TEST("body temp 37C -> 98.6F", APPROX(celsiusToFahrenheit(37.0f), 98.6f));
  RUN_TEST("negative -40C -> -40F", APPROX(celsiusToFahrenheit(-40.0f), -40.0f));

  printf("\n=== Voltage: adcToVoltage ===\n");
  // Formula: (raw * 2.0 / 4096.0) * 3.3 — accounts for voltage divider on battery pin
  RUN_TEST("zero ADC -> 0V",        APPROX(adcToVoltage(0), 0.0f));
  RUN_TEST("max ADC 4096 -> 6.6V",  APPROX(adcToVoltage(4096), 6.6f));
  RUN_TEST("mid ADC 2048 -> 3.3V",  APPROX(adcToVoltage(2048), 3.3f));
  RUN_TEST("typical LiPo ~2296 -> ~3.69V", APPROX(adcToVoltage(2296), 3.69f));

  printf("\n=== Altitude: pressureToAltitude ===\n");
  RUN_TEST("sea level 1013.25 -> 1.0", APPROX(pressureToAltitude(1013.25f), 1.0f));
  RUN_TEST("half pressure -> 0.5",     APPROX(pressureToAltitude(1013.25f / 2.0f), 0.5f));
  RUN_TEST("zero pressure -> 0.0",     APPROX(pressureToAltitude(0.0f), 0.0f));

  printf("\n=== Line protocol: buildUdpPayload ===\n");
  String payload = buildUdpPayload(22.5f, 72.5f, 1013.25f, 45.0f, 80000.0f, 1.0f, 2048, 1.65f);
  const char* s = payload.c_str();

  RUN_TEST("starts with measurement",  strncmp(s, "bme680,location=363Office ", 26) == 0);
  RUN_TEST("contains Temp=",           strstr(s, "Temp=22.50") != NULL);
  RUN_TEST("contains TempF=",          strstr(s, "TempF=72.50") != NULL);
  RUN_TEST("contains hPa=",            strstr(s, "hPa=1013.25") != NULL);
  RUN_TEST("contains RH=",             strstr(s, "RH=45.00") != NULL);
  RUN_TEST("contains VOCOhms=",        strstr(s, "VOCOhms=80000.00") != NULL);
  RUN_TEST("contains Altitude=",       strstr(s, "Altitude=1.00") != NULL);
  RUN_TEST("contains Vraw=",           strstr(s, "Vraw=2048") != NULL);
  RUN_TEST("contains Voltage=",        strstr(s, "Voltage=1.65") != NULL);
  RUN_TEST("no Altitide typo",         strstr(s, "Altitide") == NULL);

  printf("\n%d/%d tests passed\n", tests_passed, tests_run);
  return (tests_passed == tests_run) ? 0 : 1;
}
