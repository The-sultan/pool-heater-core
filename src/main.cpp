#include <Arduino.h>
#include "PoolHeaterCore.h"

// --- SELECCIÓN DINÁMICA DE HARDWARE ---
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
#include "ESP32HeaterDriver.h"

// Instanciamos el obrero del ESP32 (usando pines de ejemplo)
ESP32HeaterDriver hardwareDriver(18, 19);

#elif defined(ESP8266) || defined(ARDUINO_ARCH_ESP8266)
#include "ESP8266HeaterDriver.h"
// Instanciamos el obrero del D1 Mini con tus pines originales
ESP8266HeaterDriver hardwareDriver(D7, D2);

#else
#error "Arquitectura de hardware no soportada por esta librería."
#endif

// --- INYECCIÓN DE DEPENDENCIAS ---
// Le pasamos el obrero (sea cual sea) al cerebro
PoolHeaterCore heater(&hardwareDriver);

void setup() {
Serial.begin(115200);
delay(1000);

}

void loop() {
// El cerebro procesa en silencio.
// Solo devuelve 'true' si detectó un cambio real.
if (heater.loop()) {

}
}