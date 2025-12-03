#include "HX711.h"

#define BAUD_RATE 115200
#define LOADCELL_DT_PIN 2
#define LOADCELL_SCK_PIN 3
#define CHANNEL_A 128
#define CHANNEL_B 32
#define SAMPLING_INTERVAL 5
#define TIMEOUT_MS 100

HX711 loadcell;

long offset_a;
long offset_b;

void setup() {
	Serial.begin(BAUD_RATE);
	loadcell.begin(LOADCELL_DT_PIN, LOADCELL_SCK_PIN);
	loadcell.set_gain(CHANNEL_A);
	offset_a = loadcell.get_units(SAMPLING_INTERVAL);
	loadcell.set_gain(CHANNEL_B);
	offset_b = loadcell.get_units(SAMPLING_INTERVAL);
	Serial.print("OFFSETS:");
	Serial.print(offset_a);
	Serial.print(',');
	Serial.println(offset_b);
}

void loop() {
	if (loadcell.wait_ready_timeout(TIMEOUT_MS)) {
		long reading;
		loadcell.set_gain(CHANNEL_A);
		reading = (long)loadcell.get_units(SAMPLING_INTERVAL) - offset_a;
		Serial.print(reading);
		Serial.print(',');
		loadcell.set_gain(CHANNEL_B);
		reading = (long)loadcell.get_units(SAMPLING_INTERVAL) - offset_b;
		Serial.println(reading);
	} else {
		Serial.println("HX711 not ready or not found.");
	}
}
