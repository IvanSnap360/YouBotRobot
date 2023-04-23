#include "config.h"

double battery[BATTERIES_COUNT][CELLS_COUNT];

#define getVoltsFromPin(pin) (((analogRead(pin) * SOURCE_VOLTAGE_VOLTS) / ADC_MAX_VALUE) / (R2 / (R1 + R2)))

uint32_t send_lt = 0;

void print_values();

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RELAY_CONTROL_PIN, OUTPUT);

    digitalWrite(RELAY_CONTROL_PIN, OFF);
    while (!Serial)
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }

    digitalWrite(RELAY_CONTROL_PIN, ON);
}

void loop()
{
    if (millis() - send_lt > (uint32_t)(1000.0 / SERIAL_SEND_RATE_HZ))
    {
        battery[BATTERY_1][CELL_1] = getVoltsFromPin(battery_1_cell_pins[CELL_1]);
        battery[BATTERY_1][CELL_2] = getVoltsFromPin(battery_1_cell_pins[CELL_2]) - battery[BATTERY_1][CELL_2];
        battery[BATTERY_1][CELL_3] = getVoltsFromPin(battery_1_cell_pins[CELL_3]) - (battery[BATTERY_1][CELL_2] + battery[BATTERY_1][CELL_3]);

        battery[BATTERY_2][CELL_1] = getVoltsFromPin(battery_2_cell_pins[CELL_1]);
        battery[BATTERY_2][CELL_2] = getVoltsFromPin(battery_2_cell_pins[CELL_2]) - battery[BATTERY_2][CELL_2];
        battery[BATTERY_2][CELL_3] = getVoltsFromPin(battery_2_cell_pins[CELL_3]) - (battery[BATTERY_2][CELL_2] + battery[BATTERY_2][CELL_3]);

        send_lt = millis();
    }
}

void print_values()
{
    char buffer[300];

    sprintf(buffer, "__%lf__%lf__%lf__;__%lf__%lf__%lf__",
            battery[BATTERY_1][CELL_1],
            battery[BATTERY_1][CELL_2],
            battery[BATTERY_1][CELL_3],
            /*                         */
            battery[BATTERY_2][CELL_1],
            battery[BATTERY_2][CELL_2],
            battery[BATTERY_2][CELL_3]
            );

    Serial.println(buffer);
    memset(buffer,0,sizeof(buffer));
}