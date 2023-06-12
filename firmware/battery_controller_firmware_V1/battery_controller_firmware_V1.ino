#include "config.h"

float battery[BATTERIES_COUNT][CELLS_COUNT];

uint32_t send_lt = 0;

void print_values();
float getVoltsFromPin(int pin);
void setup()
{
  Serial.begin(SERIAL_BAUDRATE); // инициаллизация последовательного порта
  while (!Serial); // ждем открытия порта 


}

void loop()
{
  if (millis() - send_lt > (uint32_t)(1000.0 / SERIAL_SEND_RATE_HZ))
  {
    battery[BATTERY_1][CELL_1] = getVoltsFromPin(battery_1_cell_pins[CELL_1]);
    battery[BATTERY_1][CELL_2] = getVoltsFromPin(battery_1_cell_pins[CELL_2]) - battery[BATTERY_1][CELL_1];
    battery[BATTERY_1][CELL_3] = getVoltsFromPin(battery_1_cell_pins[CELL_3]) - (battery[BATTERY_1][CELL_1] + battery[BATTERY_1][CELL_2]);

    battery[BATTERY_2][CELL_1] = getVoltsFromPin(battery_2_cell_pins[CELL_1]);
    battery[BATTERY_2][CELL_2] = getVoltsFromPin(battery_2_cell_pins[CELL_2]) - battery[BATTERY_2][CELL_1];
    battery[BATTERY_2][CELL_3] = getVoltsFromPin(battery_2_cell_pins[CELL_3]) - (battery[BATTERY_2][CELL_1] + battery[BATTERY_2][CELL_2]);
    print_values();
    send_lt = millis();
  }

  if (Serial.available() > 0)
  {
    int status = Serial.parseInt();
    if (status > 0)
      status = 1;
    else
      status = 0;
    digitalWrite(RELAY_CONTROL_PIN, status);
  }
}

void print_values()
{
  Serial.print("__");
  Serial.print( battery[BATTERY_1][CELL_1]);
  Serial.print("__");
  Serial.print( battery[BATTERY_1][CELL_2]);
  Serial.print("__");
  Serial.print( battery[BATTERY_1][CELL_3]);
  Serial.print("__;__");
  Serial.print( battery[BATTERY_2][CELL_1]);
  Serial.print("__");
  Serial.print( battery[BATTERY_2][CELL_2]);
  Serial.print("__");
  Serial.print( battery[BATTERY_2][CELL_3]);
  Serial.println("__");
}

float getVoltsFromPin(int pin)
{
  float vin = 0.0;
  vin = ((analogRead(pin) * SOURCE_VOLTAGE_VOLTS) / ADC_MAX_VALUE) / (R2 / (R1 + R2));
  if (vin < 0.3)
    vin = 0.0;

  return abs(vin);
}
