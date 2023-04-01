#include "config.h"
#include "actuator.h"

void setup()
{
    for (int i = 0; i < ACTUATORS_COUNT; i++)
    {
        actuators_list[i].setConfig(&_actuator_config[i]);
        attachInterrupt(digitalPinToInterrupt(actuators_list[i].getConfig()->encoder_pin_A),
                        (*functptr_enc_A[i]),
                        actuators_list[i].getConfig()->encoder_work_mode);

        attachInterrupt(digitalPinToInterrupt(actuators_list[i].getConfig()->encoder_pin_B),
                        (*functptr_enc_B[i]),
                        actuators_list[i].getConfig()->encoder_work_mode);

        actuators_list[i].setVelocity(0.0);
    }

#ifdef WORK_MODE__SERIAL
    Serial.begin(COM_SERIAL__BAUDRATE);

    while (!Serial)
    {
        if (millis() - blink_last_time > (uint32_t)(1000.0 / DISCONNECTION_LED_BLINK_RATE_HZ))
        {
            toggleLED();
            blink_last_time = millis();
        }
    }
#endif // WORK_MODE__SERIAL
}

void loop_f_in_serial_mode()
{
}
void loop_f_in_ros_mode()
{
}

void loop()
{
    for (auto &actuator : actuators_list)
        actuator.tick();

#ifdef WORK_MODE__SERIAL
    if (Serial)
    {
        loop_f_in_serial_mode();
        if (millis() - blink_last_time > (uint32_t)(1000.0 / CONNECTION_LED_BLINK_RATE_HZ))
        {
            toggleLED();
            blink_last_time = millis();
        }
    }

#endif
}