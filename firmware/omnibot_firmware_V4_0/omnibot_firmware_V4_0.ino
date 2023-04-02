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

#ifdef ENABLE_OUTPUT
    if (millis() - last_send_time > OUTPUT_SEND_PERIOD)
    {
#ifdef OUTPUT_ECODER_TIKS
        char encoder_buffer[200];
        sprintf(encoder_buffer, "[ENCODERS]  a1: %ld a2: %ld a3: %ld a4: %ld", 0, 0, 0, 0);
        Serial.println(encoder_buffer);
        memset(encoder_buffer, 0, sizeof(encoder_buffer));

#endif // OUTPUT_ECODER_TIKS

#ifdef OUTPUT_ACTUATORS_ANGULAR_VELOCITY
        char velocity_buffer[200];
        sprintf(velocity_buffer, "[VELOCITY]: a1: %.2lf a2: %.2lf a3: %.2lf a4: %.2lf",
                actuators_list[LEFT_FRONT /* */].getVelocity(),
                actuators_list[RIGHT_FRONT /**/].getVelocity(),
                actuators_list[LEFT_BACK /*  */].getVelocity(),
                actuators_list[RIGHT_BACK /* */].getVelocity());

        Serial.println(velocity_buffer);
        memset(velocity_buffer, 0, sizeof(velocity_buffer));
#endif // OUTPUT_ACTUATORS_ANGULAR_VELOCITY

#ifdef OUTPUT_PID_ANALAYZER
        static double target_value;
        if (Serial.available() > 1)
        {
            char incoming = Serial.read();
            float value = Serial.parseFloat();
            switch (incoming)
            {
            case 'p':
                actuators_list[ACTUATOR_NUMBER_FOR_PID_ANALAYZER].getConfig()->Kp = value;
                break;
            case 'i':
                actuators_list[ACTUATOR_NUMBER_FOR_PID_ANALAYZER].getConfig()->Ki = value;
                break;
            case 'd':
                actuators_list[ACTUATOR_NUMBER_FOR_PID_ANALAYZER].getConfig()->Kd = value;
                break;
            case 's':
                target_value = (double)value;
                actuators_list[ACTUATOR_NUMBER_FOR_PID_ANALAYZER].setVelocity(target_value);
                break;
            }
        }
        char pid_buffer[200];
        sprintf(pid_buffer, "TargetVelocity: %0.2f, CurrentVelocity: %0.2f",
                target_value,
                actuators_list[ACTUATOR_NUMBER_FOR_PID_ANALAYZER].getVelocity());
        Serial.println(pid_buffer);
        memset(pid_buffer, 0, sizeof(pid_buffer));
#endif

        last_send_time = millis();
    }
#endif // ENABLE_OUTPUT
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