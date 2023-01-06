#include "omnibot_platform_drv_lib/omnibot_platform_drv_lib.h"

OMNIBOT_PLATFORM_DRV_LIB::OMNIBOT_PLATFORM_DRV_LIB()
{
    
}

OMNIBOT_PLATFORM_DRV_LIB::~OMNIBOT_PLATFORM_DRV_LIB()
{
}

void OMNIBOT_PLATFORM_DRV_LIB::setConfig(omnibot_platform_drv_cfg_t* config)
{
    _config = config;
}

void OMNIBOT_PLATFORM_DRV_LIB::setXVelocity(double vel)
{
    x_lvel = vel;
}

void OMNIBOT_PLATFORM_DRV_LIB::setYVelocity(double vel)
{
    y_lvel = vel;
}

void OMNIBOT_PLATFORM_DRV_LIB::setZVelocity(double vel)
{
    z_avel = vel;
}

void OMNIBOT_PLATFORM_DRV_LIB::getWheelsVelocity(wheels_angular_vel_t *wheels_output)
{

    double  r = _config->wheels_deameter   / 2;
    double Lx = _config->wheel_base_lenth / 2;
    double Ly = _config->wheel_base_width / 2;

    wheels_output->w1 = (1 / r) * (x_lvel - y_lvel - (Lx + Ly) * z_avel);
    wheels_output->w2 = (1 / r) * (x_lvel + y_lvel + (Lx + Ly) * z_avel);
    wheels_output->w3 = (1 / r) * (x_lvel + y_lvel - (Lx + Ly) * z_avel);
    wheels_output->w4 = (1 / r) * (x_lvel - y_lvel + (Lx + Ly) * z_avel);
}
