#ifndef __OMNIBOT_PLATFORM_DRV_LIB_H__
#define __OMNIBOT_PLATFORM_DRV_LIB_H__
#include <iostream>
typedef struct 
{
    double wheel_base_width;
    double wheel_base_lenth;

    double wheels_deameter;
    double wheels_width;
    

    double max_linear_x_velocity;
    double min_linear_x_velocity;

    double max_linear_y_velocity;
    double min_linear_y_velocity;

    double max_angular_z_velocity;
    double min_angular_z_velocity;

    bool reverse_wheels[4];

}omnibot_platform_drv_cfg_t;

typedef struct 
{
    double w1,w2,w3,w4;
}wheels_angular_vel_t;


class OMNIBOT_PLATFORM_DRV_LIB
{
private:
    omnibot_platform_drv_cfg_t* _config;
    double x_lvel, y_lvel, z_avel;
public:
    
    
    OMNIBOT_PLATFORM_DRV_LIB();
    void setConfig(omnibot_platform_drv_cfg_t* config);
    void setXVelocity(double vel);
    void setYVelocity(double vel);
    void setZVelocity(double vel);
    
    void getWheelsVelocity(wheels_angular_vel_t *wheels_output);


    ~OMNIBOT_PLATFORM_DRV_LIB();
};




#endif // __OMNIBOT_PLATFORM_DRV_LIB_H__