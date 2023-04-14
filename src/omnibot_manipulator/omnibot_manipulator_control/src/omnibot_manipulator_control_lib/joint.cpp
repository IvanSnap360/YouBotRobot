#include "omnibot_manipulator_control_lib/joint.h"

JOINT::JOINT(joint_cfg_t *config_ptr, DynamixelWorkbench *dxl)
{
    _dxl = dxl;
    _cfg = config_ptr;

    _res = _dxl->ping(_cfg->id);

    if (_res)
        ROS_INFO("Success to ping dynamixel id: %d", _cfg->id);
    else
        ROS_ERROR("Failed to ping dynamixel id: %d", _cfg->id);

    _res = _dxl->setPositionControlMode(_cfg->id);

    if (_res)
        ROS_INFO("Success to set PositionControlMode parameter for id: %d", _cfg->id);
    else
        ROS_ERROR("Failed to set PositionControlMode parameter for id: %d", _cfg->id);
}

bool JOINT::setLED(bool state)
{
    _res = _dxl->led(_cfg->id, state);

    if (_res)
        ROS_INFO("Success to set LED parameter for id: %d", _cfg->id);
    else
        ROS_ERROR("Failed to set LED parameter for id: %d", _cfg->id);

    return _res;
}

bool JOINT::setVelocity(double val)
{
    _res = _dxl->goalVelocity(_cfg->id, (float)val);

    if (_res)
        ROS_INFO("Success to set GoalVelocity parameter for id: %d", _cfg->id);
    else
        ROS_ERROR("Failed to set GoalVelocity parameter for id: %d", _cfg->id);

    return _res;
}

bool JOINT::setPosition(double val)
{
    if (val > _cfg->costrains_max_position)
        val = _cfg->costrains_max_position;
    if (val < _cfg->costrains_min_position)
        val = _cfg->costrains_min_position;

    _res = _dxl->goalPosition(_cfg->id, (float)val);

    if (_res)
        ROS_INFO("Success to set GoalPosition parameter for id: %d", _cfg->id);
    else
        ROS_ERROR("Failed to set GoalPosition parameter for id: %d", _cfg->id);

    return _res;
}

bool JOINT::getPosition(double &data)
{
    int32_t d = 0;
    _res = _dxl->getPresentPositionData(_cfg->id, &d);
    data = _dxl->convertValue2Radian(_cfg->id, d);

    if (_res)
        ROS_INFO("Success to get GoalPosition parameter for id: %d", _cfg->id);
    else
        ROS_ERROR("Failed to get GoalPosition parameter for id: %d", _cfg->id);

    return _res;
}

JOINT::~JOINT()
{
}
