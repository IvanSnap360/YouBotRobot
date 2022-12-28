#ifndef __OMNIPLATFORMDRV_H__
#define __OMNIPLATFORMDRV_H__

class OMNIPLATFORMDRV
{

 
public:

    typedef struct OmniPlatformCfg_t
    {
        double wheel_base_width;
        double wheel_base_lenth;
    };
    

    OMNIPLATFORMDRV();
    ~OMNIPLATFORMDRV();

    void setParamsConfig();

private:

};

OMNIPLATFORMDRV::OMNIPLATFORMDRV(/* args */)
{
}

OMNIPLATFORMDRV::~OMNIPLATFORMDRV()
{
}


#endif // __OMNIPLATFORMDRV_H__