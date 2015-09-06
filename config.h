#ifndef _CONFIG_ADC_
#define _CONFIG_ADC_

class configuration
{
    public:
        // GY87
        int16_t gy87Offset[6] = {-1685, 3946, 1838, 0, -11, 107};

        // Altitude filter
        float k[3] = {0.55, 1.0, 0.05};

        // System
        float dt = 0.01; // System frequency
        float g = 9.80151; // Beijing
        float seaLevelPressure = 101500;
        int controlled_esc[4] = {6, 13, 19, 26};
        int ESCFrequency = 400; // Hz
        int outMin = 1200;
        int outMax = 1800;
};

#endif // _CONFIG_ADC_
