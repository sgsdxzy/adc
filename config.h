#ifndef _CONFIG_ADC_
#define _CONFIG_ADC_

class configuration
{
    public:
        // System
        float dt = 0.01; // System frequency
        float g = 9.80151; // Beijing
        float seaLevelPressure = 101500;

        // GY87
        int16_t gy87Offset[6] = {-1685, 3946, 1838, 0, -11, 107};

        // Altitude filter
        float k[3] = {0.55, 1.0, 0.05};

        // ESC controller
        int controlled_esc[4] = {6, 13, 19, 26};
        int ESCFrequency = 400; // Hz
        int ESCOutMin = 1200;
        int ESCOutMax = 1800;

        // PID system
        float ratePIDSystemConfig[3][7];
        float attitudePIDSystemConfig[3][7] = 
        {
            {100, 1, 0.03, 200, -200, 300, -300},
            {100, 1, 0.02, 200, -200, 300, -300},
            {100, 1, 0.02, 200, -200, 300, -300}
        };
        float ZPIDSystemConfig[4][7] =
        {
            {20, 20, 20, 25, 0, 600, 0}, 
            {20, 20, 20, 25, 0, 600, 0}, 

            {20, 20, 20, 25, 0, 600, 0}, 
            {20, 20, 20, 25, 0, 600, 0}, 
        }
};

#endif // _CONFIG_ADC_
