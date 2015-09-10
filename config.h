#ifndef _CONFIG_ADC_
#define _CONFIG_ADC_

#include <iostream>

class configuration
{
    public:
        // System
        float dt = 0.01; // System frequency
        float g = 9.80151; // Beijing
        float seaLevelPressure = 101500;

        // GY87
        int16_t gy87Offset[6] = {-1685, 3937, 1803, 0, -17, 109};

        // Altitude filter
        float baroFilterConfig[3] = {0.55, 1.0, 0.05};
        float sonarFilterConfig[3] = {0.75, 1.0, 0.035};

        // ESC controller
        int controlled_esc[4] = {6, 13, 19, 26};
        int ESCFrequency = 400; // Hz
        int ESCOutMin = 1200;
        int ESCOutMax = 1800;

        // PID system
        float ratePIDSystemConfig[3][7];
        float attitudePIDSystemConfig[3][7] = 
        {
            {100, 1, 0.03, 200, -200, 400, -400},
            {300, 1, 0.02, 200, -200, 400, -400},
            {300, 1, 0.02, 200, -200, 400, -400}
        };
        float ZPIDSystemConfig[4][7] =
        {
            {20, 20, 20, 25, 0, 600, 0}, 
            {20, 20, 20, 25, 0, 600, 0}, 

            {20, 20, 20, 25, 0, 600, 0}, 
            {20, 20, 20, 25, 0, 600, 0}, 
        };
};

using std::ostream;
using std::istream;
using std::endl;

std::ostream& operator<<(std::ostream& stream, configuration const& data);
std::istream& operator>>(std::istream& stream, configuration& data);

#endif // _CONFIG_ADC_
