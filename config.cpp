#include "config.h"

ostream& operator<<(ostream& stream, configuration const& config)
{
    stream << config.dt << endl;
    stream << config.g << endl;
    stream << config.seaLevelPressure << endl;

    for (int i=0;i<6;i++) {
        stream << config.gy87Offset[i] << " ";
    }
    stream << endl;

    for (int i=0;i<3;i++) {
        stream << config.baroFilterConfig[i] << " ";
    }
    stream << endl;

    for (int i=0;i<3;i++) {
        stream << config.sonarFilterConfig[i] << " ";
    }
    stream << endl;

    for (int i=0;i<4;i++) {
        stream << config.controlled_esc[i] << " ";
    }
    stream << endl;

    stream << config.ESCFrequency << endl;
    stream << config.ESCOutMin << endl;
    stream << config.ESCOutMax << endl;

    for (int i=0;i<3;i++) {
        for (int j=0;j<7;j++) {
            stream << config.ratePIDSystemConfig[i][j] << " ";
        }
        stream << endl;
    }
    for (int i=0;i<3;i++) {
        for (int j=0;j<7;j++) {
            stream << config.attitudePIDSystemConfig[i][j] << " ";
        }
        stream << endl;
    }
    for (int i=0;i<4;i++) {
        for (int j=0;j<7;j++) {
            stream << config.ZPIDSystemConfig[i][j] << " ";
        }
        stream << endl;
    }

    return stream;
}

istream& operator>>(istream& stream, configuration& config)
{
    stream >> config.dt;
    stream >> config.g;
    stream >> config.seaLevelPressure;

    for (int i=0;i<6;i++) {
        stream >> config.gy87Offset[i];
    }

    for (int i=0;i<3;i++) {
        stream >> config.baroFilterConfig[i];
    }
    for (int i=0;i<3;i++) {
        stream >> config.sonarFilterConfig[i];
    }

    for (int i=0;i<4;i++) {
        stream >> config.controlled_esc[i];
    }
    stream >> config.ESCFrequency;
    stream >> config.ESCOutMin;
    stream >> config.ESCOutMax;

    for (int i=0;i<3;i++) {
        for (int j=0;j<7;j++) {
            stream >> config.ratePIDSystemConfig[i][j];
        }
    }
    for (int i=0;i<3;i++) {
        for (int j=0;j<7;j++) {
            stream >> config.attitudePIDSystemConfig[i][j];
        }
    }
    for (int i=0;i<4;i++) {
        for (int j=0;j<7;j++) {
            stream >> config.ZPIDSystemConfig[i][j];
        }
    }

    return stream;
}
