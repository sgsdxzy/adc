#include "debug.h"

void info(string inf)
{
    std::cout << "Info: " << inf << std::endl;
}

void warn(string info)
{
    std::cerr << "Warning: " << info << std::endl;
}

void err(string info)
{
    std::cerr << "Error: " << info << std::endl;
}
