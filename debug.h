#ifndef _DEBUG_ADC_
#define _DEBUG_ADC_

#include <iostream>
#include <string>

using  std::string;

void warn(string info)
{
    std::cerr << "Warning: " << info << std::endl;
}

void err(string info)
{
    std::cerr << "Error: " << info << std::endl;
}

#endif // _DEBUG_ADC_
