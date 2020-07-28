#include <stdio.h>
#include <iostream>

void print_init_info(float lambda, float alpha, float gamma)
{
    std::cout << "INITIALIZING CONTROL AND OBSERVATION NODE..."  <<std::endl;
    std::cout << "Observer.-> lambda=" << lambda << "    alpha=" << alpha << "   gamma=" << gamma << std::endl;
}

