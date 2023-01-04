//Markus Buchholz, 2023
// g++ spiral_helix.cpp -o t -I/usr/include/python3.8 -lpython3.8
//https://en.wikipedia.org/wiki/Helix
#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

//-----------------------------------------------------------

void plot3D(std::tuple<std::vector<float>, std::vector<float>, std::vector<float>> data)
{

    std::vector<float> xX = std::get<0>(data);
    std::vector<float> yY = std::get<1>(data);
    std::vector<float> zZ = std::get<2>(data);

    plt::plot3(xX, yY, zZ);
    plt::xlabel("x");
    plt::ylabel("y");
    plt::set_zlabel("z");
    plt::show();
}

//-----------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>, std::vector<float>> spiralHelix()
{

    float a = 2.0;
    float b = 1.0;

    float theta = M_PI / 6;
    int size = 5000;
    float dt = 0.01;

    std::vector<float> xX;
    std::vector<float> yY;
    std::vector<float> zZ;

    for (int ii = 0; ii < size; ii++)
    {

        float xXi = a * std::cos(theta * ii * dt);
        float yYi = a * std::sin(theta * ii * dt);
        float zZi = b * ii * dt;

        xX.push_back(xXi);
        yY.push_back(yYi);
        zZ.push_back(zZi);
    }
    return std::make_tuple(xX, yY, zZ);
}

//-----------------------------------------------------------

int main()
{
    std::tuple<std::vector<float>, std::vector<float>, std::vector<float>> spiral3D = spiralHelix();

    plot3D(spiral3D);
}
