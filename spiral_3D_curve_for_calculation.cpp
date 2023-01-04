// Markus Buchholz, 2023
//  g++ spiral_helix.cpp -o t -I/usr/include/python3.8 -lpython3.8
// https://en.wikipedia.org/wiki/Helix
#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

//------------------------------------------------------------

struct R
{
    float x;
    float y;
    float z;
};

//------------------------------------------------------------

R computeVelo(float t)
{

    R velo;

    float dt = 0.1;
    auto a = 2.0;
    auto b = 5.0;

    auto fx = [=](float t)
    { return a * std::cos(t); };

    auto fy = [=](float t)
    { return a * std::sin(t); };

    auto fz = [=](float t)
    { return b * std::pow(std::cos(t), 2); };

    velo.x = (fx(t + dt) - fx(t)) / dt;
    velo.y = (fy(t + dt) - fy(t)) / dt;
    velo.z = (fz(t + dt) - fz(t)) / dt;

    return velo;
}

//-----------------------------------------------------------

float computeSpeed(float t)
{

    return std::sqrt(std::pow(computeVelo(t).x, 2) + std::pow(computeVelo(t).y, 2) + std::pow(computeVelo(t).z, 2));
}

//------------------------------------------------------------

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

std::tuple<std::vector<float>, std::vector<float>, std::vector<float>> curve3D()
{

    float a = 2.0;
    float b = 5.0;

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
        float zZi = b * std::pow(std::cos(theta * ii * dt), 2);

        xX.push_back(xXi);
        yY.push_back(yYi);
        zZ.push_back(zZi);
    }
    return std::make_tuple(xX, yY, zZ);
}

//-----------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>> compute2DSpeed()
{

    std::vector<float> time;
    std::vector<float> speed;
    float dt = 0.1;

    for (float ii = 0.0f; ii < 10.0f; ii = ii + dt)
    {

        speed.push_back(computeSpeed(dt));
        time.push_back(ii);
    }

    return std::make_tuple(time, speed);
}
//---------------------------------------------------------------------------------------------------------

void plot2D(std::vector<float> xX, std::vector<float> yY)
{
    plt::title("Speed (constant) ");
    plt::named_plot("solution", xX, yY);
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::legend();
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::show();
}

//---------------------------------------------------------------------------------------------------------

int main()
{
    std::tuple<std::vector<float>, std::vector<float>, std::vector<float>> spiral3D = curve3D();

    auto test = computeSpeed(0);
    std::cout << test << "\n";

    auto speedC = compute2DSpeed();
    auto time = std::get<0>(speedC);
    auto speed = std::get<1>(speedC);

    // plot3D(spiral3D);
    plot2D(time, speed);
}
