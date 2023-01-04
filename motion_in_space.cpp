//Markus Buchholz, 2023

#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <tuple>


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

    float dt = 0.01;

    auto fx = [](float t)
    { return std::cos(t) * t; };

    auto fy = [](float t)
    { return std::sin(t) * t; };

    auto fz = [](float t)
    { return t; };

    velo.x = (fx(t + dt) - fx(t)) / dt;
    velo.y = (fy(t + dt) - fy(t)) / dt;
    velo.z = (fz(t + dt) - fz(t)) / dt;

    return velo;
}
//------------------------------------------------------------
R computeAcc(float t)
{

    R acc;

    float dt = 0.01;

    acc.x = (computeVelo(t + dt).x - computeVelo(t).x) / dt;
    acc.y = (computeVelo(t + dt).y - computeVelo(t).y) / dt;
    acc.z = (computeVelo(t + dt).z - computeVelo(t).z) / dt;

    return acc;
}

//------------------------------------------------------------

R computeJerk(float t)
{

    R jerk;

    float dt = 0.01;

    jerk.x = (computeAcc(t + dt).x - computeAcc(t).x) / dt;
    jerk.y = (computeAcc(t + dt).y - computeAcc(t).y) / dt;
    jerk.z = (computeAcc(t + dt).z - computeAcc(t).z) / dt;

    return jerk;
}

//------------------------------------------------------------

float computeSpeed(float t)
{

    return std::sqrt(std::pow(computeVelo(t).x, 2) + std::pow(computeVelo(t).y, 2) + std::pow(computeVelo(t).z, 2));
}

//------------------------------------------------------------

R computeTangent(float t)
{

    R tan;
    float speed = computeSpeed(t);
    return {computeVelo(t).x / speed, computeVelo(t).y / speed, computeVelo(t).z / speed};
}

//------------------------------------------------------------

float computeDiffTangentModule(float t)
{

    float dt = 0.01;
    R tan_dot;

    tan_dot.x = (computeTangent(t + dt).x - computeTangent(t).x) / dt;
    tan_dot.y = (computeTangent(t + dt).y - computeTangent(t).y) / dt;
    tan_dot.z = (computeTangent(t + dt).z - computeTangent(t).z) / dt;

    return std::sqrt(std::pow(tan_dot.x, 2) + std::pow(tan_dot.y, 2) + std::pow(tan_dot.z, 2));
}

//------------------------------------------------------------
float computeCurvature(float t)
{
    auto tan_mod = computeDiffTangentModule(t);

    return tan_mod / computeSpeed(t);
}

//------------------------------------------------------------

R computeNormal(float t)
{

    float dt = 0.01;
    R tan_dot;

    tan_dot.x = (computeTangent(t + dt).x - computeTangent(t).x) / dt;
    tan_dot.y = (computeTangent(t + dt).y - computeTangent(t).y) / dt;
    tan_dot.z = (computeTangent(t + dt).z - computeTangent(t).z) / dt;

    auto tan_mod = computeDiffTangentModule(t);

    return {tan_dot.x / tan_mod, tan_dot.y / tan_mod, tan_dot.z / tan_mod};
}

//------------------------------------------------------------

R computeBinominal(float t)
{

    R bin;

    auto a = computeTangent(t);
    auto b = computeNormal(t);
    // Sarruns rule
    bin.x = a.y * b.z - a.z * b.y;
    bin.y = a.z * b.x - a.x * b.z;
    bin.z = a.x * b.y - a.y * b.x;

    return bin;
}

//------------------------------------------------------------

float computeTorsion(float t)
{

    auto mod_v_x_a = computeCurvature(t) * std::pow(computeSpeed(t), 3);

    auto velo = computeVelo(t);
    auto acc = computeAcc(t);
    auto jerk = computeJerk(t);

    auto a = velo.x, b = velo.y, c = velo.z;
    auto d = acc.x, e = acc.y, f = acc.z;
    auto g = jerk.x, h = jerk.y, i = jerk.z;

    auto determinant = a * e * i + b * f * g + c * d * h - c * e * g - b * d * i - a * f * h;

    return determinant / std::pow(mod_v_x_a, 2);
}

//------------------------------------------------------------

std::tuple<float, float> computeComonentsOfAcc(float t)
{

    float dt = 0.01;
    auto acc_tan = (computeSpeed(t + dt) - computeSpeed(t)) / dt;

    auto acc_norm = computeCurvature(t) * std::pow(computeSpeed(t), 2);

    return std::make_tuple(acc_tan, acc_norm);
}

//------------------------------------------------------------

R computeVeloArcLength(float t)
{

    R velo;

    float dt = 0.01;

    auto fx = [](float t)
    { return 4 * std::cos(t); };

    auto fy = [](float t)
    { return 4 * std::sin(t); };

    auto fz = [](float t)
    { return 3 * t; };

    velo.x = (fx(t + dt) - fx(t)) / dt;
    velo.y = (fy(t + dt) - fy(t)) / dt;
    velo.z = (fz(t + dt) - fz(t)) / dt;

    return velo;
}
//------------------------------------------------------------

float computeSpeedArcLength(float t)
{

    return std::sqrt(std::pow(computeVeloArcLength(t).x, 2) + std::pow(computeVeloArcLength(t).y, 2) + std::pow(computeVeloArcLength(t).z, 2));
}

//------------------------------------------------------------

float trapezoidalMethod(float a, float b)
{

    int n = 10000;
    auto h = (b - a) / n;
    auto aA = computeSpeedArcLength(a) * 0.5;
    auto bB = computeSpeedArcLength(b) * 0.5;

    auto sum = 0;

    for (int ii = 1; ii < n; ii++)
    {

        sum += computeSpeedArcLength(a + ii * h);
    }
    return h * (aA + bB + sum);
}

//-----------------------------------------------------------

float computeArcLength(float a, float b)
{
    return trapezoidalMethod(a, b);
}

//------------------------------------------------------------

int main()
{

    float t = std::sqrt(3);

    R velo = computeVelo(t);

    std::cout << " v_x= " << velo.x << ", v_y= " << velo.y << ", v_z= " << velo.z << "\n";

    R acc = computeAcc(t);

    std::cout << " a_x= " << acc.x << ", a_y= " << acc.y << ", a_z= " << acc.z << "\n";

    R jerk = computeJerk(t);

    std::cout << " jerk_x= " << jerk.x << ", jerk_y= " << jerk.y << ", jerk_z= " << jerk.z << "\n";

    auto speed = computeSpeed(t);

    std::cout << " speed= " << speed << "\n";

    auto tan = computeTangent(t);

    std::cout << " tangent_x= " << tan.x << ", tangent_y= " << tan.y << ", tangent_z= " << tan.z << "\n";

    auto k = computeCurvature(t);

    std::cout << " curvature_k= " << k << "\n";

    auto norm = computeNormal(t);

    std::cout << " norm_x= " << norm.x << ", norm_y= " << norm.y << ", norm_z= " << norm.z << "\n";

    auto bin = computeBinominal(t);

    std::cout << " binominal_x= " << bin.x << ", binominal_y= " << bin.y << ", binominal_z= " << bin.z << "\n";

    auto torsion = computeTorsion(t);

    std::cout << " torsion= " << torsion << "\n";

    auto acc_comp = computeComonentsOfAcc(t);

    auto acc_tan = std::get<0>(acc_comp);
    auto acc_norm = std::get<1>(acc_comp);

    std::cout << " acc_tan= " << acc_tan << ", acc_norm= " << acc_norm << "\n";

    auto arc = computeArcLength(0, M_PI / 2);

    std::cout << " arc_length= " << arc << "\n";
}
