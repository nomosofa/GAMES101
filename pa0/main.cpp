#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

#define PI 3.1415926f
#define DEG2RAD (PI / 180.0f)

int main()
{
    Eigen::Vector3f P(2, 1, 1), ans;
    Eigen::Matrix3f rotate;
    float rad = 45 * DEG2RAD;
    rotate << cos(rad), -sin(rad), 0,
        sin(rad), cos(rad), 0,
        0, 0, 1;
    ans = rotate * P;
    std::cout << ans << std::endl;
    return 0;
}