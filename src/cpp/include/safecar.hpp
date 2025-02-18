#pragma once

#include <cmath>      // std::pow, std::tan, std::abs
#include <functional> // std::abs
#include <iostream>   // std::cerr
#include <utility>    // std::pair

#include <pybind11/pybind11.h>

#define DANGER -1.0

typedef struct _velocityBound
{
    bool isSafe;
    double vMax;
    double vMin;
} veloBound;

class SafeCar
{
private:
    const double _omega;    // 조향각속도 최대값(rad/s)
    const double _maxDelta; // 조향각 최대값(rad)
    const double _dt;       // 시간 간격(s)
    const double _L;        // 차량의 wheelbase(m)
    const double _c;        // 상수(c = sqrt(L*mu*g))

    /**
     * @brief 4차 다항식
     *
     * @param currentVelocity 현재 속도(m/s) : 4차 다항식의 계수를 만드는데 쓰임
     * @param nextDelta 다음 step에서의 조향각(rad) : 4차 다항식의 계수를 만드는데 쓰임
     * @param v 속도(m/s) : 4차 다항식의 변수
     * @return double - 4차 다항식의 값
     */
    class Polynomial
    {
    private:
        double _a4;
        // _a3 = 0
        double _a2;
        double _a1;
        double _a0;

    public:
        Polynomial() = default;
        Polynomial(double currentVelocity, double nextDelta, double L, double dt, double c);

        /**
         * @brief Set the Coefficients of the Polynomial
         *
         * @param currentVelocity 현재 속도
         * @param nextDelta 다음 step에서의 조향각
         * @param L 차량의 wheelbase
         * @param dt 시간 간격(제어주기)
         * @param c 상수(c = sqrt(L*mu*g))
         */
        void setCoefficients(double currentVelocity, double nextDelta, double L, double dt,
                             double c);

        /**
         * @brief Get the Value of the Polynomial
         *
         * @param v 속도
         * @return double - 4차 다항식의 값
         */
        double getValue(double v);

        /**
         * @brief Get the Derivative Value of the Polynomial
         *
         * @param v 속도
         * @return double - 4차 다항식의 미분값
         */
        double getDerivativeValue(double v);
    };

    Polynomial _polynomial;

public:
    SafeCar(double omega = 3.2, double maxDelta = 1.1, double dt = 0.05, double L = 0.325,
            double c = 2.0);

    /**
     * @brief 현재 속도와 다음 step에서의 조향각을 받아서 안전한 속도의 범위를 반환하는 함수
     *
     * @param currentVelocity 현재 속도
     * @param nextDelta 다음 step에서의 조향각
     * @param tolerance Newton-Raphson method의 tolerance(optional)
     * @return veloBound - 안전한 최대 속도, 최소 속도
     */
    veloBound makeVeloBoundNextStep(double currentVelocity, double nextDelta,
                                    double tolerance = 1e-9);

    /**
     * @brief worst case 시뮬레이션을 통해 차량의 state가 안전한지 판단하는 함수
     *
     * @param currentVelocity 현재 속도
     * @param nextDelta 다음 step에서의 조향각
     * @return bool - true, false
     */
    bool isPassWorstCase(double currentVelocity, double nextDelta);

    /**
     * @brief Make the Velo Bound With Worst Case simulation
     *
     * @param currentVelocity 현재 속도
     * @param nextDelta 다음 step에서의 조향각
     * @param tolerance binary search의 tolerance(optional)
     * @return veloBound - 안전한 최대 속도, 최소 속도
     */
    veloBound makeVeloBoundWithWorstCase(double currentVelocity, double nextDelta,
                                         double tolerance = 1e-9);
};
