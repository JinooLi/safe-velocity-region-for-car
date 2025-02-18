#include "../include/safecar.hpp"

void SafeCar::Polynomial::setCoefficients(double currentVelocity, double nextDelta, double L,
                                          double dt, double c)
{
    double alpha = tan(std::abs(nextDelta)) / L;
    double pdt = std::pow(dt, 2);

    _a4 = std::pow(alpha, 2);
    // _a3 = 0
    _a2 = 1 / pdt;
    _a1 = -2 * currentVelocity / pdt;
    _a0 = std::pow(currentVelocity, 2) / pdt - std::pow((std::pow(c, 2) / L), 2);
}

SafeCar::Polynomial::Polynomial(double currentVelocity, double nextDelta, double L, double dt,
                                double c)
{
    setCoefficients(currentVelocity, nextDelta, L, dt, c);
}

SafeCar::SafeCar(double omega, double maxDelta, double dt, double L, double c)
    : _omega(omega), _maxDelta(maxDelta), _dt(dt), _L(L), _c(c)
{
    _polynomial = Polynomial(0, 0, _L, _dt, _c);
}

double SafeCar::Polynomial::getValue(double v)
{
    double v1 = v;
    double v2 = v * v1;
    double v4 = v2 * v2;

    return _a4 * v4 + _a2 * v2 + _a1 * v1 + _a0;
}

double SafeCar::Polynomial::getDerivativeValue(double v)
{
    double v1 = v;
    double v2 = v * v1;
    double v3 = v * v2;

    return 4 * _a4 * v3 + 2 * _a2 * v1 + _a1;
}

veloBound SafeCar::makeVeloBoundNextStep(double currentVelocity, double nextDelta,
                                         double tolerance)
{
    _polynomial.setCoefficients(currentVelocity, nextDelta, _L, _dt, _c);
    veloBound result;

    if (_polynomial.getValue(currentVelocity) > 0 || std::abs(nextDelta) > _maxDelta)
    {
        result.isSafe = false;
        result.vMax = DANGER;
        result.vMin = DANGER;
        return result;
    }

    result.isSafe = true;

    // newton-raphson method for vMax
    double searchRange = _dt * std::pow(_c, 2) / _L; // 조향각이 0일 때의 속도 범위
    double vMax = currentVelocity + searchRange;
    for (;;)
    {
        double f = _polynomial.getValue(vMax);
        double dF = _polynomial.getDerivativeValue(vMax);

        double vMaxNext = vMax - f / dF;

        if (std::abs(vMaxNext - vMax) < tolerance)
        {
            vMax = vMaxNext;
            break;
        }

        vMax = vMaxNext;
    }

    // newton-raphson method for vMin
    double vMin = currentVelocity - searchRange;
    for (;;)
    {
        double f = _polynomial.getValue(vMin);
        double dF = _polynomial.getDerivativeValue(vMin);

        double vMinNext = vMin - f / dF;

        if (std::abs(vMinNext - vMin) < tolerance)
        {
            vMin = vMinNext;
            break;
        }

        vMin = vMinNext;
    }

    result.vMax = vMax;
    result.vMin = vMin;
    return result;
}

bool SafeCar::isPassWorstCase(double currentVelocity, double nextDelta)
{
    double newNextDelta = std::abs(nextDelta);
    double minV = currentVelocity;
    bool endFlag = false;
    veloBound result;
    for (;;)
    {
        result = makeVeloBoundNextStep(minV, newNextDelta);
        if (!result.isSafe)
        {
            return false;
        }

        // 범위가 0을 포함하는 경우 속도가 0으로 갈 수 있으므로 안전한 속도 범위로 판단
        if (result.vMax * result.vMin < 0)
        {
            return true;
        }
        else
        {
            double minSpeed = std::min(std::abs(result.vMax), std::abs(result.vMin));
            minV = minSpeed * (minV < 0 ? -1 : 1);
        }

        if (endFlag)
        {
            return true;
        }

        newNextDelta += _dt * _omega;

        // 조향각이 최대 조향각에 도달함을 확인.
        if (newNextDelta >= _maxDelta)
        {
            newNextDelta = _maxDelta;
            endFlag = true;
        }
    }
}

veloBound SafeCar::makeVeloBoundWithWorstCase(double currentVelocity, double nextDelta,
                                              double tolerance)
{
    veloBound result = makeVeloBoundNextStep(currentVelocity, nextDelta, tolerance);

    // 시뮬레이션 하기 전에 기존 속도가 안전한 속도가 아닌 경우
    if (!result.isSafe)
    {
        return result;
    }

    // worst case 시뮬레이션을 통해 안전한 속도 범위 찾기
    // 최대가 안전하면 최소도 안전하므로 바로 반환
    if (isPassWorstCase(result.vMax, nextDelta))
    {
        return result;
    }

    // 최소가 위험하면 최대도 위험하므로 반환
    if (!isPassWorstCase(result.vMin, nextDelta))
    {
        result.isSafe = false;
        result.vMax = DANGER;
        result.vMin = DANGER;
        return result;
    }

    // binary search를 통해 안전한 최대 속도 찾기
    double width = (result.vMax - result.vMin) / 2;
    double midV = result.vMin + width;
    bool isMidSafe;
    for (;;)
    {
        isMidSafe = isPassWorstCase(midV, nextDelta);

        if (isMidSafe)
        {
            if (width < tolerance)
            {
                result.vMax = midV;
                break;
            }
            midV += width;
        }
        else
        {
            midV -= width;
        }
        width /= 2;
    }

    return result;
}

// pybind11 네임스페이스
namespace py = pybind11;

PYBIND11_MODULE(safecar, m)
{
    // 1) veloBound 바인딩
    py::class_<veloBound>(m, "veloBound")
        .def(py::init<>()) // 기본 생성자 필요
        .def_readwrite("isSafe", &veloBound::isSafe)
        .def_readwrite("vMax", &veloBound::vMax)
        .def_readwrite("vMin", &veloBound::vMin);

    // 2) SafeCar 바인딩
    py::class_<SafeCar>(m, "SafeCar")
        .def(py::init<double, double, double, double, double>(),
             py::arg("omega") = 3.2,
             py::arg("maxDelta") = 1.1,
             py::arg("dt") = 0.05,
             py::arg("L") = 0.325,
             py::arg("c") = 1.0)
        .def("makeVeloBoundNextStep", &SafeCar::makeVeloBoundNextStep,
             py::arg("currentVelocity"),
             py::arg("nextDelta"),
             py::arg("tolerance") = 1e-9)
        .def("isPassWorstCase", &SafeCar::isPassWorstCase,
             py::arg("currentVelocity"),
             py::arg("nextDelta"))
        .def("makeVeloBoundWithWorstCase", &SafeCar::makeVeloBoundWithWorstCase,
             py::arg("currentVelocity"),
             py::arg("nextDelta"),
             py::arg("tolerance") = 1e-9);
}