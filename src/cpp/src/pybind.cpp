#include "../include/safecar.hpp"

// pybind11 네임스페이스
namespace py = pybind11;

PYBIND11_MODULE(safecar, m) {
    // 1) veloBound 바인딩
    py::class_<veloBound>(m, "veloBound")
        .def(py::init<>())  // 기본 생성자 필요
        .def_readwrite("isSafe", &veloBound::isSafe)
        .def_readwrite("vMax", &veloBound::vMax)
        .def_readwrite("vMin", &veloBound::vMin);

    // 2) SafeCar 바인딩
    py::class_<SafeCar>(m, "SafeCar")
        .def(py::init<double, double, double, double, double>(), py::arg("omega") = 3.2,
             py::arg("maxDelta") = 1.1, py::arg("dt") = 0.05, py::arg("L") = 0.325,
             py::arg("c") = 1.0)
        .def("makeVeloBoundNextStep", &SafeCar::makeVeloBoundNextStep, py::arg("currentVelocity"),
             py::arg("nextDelta"), py::arg("tolerance") = 1e-9)
        .def("isPassWorstCase", &SafeCar::isPassWorstCase, py::arg("currentVelocity"),
             py::arg("nextDelta"))
        .def("makeVeloBoundWithWorstCase", &SafeCar::makeVeloBoundWithWorstCase,
             py::arg("currentVelocity"), py::arg("nextDelta"), py::arg("tolerance") = 1e-9);
}