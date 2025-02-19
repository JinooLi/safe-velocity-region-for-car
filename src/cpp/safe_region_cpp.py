#!/usr/bin/env python3
from . import safecar

# SafeCar 클래스를 Python에서 바로 사용 가능


class SafeCar:
    def __init__(
        self,
        omega: float = 3.2,
        max_delta: float = 1.1,
        dt: float = 0.05,
        L: float = 0.325,
        c: float = 1.0,
    ):
        self.car = safecar.SafeCar(omega, max_delta, dt, L, c)

    def make_velo_bound_with_worst_case(self, v_n: float, delta_next: float):
        bound = self.car.makeVeloBoundWithWorstCase(v_n, delta_next)
        return bound.vMax, bound.vMin

    def make_velo_bound_next_step(self, v_n: float, delta_next: float):
        bound = self.car.makeVeloBoundNextStep(v_n, delta_next)
        return bound.vMax, bound.vMin


if __name__ == "__main__":
    car = SafeCar()
    print(car.make_velo_bound_next_step(2, 0.1))
    print(car.make_velo_bound_with_worst_case(2, 0.1))
