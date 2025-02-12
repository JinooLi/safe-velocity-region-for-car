#!/usr/bin/env python3

import time
from typing import Tuple

import matplotlib.pyplot as plt
import numpy as np

# 맞는 속도가 없는 경우를 나타내기 위한 상수.
# matplot에서는 np.nan을 사용할 수 없기 때문에 -1.0으로 설정
NAN: float = -1.0


class SafeCar:
    def __init__(
        self,
        omega: float = 3.2,
        max_delta: float = 1.1,
        dt: float = 0.05,
        L: float = 0.325,
        c: float = 1.0,
    ):
        """SafeCar 클래스 생성자

        Args:
            omega (float, optional)     : 조향각 최대 각속도(rad/s). Defaults to 3.2.
            max_delta (float, optional) : 최대 조향각(rad). Defaults to 1.1.
            dt (float, optional)        : 제어 주기. Defaults to 0.05.
            L (float, optional)         : wheelbase 차량의 앞, 뒷 축 간의 거리(m). Defaults to 0.325.
            c (float, optional)         : sqrt(L*mu*g). 실험을 통해 얻는 상수. Defaults to 1.0.
        """
        self.omega = omega
        self.max_delta = max_delta
        self.dt = dt
        self.L = L
        self.c = c

    def make_velo_bound_next_step(
        self,
        v_n: float,
        delta_next: float,
    ) -> Tuple[float, float]:
        """주어진 상황에서 다음 스텝(제어 주기)에서의 속도 범위를 계산하는 함수.

        Args:
            v_n(float) : 현재 속도(m/s)
            delta_next : 다음 스텝에서의 조향각(rad)

        Returns:
            Tuple[float,float] : v_{n+1}의 안전하기 위한 최대값과 최소값. 만약 이 값이 NAN(==-1)이면, 주어진 상황에 적합한 범위가 없다는 것을 의미한다.
        """
        if abs(delta_next) > self.max_delta:
            return NAN, NAN

        # c = sqrt(L*mu*g) -> c^2/L = mu*g

        # 1) 보조 상수 정의
        # tan(|delta_{n+1}|) / L
        alpha = np.tan(abs(delta_next)) / self.L

        # 2) 4차 다항식 f(v) = 0 의 계수 구하기
        #    f(v) = (alpha^2)*v^4
        #         + (1/dt^2)*v^2
        #         - 2*v_n/dt^2 * v
        #         + (v_n^2/dt^2 - (mu*g)^2)

        # v^4 항
        A4 = alpha**2
        # v^3 항 (없음)
        A3 = 0.0
        # v^2 항
        A2 = 1.0 / (self.dt**2)
        # v^1 항
        A1 = -2.0 * v_n / (self.dt**2)
        # 상수항. c = sqrt(L*mu*g) -> c^2/L = mu*g
        A0 = (v_n**2) / (self.dt**2) - (self.c**2 / self.L) ** 2

        # 3) 다항식 근사 찾기
        coeffs: list[float] = [A4, A3, A2, A1, A0]
        roots = np.roots(coeffs)  # 4개의 근(복소수 포함)을 반환

        # 4) 실근만 선별
        real_roots: list[float] = []
        for r in roots:
            if np.isreal(r):
                real_roots.append(r.real)

        # 5) 근들을 오름차순 정렬
        real_roots.sort()

        # 6) 각 case에 따라 최대, 최소 속도 설정
        max_feasible_speed: float = NAN
        min_feasible_speed: float = NAN
        if len(real_roots) == 0:
            max_feasible_speed, min_feasible_speed = NAN, NAN
        elif len(real_roots) == 1:
            max_feasible_speed, min_feasible_speed = real_roots[0], real_roots[0]
        elif len(real_roots) == 2:
            max_feasible_speed, min_feasible_speed = real_roots[1], real_roots[0]
        else:  # 3개 이상의 실근이 나오면 에러 발생. 그런데 이게 수학적으로 불가능함. 만약 이 에러가 난다면 코드를 잘못 짠 것이다.
            raise ValueError(
                "bound_speed_next_step: too many real roots. something wrong."
            )

        return max_feasible_speed, min_feasible_speed

    def is_pass_worst_case_test(
        self,
        v_n: float,
        delta_next: float,
        iteration_limit: int = 100,
    ) -> bool:
        """worst case test. 조향각속도를 최대한으로 하는 시뮬레이션을 통해 안전 영역에 머무르는지 확인하는 함수.
        안전영역에 머무르는 것이 가능하면 True를 반환한다. 아니면 False.

        Args:
            v_n (float)                     : 현재 속도(m/s)
            delta_next (float)              : 다음 조향각(rad)
            iteration_limit (int, optional) : 반복 횟수 제한. Defaults to 100.

        Raises:
            ValueError: iteration limit exceeded. 반복 횟수 제한을 초과한 경우 발생.

        Returns:
            bool: 안전영역에 머무르는 것이 가능하면 True, 아니면 False
        """
        # 조향각의 방향과 같은 방향의 최대 각가속도로 설정
        if delta_next < 0:
            omega = -self.omega
        else:
            omega = self.omega

        new_delta_next = delta_next
        min_velo = v_n
        end_flag = False
        for __ in range(iteration_limit):
            max_velo, min_velo = self.make_velo_bound_next_step(
                min_velo, new_delta_next
            )

            if min_velo == NAN:
                return False

            # 속도 범위가 0을 포함하는 경우 0으로 설정
            if min_velo * max_velo <= 0:
                min_velo = 0
            else:  # 속도 범위가 0을 포함하지 않는 경우, 속도 범위에서 더 작은 크기의 속도를 선택
                min_speed = min(abs(min_velo), abs(max_velo))
                min_velo = min_speed * np.sign(min_velo)

            if end_flag:
                return True

            new_delta_next += self.dt * omega

            # steering limit - 최대 조향각에 도달했음에도 안전하면 True를 반환하기 위해 end_flag를 True로 설정
            if abs(new_delta_next) > self.max_delta:
                new_delta_next = self.max_delta * np.sign(new_delta_next)
                end_flag = True

        # iteration limit에 도달하면 에러 발생
        raise ValueError(
            "worst_case_test: iteration limit exceeded. Please set the iteration limit larger."
        )

    def make_velo_bound_with_worst_case(
        self,
        v_n: float,
        delta_next: float,
        iteration_limit: int = 100,
        binary_search_iter_time: int = 20,
    ) -> Tuple[float, float]:
        """worst case test를 통해 안전한 속도 범위를 찾는 함수

        Args:
            v_n (float)                             : 현재 속도
            delta_next (float)                      : 현재 조향각
            iteration_limit (int, optional)         : worst case test 반복 횟수 제한. Defaults to 100.
            binary_search_iter_time (int, optional) : binary search iteration 횟수 제한. Defaults to 20.

        Returns:
            Tuple[float,float] : v_{n+1}의 안전하기 위한 최대값과 최소값.
            만약 이 값이 NAN(==-1)이면, 이 부등식을 만족하는 속도가 없다는 것을 의미한다.
        """
        max_speed, min_speed = self.make_velo_bound_next_step(v_n, delta_next)

        # 적절한 속도가 없는 경우 NAN 반환
        if min_speed == NAN:
            return NAN, NAN

        max_worst_case_test_result = self.is_pass_worst_case_test(
            max_speed, delta_next, iteration_limit
        )

        # 둘 다 같은 값을 가지는 case 처리
        if max_speed == min_speed:
            if max_worst_case_test_result:  # 안전한 경우 그대로 반환
                return max_speed, min_speed
            else:  # 안전하지 않은 경우 NAN 반환
                return NAN, NAN

        min_worst_case_test_result = self.is_pass_worst_case_test(
            min_speed, delta_next, iteration_limit
        )

        # 두 값이 다른 경우 처리
        if max_worst_case_test_result:  # max_speed가 안전한 경우 그대로 반환
            return max_speed, min_speed
        elif (
            min_worst_case_test_result
        ):  # min_speed만 안전한 경우 binary search로 안전한 최대 속도 찾기
            width = max_speed - min_speed / 2
            test_speed = min_speed + width
            for __ in range(binary_search_iter_time):
                is_it_okay = self.is_pass_worst_case_test(
                    test_speed, delta_next, iteration_limit
                )

                if is_it_okay:
                    test_speed += width
                else:
                    test_speed -= width

                width /= 2

            return test_speed - width, min_speed
        else:  # min_speed도 안전하지 않은 경우 NAN 반환
            return NAN, NAN

    def get_max_speed(self, iteration_limit: int = 1000) -> float:
        """최대 속도를 찾는 함수. 최대한 빠르게 가속하는 상황을 시뮬레이션 하여 수렴하는 속도를 찾는다.

        Args:
            iteration_limit (int, optional): 최대 속도를 찾기 위한 시뮬레이션 step 수. Defaults to 1000.

        Raises:
            ValueError: iteration limit exceeded. 최대 속도를 찾기 위한 시뮬레이션 step 수가 부족한 경우 발생.

        Returns:
            float: 수렴한 최대 속도(m/s)
        """
        # 최대 속도 찾기
        max_speed = 0
        for __ in range(iteration_limit):
            next_max_speed, _ = self.make_velo_bound_with_worst_case(
                v_n=max_speed, delta_next=0
            )
            absolute_error = abs(next_max_speed - max_speed)
            if absolute_error < 1e-4:
                return next_max_speed
            max_speed = next_max_speed

        raise ValueError("get_max_speed: iteration limit exceeded.")

    def visualize_speed_bound(self):
        """state에 따른 속도 범위를 그래프로 그리는 함수."""
        # 그래프 그리기
        # 각 v_n을 x축으로 놓고, delta_next를 y축으로 놓았을 때 bound_speed_next_step을 계산하고
        # z축에 놓은 그래프를 그린다.
        # (v_n, delta_next)의 범위는 각각 아래와 같이 설정한다.
        v_n_range = np.linspace(0, 10, 100)
        delta_next_range = np.linspace(-1.2, 1.2, 100)

        v_n_mesh, delta_next_mesh = np.meshgrid(v_n_range, delta_next_range)
        bound_speed_next_step_mesh = np.vectorize(self.make_velo_bound_with_worst_case)(
            v_n_mesh, delta_next_mesh
        )
        cmap = "rainbow"

        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")
        ax.contour(
            v_n_mesh,
            delta_next_mesh,
            bound_speed_next_step_mesh[0],
            levels=20,
            cmap=cmap,
        )
        ax.plot_wireframe(v_n_mesh, delta_next_mesh, bound_speed_next_step_mesh[1])
        ax.set_xlabel("$v_{n}$")
        ax.set_ylabel(r"$\delta_{\t{next}}$")
        ax.set_zlabel("velocity bound of next step")

        plt.show()


if __name__ == "__main__":
    # 변수
    v_n_example = 2  # m/s
    delta_next_example = 1  # 라디안

    # 상수
    c_example = 2  # sqrt(L*mu*g)
    omega_example = 3.2  # rad/s
    dt_example = 0.05  # s

    car = SafeCar(c=c_example, omega=omega_example, dt=dt_example)

    # 하나의 state에 대해 범위를 구하는데 걸리는 시간 측정
    start = time.time()
    print(
        "Test-bound_speed_next_step :",
        car.make_velo_bound_with_worst_case(v_n_example, delta_next_example),
    )
    end = time.time()
    print(f"Elapsed time: {end - start} sec")

    print("max speed:", car.get_max_speed())

    # 그래프 그리기
    car.visualize_speed_bound()
