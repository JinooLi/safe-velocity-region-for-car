#!/usr/bin/env python3

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation


def get_max_speed(safe_car, iteration_limit: int = 1000) -> float:
    """최대 속도를 찾는 함수. 최대한 빠르게 가속하는 상황을 시뮬레이션 하여 수렴하는 속도를 찾는다.

    Args:
        safe_car (SafeCar): SafeCar 객체
        iteration_limit (int, optional): 최대 속도를 찾기 위한 시뮬레이션 step 수. Defaults to 1000.

    Raises:
        ValueError: iteration limit exceeded. 최대 속도를 찾기 위한 시뮬레이션 step 수가 부족한 경우 발생.

    Returns:
        float: 수렴한 최대 속도(m/s)
    """
    # 최대 속도 찾기
    max_speed = 0
    for __ in range(iteration_limit):
        next_max_speed, _ = safe_car.make_velo_bound_with_worst_case(
            v_n=max_speed, delta_next=0
        )
        absolute_error = abs(next_max_speed - max_speed)
        if absolute_error < 1e-4:
            return next_max_speed
        max_speed = next_max_speed

    raise ValueError("get_max_speed: iteration limit exceeded.")


def visualize_speed_bound(safe_car, animate_graph: bool = False):
    """state에 따른 속도 범위를 그래프로 그리는 함수.

    Args:
        safe_car (SafeCar): SafeCar 객체
        animate_graph (bool, optional) : 그래프를 애니메이션으로 보여줄지 여부. Defaults to False.
    """
    # 그래프 그리기
    # 각 v_n을 x축으로 놓고, delta_next를 y축으로 놓았을 때 bound_speed_next_step을 계산하고
    # z축에 놓은 그래프를 그린다.
    # (v_n, delta_next)의 범위는 각각 아래와 같이 설정한다.
    v_n_range = np.linspace(0, 10, 100)
    delta_next_range = np.linspace(-1.2, 1.2, 100)
    v_n_mesh, delta_next_mesh = np.meshgrid(v_n_range, delta_next_range)

    # worst case 없이 속도 범위 계산
    vmvbns = np.vectorize(safe_car.make_velo_bound_next_step)(v_n_mesh, delta_next_mesh)

    cmap = "rainbow"

    fig, ax = plt.subplots(1, 2, subplot_kw={"projection": "3d"}, figsize=(20, 10))  # type: ignore
    fig.suptitle("velocity bound of next step")  # type: ignore

    ax[0].contour(v_n_mesh, delta_next_mesh, vmvbns[0], levels=20, cmap=cmap)

    ax[0].plot_wireframe(v_n_mesh, delta_next_mesh, vmvbns[1])

    ax[0].set_title("velocity bound without worst case")
    ax[0].set_xlabel("$v_{n}$")
    ax[0].set_ylabel(r"$\delta_{\t{next}}$")
    ax[0].set_zlabel("velocity bound of next step")

    # worst case 고려한 속도 범위 계산
    vmvbwwc = np.vectorize(safe_car.make_velo_bound_with_worst_case)(
        v_n_mesh, delta_next_mesh
    )

    # 최대 속도 그래프 설정
    ax[1].contour(
        v_n_mesh,
        delta_next_mesh,
        vmvbwwc[0],
        levels=20,
        cmap=cmap,
    )
    # 최소 속도 그래프 설정
    ax[1].plot_wireframe(v_n_mesh, delta_next_mesh, vmvbwwc[1])
    # 그래프 축 이름 설정
    ax[1].set_title("velocity bound with worst case")
    ax[1].set_xlabel("$v_{n}$")
    ax[1].set_ylabel(r"$\delta_{\t{next}}$")
    ax[1].set_zlabel("velocity bound of next step")

    fig.tight_layout()

    if animate_graph:

        def animate(i):  # type: ignore
            ax[0].view_init(elev=10, azim=i)
            ax[1].view_init(elev=10, azim=i)
            return fig

        ani = animation.FuncAnimation(fig, animate, frames=360, interval=50)

        ani.save("image/velocity_bound.gif", writer="pillow", fps=30)  # type: ignore
    else:
        plt.show()  # type: ignore


if __name__ == "__main__":

    # 변수
    v_n_example = 2  # m/s
    delta_next_example = 0.1  # 라디안

    # 상수
    c_example = 2  # sqrt(L*mu*g)
    omega_example = 3.2  # rad/s
    dt_example = 0.05  # s

    select = input("py or cpp? ")
    if select == "py":
        from src.py import safe_region

        car = safe_region.SafeCar(c=c_example, omega=omega_example, dt=dt_example)
    else:
        from src.cpp import safe_region_cpp

        car = safe_region_cpp.SafeCar(c=c_example, omega=omega_example, dt=dt_example)

    # 하나의 state에 대해 범위를 구하는데 걸리는 시간 측정
    start = time.time()
    print(
        "Test-bound_speed_next_step :",
        car.make_velo_bound_with_worst_case(v_n_example, delta_next_example),
    )
    end = time.time()
    print(f"Elapsed time: {end - start} sec")

    print("max speed:", get_max_speed(car))

    # 그래프 그리기
    visualize_speed_bound(car, animate_graph=False)
