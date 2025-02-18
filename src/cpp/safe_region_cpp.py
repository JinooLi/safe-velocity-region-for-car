#!/usr/bin/env python3
import build.safecar as safecar

# SafeCar 클래스를 Python에서 바로 사용 가능
car = safecar.SafeCar(c=2.0)


def makeVeloBoundWithWorstCase(v_n: float, delta_next: float):
    bound = car.makeVeloBoundWithWorstCase(v_n, delta_next)
    return bound.vMax, bound.vMin


def makeVeloBoundNextStep(v_n: float, delta_next: float):
    bound = car.makeVeloBoundNextStep(v_n, delta_next)
    return bound.vMax, bound.vMin


import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np


def visualize_speed_bound(animate: bool = False):
    """state에 따른 속도 범위를 그래프로 그리는 함수.

    Args:
        animate (bool, optional) : 그래프를 애니메이션으로 보여줄지 여부. Defaults to False.
    """
    # 그래프 그리기
    # 각 v_n을 x축으로 놓고, delta_next를 y축으로 놓았을 때 bound_speed_next_step을 계산하고
    # z축에 놓은 그래프를 그린다.
    # (v_n, delta_next)의 범위는 각각 아래와 같이 설정한다.
    v_n_range = np.linspace(0, 10, 100)
    delta_next_range = np.linspace(-1.2, 1.2, 100)
    v_n_mesh, delta_next_mesh = np.meshgrid(v_n_range, delta_next_range)

    # worst case 없이 속도 범위 계산
    vmvbns = np.vectorize(makeVeloBoundNextStep)(v_n_mesh, delta_next_mesh)

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
    vmvbwwc = np.vectorize(makeVeloBoundWithWorstCase)(v_n_mesh, delta_next_mesh)

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

    if animate:

        def animate(i):  # type: ignore
            ax[0].view_init(elev=10, azim=i)
            ax[1].view_init(elev=10, azim=i)
            return fig

        ani = animation.FuncAnimation(fig, animate, frames=360, interval=50)

        ani.save("image/velocity_bound.gif", writer="pillow", fps=30)  # type: ignore
    else:
        plt.show()  # type: ignore


if __name__ == "__main__":
    visualize_speed_bound(animate=False)
