# Only with Ruckig Pro

from pathlib import Path
from sys import path

import numpy as np

# Path to the build directory including a file similar to 'ruckig.cpython-37m-x86_64-linux-gnu'.
build_path = Path(__file__).parent.absolute().parent / 'build'
path.insert(0, str(build_path))

from ruckig import Trackig, TargetState, InputParameter, OutputParameter


def model_ramp(t, ramp_vel=0.5, ramp_pos=1.0):
    target = TargetState(1)
    on_ramp = t < ramp_pos / (0.01 * abs(ramp_vel))
    target.position = [t * ramp_vel * 0.01] if on_ramp else [ramp_pos]
    target.velocity = [ramp_vel] if on_ramp else [0.0]
    target.acceleration = [0.0]
    return target

def model_constant_acceleration(t, ramp_acc=0.05):
    target = TargetState(1)
    target.position = [(0.01 * t)**2 * ramp_acc]
    target.velocity = [0.01 * t * ramp_acc]
    target.acceleration = [ramp_acc]
    return target

def model_sinus(t, ramp_vel=0.4):
    target = TargetState(1)
    target.position = [np.sin(ramp_vel * t * 0.01)]
    target.velocity = [ramp_vel * np.cos(ramp_vel * t * 0.01)]
    target.acceleration = [-ramp_vel * ramp_vel * np.sin(ramp_vel * t * 0.01)]
    return target


if __name__ == '__main__':
    inp = InputParameter(1)
    out = OutputParameter(inp.degrees_of_freedom)
    otg = Trackig(inp.degrees_of_freedom, 0.01)

    inp.current_position = [0.0]
    inp.current_velocity = [0.0]
    inp.current_acceleration = [0.0]

    inp.max_velocity = [0.8]
    inp.max_acceleration = [2.0]
    inp.max_jerk = [5.0]

    # otg.reactiveness = 1.0 # default value, should be in [0, 1]

    print('\t'.join(['t'] + [str(i) for i in range(otg.degrees_of_freedom)]))

    steps, target_list, follow_list = [], [], []
    for t in range(500):
        target_state = model_ramp(t)

        steps.append(t)
        res = otg.update(target_state, inp, out)

        out.pass_to_input(inp)

        print('\t'.join([f'{p:0.3f}' for p in target_state.position] + [f'{p:0.3f}' for p in out.new_position]) + f' in {out.calculation_duration:0.2f} [µs]')

        target_list.append([target_state.position, target_state.velocity, target_state.acceleration])
        follow_list.append([out.new_position, out.new_velocity, out.new_acceleration])


    # Plot the trajectory
    # import matplotlib.pyplot as plt

    # follow_list = np.array(follow_list)
    # target_list = np.array(target_list)
    
    # plt.ylabel(f'DoF 1')
    # plt.plot(steps, follow_list[:, 0])
    # plt.plot(steps, target_list[:, 0], color='r')
    # plt.grid(True)

    # plt.savefig(Path(__file__).parent.parent / 'build' / 'otg_trajectory.pdf')
