from copy import copy

from ruckig import InputParameter, OutputParameter, Result, Ruckig, ControlInterface, Trajectory


if __name__ == "__main__":
    # Create instances: the Ruckig OTG as well as input and output parameters
    otg = Ruckig(1)  # DoFs, control cycle
    inp = InputParameter(1)

    inp.control_interface = ControlInterface.Velocity

    inp.current_velocity = [0.0]

    inp.target_velocity = [5]
    inp.target_acceleration = [inp.target_velocity[0] * 2]

    inp.max_acceleration = [inp.target_velocity[0] * 2]
    inp.max_jerk = [inp.target_velocity[0] * 4]

    print("\t".join(["t"] + [str(i) for i in range(otg.degrees_of_freedom)]))

    trajectory = Trajectory(1)
    result = otg.calculate(inp, trajectory)
    if result == Result.ErrorInvalidInput:
        raise Exception("Invalid input!")

    print(f"Trajectory duration: {trajectory.duration:0.4f} [s]")

    new_time = 0.75

    # Then, we can calculate the kinematic state at a given time
    new_position, new_velocity, new_acceleration = trajectory.at_time(new_time)

    print(f"Position at time {new_time:0.4f} [s]: {new_velocity}")