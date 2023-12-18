from copy import copy

from ruckig import InputParameter, OutputParameter, Result, Ruckig


if __name__ == "__main__":
    # Create instances: the Ruckig OTG as well as input and output parameters
    otg = Ruckig(1, 0.005)  # DoFs, control cycle
    inp = InputParameter(1)
    out = OutputParameter(1)
    
    # Set input parameters
    inp.current_position = [0.6]
    inp.current_velocity = [1.8]
    inp.current_acceleration = [1.5]

    inp.target_position = [2.6]
    inp.target_velocity = [0.0]
    inp.target_acceleration = [0.0]

    inp.max_velocity = [2.0]
    inp.max_acceleration = [4.0]
    inp.max_jerk = [8.0]

    print("\t".join(["t"] + [str(i) for i in range(otg.degrees_of_freedom)]))

    # Generate the trajectory within the control loop
    first_output, out_list = None, []
    res = Result.Working
    while res == Result.Working:
        res = otg.update(inp, out)

        print("\t".join([f"{out.time:0.3f}"] + [f"{p:0.3f}" for p in out.new_position]))
        out_list.append(copy(out))

        out.pass_to_input(inp)

        if not first_output:
            first_output = copy(out)

    print(f"Calculation duration: {first_output.calculation_duration:0.1f} [µs]")
    print(f"Trajectory duration: {first_output.trajectory.duration:0.4f} [s]")

    # Plot the trajectory
    from pathlib import Path
    from plotter import Plotter

    project_path = Path(__file__).parent.parent.absolute()
    Plotter.plot_trajectory(project_path / 'examples' / '01_trajectory_dof1.pdf', otg, inp, out_list, plot_jerk=False)
