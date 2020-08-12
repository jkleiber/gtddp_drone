#! /usr/bin/python

import csv
import os
import subprocess32
import pickle
import sys

import matplotlib.pyplot as plt
import numpy as np

from termcolor

NUM_RUNS = 50

# Cart-Pole Simulation files
home_directory = os.path.expanduser("~")
sim_log = home_directory + "/cart_pole_state.csv"
raw_log = home_directory + "/monte_carlo_cp_raw.pkl"
result_log = home_directory + "/monte_carlo_cp_result.csv"

# List of trajectories
current_results = []

def cart_pole_runner():
    global current_results

    # Run the simulation N times
    for i in range(NUM_RUNS):
        # Run the simulator
        subprocess32.call("roslaunch gtddp_drone cart_pole_sim.launch", shell=True)
        # os.system("roslaunch gtddp_drone cart_pole_sim.launch")

        # Load the results from the CSV
        result_frame = np.loadtxt(open(sim_log, 'rb'), delimiter=',')

        # Add to current results
        current_results.append(result_frame)
        print(result_frame)

        # Save current raw results as a pickle
        pickle.dump(np.asarray(current_results), open(raw_log, 'w'))


def load_pickle():
    global current_results

    # Deserialize the pickle
    current_results = pickle.load(open(raw_log, 'rb'))


def process_results():
    """
    Average the performances into a single trajectory
    """
    # Convert to 3D array
    state_cube = np.asarray(current_results)

    ### Calculate average for each recorded point
    # Find data-cube params
    num_frames = len(state_cube)
    num_cols = len(state_cube[0][0, :])

    # Find the number of rows per frame
    rows = []
    for i in range(num_frames):
        rows.append(len(state_cube[i]))

    # Constrain the number of rows to the minimum length.
    # This effectively truncates longer runs, which shouldn't matter
    # for our purposes where convergence is quick.
    num_rows = min(rows)

    # Compute the mean trajectory row-by-row
    current_row = 0
    mean_traj = np.ndarray((num_rows, num_cols))
    for row in range(num_rows):
        total = np.ndarray((1, 5))
        for i in range(len(state_cube)):
            for col in range(num_cols):
                total[0, col] += state_cube[i][row, col]

        # Add the mean to the working trajectory
        mean_row = np.divide(total, num_frames)
        mean_traj[row, :] = mean_row

    # Get the mean trajectory as a numpy array
    mean_traj = np.asarray(mean_traj)

    # Save the result to CSV
    np.savetxt(result_log, mean_traj, delimiter=',')

    ### Plot
    # Gather arrays
    t = mean_traj[:, 0]
    x = mean_traj[:, 1]
    x_dot = mean_traj[:, 2]
    theta = mean_traj[:, 3]
    theta_dot = mean_traj[:, 4]

    fig, axs = plt.subplots(2, 2)
    axs[0, 0].plot(t, x)
    axs[0, 0].set_title("x")
    axs[0, 1].plot(t, x_dot)
    axs[0, 1].set_title("x dot")
    axs[1, 0].plot(t, theta)
    axs[1, 0].set_title("theta")
    axs[1, 1].plot(t, theta_dot)
    axs[1, 1].set_title("theta dot")

    plt.show()


if __name__ == "__main__":
    # Check the mode from command line args
    mode = "monte-carlo"
    if len(sys.argv) > 1:
        mode = sys.argv[1]

    # Run the monte-carlo simulation
    if mode == "monte-carlo":
        cart_pole_runner()
    # Load an old run from a pickle file
    elif mode == "offline-data":
        load_pickle()
    else:
        print("Invalid arguments! Valid args are 'monte-carlo' and 'offline-data'")

    # plot the results
    process_results()
