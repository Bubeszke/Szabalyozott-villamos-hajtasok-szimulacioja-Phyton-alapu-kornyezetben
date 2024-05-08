import matplotlib.pyplot as plt

def plot_variable(time_values, all_states, state_names, index, subplot_index, num_subplots, title):
    plt.subplot(num_subplots, 1, subplot_index)
    plt.plot(time_values, [s[index] for s in all_states])
    plt.xlabel('Time (s)')
    plt.ylabel(state_names[index])
    plt.legend([f'State {index + 1}'])

def plot_all_variables(time_values, all_states, state_names, title="PMSM Simulation Results"):
    plt.figure(figsize=(12, 16))
    num_states = len(all_states[0])

    for i in range(num_states):
        plot_variable(time_values, all_states, state_names, i, i + 1, num_states, title)

    plt.suptitle(title)
    plt.tight_layout(pad=0.5)
    plt.show()

def plot_omega_i_d_and_i_q(time_values, all_states, state_names, title="PMSM Simulation Results"):
    plt.figure(figsize=(12, 16))
    indices = [0, 5, 6]  # indices for omega, i_d, and i_q

    for idx, state_index in enumerate(indices):
        plot_variable(time_values, all_states, state_names, state_index, idx + 1, 3, title)

    plt.suptitle(title)
    plt.tight_layout(pad=0.5)
    plt.show()

def plot_torque_i_d_and_i_q(time_values, all_states, state_names, title="PMSM Simulation Results"):
    plt.figure(figsize=(12, 16))
    indices = [1, 5, 6]  # indices for torque, i_d, and i_q

    for idx, state_index in enumerate(indices):
        plot_variable(time_values, all_states, state_names, state_index, idx + 1, 3, title)

    plt.suptitle(title)
    plt.tight_layout(pad=0.5)
    plt.show()
