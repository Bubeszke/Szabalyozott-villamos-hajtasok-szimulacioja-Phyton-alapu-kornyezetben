import matplotlib.pyplot as plt

def plot_variable(time_values, all_states, state_names, index, subplot_index, num_subplots, title, y_limits=None):
    ax = plt.subplot(num_subplots, 1, subplot_index)
    ax.plot(time_values, [s[index] for s in all_states], label=f'State {index + 1}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(state_names[index])
    ax.legend()
    if y_limits:
        ax.set_ylim(y_limits)

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
    # indices for omega, i_d, and i_q
    indices = [0, 5, 6]  

    for idx, state_index in enumerate(indices):
        plot_variable(time_values, all_states, state_names, state_index, idx + 1, 3, title)

    plt.suptitle(title)
    plt.tight_layout(pad=0.5)
    plt.show()

def plot_torque_i_d_and_i_q(time_values, all_states, state_names, title="PMSM Simulation Results"):
    plt.figure(figsize=(12, 16))
    # indices for torque, i_d, and i_q
    indices = [1, 5, 6]  

    i_d_data = all_states[5]
    i_q_data = all_states[6]
    common_min = min(i_d_data.min(), i_q_data.min())
    common_max = max(i_d_data.max(), i_q_data.max())
    y_limits_iq_id = (common_min, common_max)


    for idx, state_index in enumerate(indices):
        if state_index in [5, 6]: # only setting common range for currents
            plot_variable(time_values, all_states, state_names, state_index, idx + 1, 3, title, y_limits_iq_id)
        else:
            plot_variable(time_values, all_states, state_names, state_index, idx + 1, 3, title)

    plt.suptitle(title)
    plt.tight_layout(pad=0.5)
    plt.show()
