import matplotlib.pyplot as plt

def plot_variable(time_values, all_states, state_names, limits, index, subplot_index,  num_subplots_y, title, num_subplots_x= 1, y_limits=None):
    ax = plt.subplot(num_subplots_y, num_subplots_x, subplot_index)
    ax.plot(time_values, [(s[index]*limits[index]) for s in all_states], label=f'State {index + 1}')
    ax.set_xlabel('Time (s)')
    u_of_meas = ''
    if(state_names[index] == 'omega'):
        u_of_meas = 'rad/s'
    if(state_names[index] == 'torque'):
        u_of_meas = 'Nm'
    if(state_names[index] == 'i_a'):
        u_of_meas = 'A'
    if(state_names[index] == 'i_b'):
        u_of_meas = 'A'
    if(state_names[index] == 'i_c'):
        u_of_meas = 'A'
    if(state_names[index] == 'i_sd'):
        u_of_meas = 'A'
    if(state_names[index] == 'i_sq'):
        u_of_meas = 'A'
    if(state_names[index] == 'u_a'):
        u_of_meas = 'V'
    if(state_names[index] == 'u_b'):
        u_of_meas = 'V'
    if(state_names[index] == 'u_c'):
        u_of_meas = 'V'
    if(state_names[index] == 'u_sd'):
        u_of_meas = 'V'
    if(state_names[index] == 'u_sq'):
        u_of_meas = 'V'
    if(state_names[index] == 'epsilon'):
        u_of_meas = 'rad'
    if(state_names[index] == 'u_sup'):
        u_of_meas = 'V'
    ax.set_ylabel(state_names[index] + ' [' + u_of_meas + ']')
    ax.legend()
    if y_limits:
        ax.set_ylim(y_limits)
    ax.grid()

def plot_all_variables(time_values, all_states, state_names, limits, title="PMSM Simulation Results"):
    plt.figure(figsize=(12, 16))
    num_states = len(all_states[0])

    for i in range(num_states):
        plot_variable(time_values, all_states, state_names, limits, i, i + 1, num_states, title)

    plt.suptitle(title)
    plt.tight_layout(pad=0.5)
    plt.show()

def plot_omega_i_d_and_i_q(time_values, all_states, state_names, limits, title="PMSM Simulation Results"):
    plt.figure(figsize=(12, 16))
    # indices for omega, i_d, and i_q
    indices = [0, 5, 6]  

    for idx, state_index in enumerate(indices):
        plot_variable(time_values, all_states, state_names, limits, state_index, idx + 1, 3, title)

    plt.suptitle(title)
    plt.tight_layout(pad=0.5)
    plt.show()

def plot_torque_i_d_and_i_q(time_values, all_states, state_names, limits, title="PMSM Simulation Results"):
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
            plot_variable(time_values, all_states, state_names, limits, state_index, idx + 1, 3, title, y_limits_iq_id)
        else:
            plot_variable(time_values, all_states, state_names, limits, state_index, idx + 1, 3, title)

    plt.suptitle(title)
    plt.tight_layout(pad=0.5)
    plt.show()

def plot_omega_torque_i_d_and_i_q(time_values, all_states, state_names, limits, title="PMSM Simulation Results"):
    plt.figure(figsize=(12, 16))
    # indices for omega, torque, i_d, and i_q
    indices = [0, 1, 5, 6]  

    i_d_data = all_states[5] * limits[5]
    i_q_data = all_states[6] * limits[6]
    common_min = min(i_d_data.min(), i_q_data.min())
    common_max = max(i_d_data.max(), i_q_data.max())
    y_limits_iq_id = (common_min, common_max)

    for idx, state_index in enumerate(indices):
        if state_index in [5, 6]: # only setting common range for currents
            plot_variable(time_values, all_states, state_names, limits, state_index, idx + 1, 4, title, y_limits_iq_id)
        else:
            plot_variable(time_values, all_states, state_names, limits, state_index, idx + 1, 4, title)

    plt.suptitle(title)
    plt.tight_layout(pad=0.5)
    plt.show()


def plot_omega_torque_i_d_and_i_q_n_times(time_values_ph, all_states_ph, state_names, limits, num_of_meas, title="Prediction horizont effect"):
    plt.figure()
    for i in range(num_of_meas):
        all_states = all_states_ph[i]
        time_values = time_values_ph[i]


        # indices for omega, torque, i_d, and i_q
        indices = [0, 1, 5, 6]  

        i_d_data = all_states[5] * limits[5]
        i_q_data = all_states[6] * limits[6]
        common_min = min(i_d_data.min(), i_q_data.min())
        common_max = max(i_d_data.max(), i_q_data.max())
        y_limits_iq_id = (common_min, common_max)

        for idx, state_index in enumerate(indices):
            if state_index in [5, 6]: # only setting common range for currents
                plot_variable(time_values, all_states, state_names, limits, state_index, (idx*num_of_meas)+1+i, 4, title, num_subplots_x=num_of_meas, y_limits=y_limits_iq_id)
            else:
                plot_variable(time_values, all_states, state_names, limits, state_index, (idx*num_of_meas)+1+i, 4, title, num_subplots_x=num_of_meas)

    plt.suptitle(title)
    plt.tight_layout()
    plt.show()

