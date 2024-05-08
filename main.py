import json
from modules.env_setup import *
from modules.controller import *
from modules.simulation_runner import *
from modules.visualization import *
from modules.reference_generator import * 

try:
    with open('config/config.json', 'r') as config_file:
        config = json.load(config_file)

    env = initialize_environment(config)
    if not env:
        raise ValueError("Failed to initialize the environment")

    Controller = PID_Controller(env, config.get('tau'))
    if not Controller:
        raise ValueError("Failed to initialize the controller")

    references = initialize_references(config)
    if references is None:
        raise ValueError("Failed to initialize references")

    time_values, all_states = run_simulation(env, Controller, references, config['num_steps'], config['tau'])
    if time_values is None or all_states is None:
        raise ValueError("Simulation failed")

    # Plot results if simulation is successful
    plot_all_variables(time_values, all_states, env.state_names)
    #plot_torque_i_d_and_i_q(time_values, all_states, env.state_names)
    #plot_omega_i_d_and_i_q(time_values, all_states, env.state_names)

except Exception as e:
    print(f"An error occurred: {e}")
