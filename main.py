import json
from modules.env_setup import *
from modules.controller import *
from modules.simulation_runner import *
from modules.visualization import *
from modules.reference_generator import * 

ERRORHANDLING_ON = True # Only for in depth debugging is false needed
if(ERRORHANDLING_ON):
    try:
        with open('config/config.json', 'r') as config_file:
            config = json.load(config_file)
        print('CONFIG LOADED')

        env = initialize_environment(config)
        tau = env.physical_system.tau
        print(tau)
        if not env:
            raise ValueError("Failed to initialize the environment")
        print('initialize_environment')

        Controller = MPC_Controller(env, tau)
        if not Controller:
            raise ValueError("Failed to initialize the controller")
        print('MPC_Controller')

        references = initialize_references(config)
        if references is None:
            raise ValueError("Failed to initialize references")
        print('initialize_references')

        time_values, all_states = run_simulation(env, Controller, references, config['num_steps'], tau)
        if time_values is None or all_states is None:
            raise ValueError("Simulation failed")

        # Plot results if simulation is successful
        plot_all_variables(time_values, all_states, env.state_names)
        # plot_torque_i_d_and_i_q(time_values, all_states, env.state_names)
        # plot_omega_i_d_and_i_q(time_values, all_states, env.state_names)

    except Exception as e:
        print(f"An error occurred: {e}")
