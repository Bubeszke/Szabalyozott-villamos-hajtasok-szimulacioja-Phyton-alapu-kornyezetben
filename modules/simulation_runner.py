

def run_simulation(env, controller, references, num_steps, tau):
    time_values = []
    all_states = []

    user_reference = references
    terminated = True
    for t in range(num_steps):
        if terminated:
            step_response = env.reset()
            state = step_response[0][0]
            reference = step_response[0][1]
        time_values.append(t * tau)
        action = controller.control(state, user_reference)
        step_response =  env.step(action) # simulate one step of the PMSM

#       ???????????????????????????????
#        (state, reference), reward, done, terminated = step_response[0]

        state = step_response[0][0]
        reference = step_response[0][1]
        rewards = step_response[1]
        terminated = step_response[2]

        all_states.append(state)

    return time_values, all_states
