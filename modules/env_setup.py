import gym_electric_motor as gem
from gym_electric_motor.physical_systems import ConstantSpeedLoad, PolynomialStaticLoad
from gym_electric_motor.reference_generators import WienerProcessReferenceGenerator

def initialize_environment(config, omega_fixed_temp):
    motor = config.get("motor", {})
    env_id = config["env_id"]
    load_config = config.get("load", {})

    load = None
    load_type = load_config.get("type")
    load_parameters = load_config.get("load_parameters")

    if load_type == "ConstantSpeedLoad":
        if "omega_fixed" not in load_parameters:
            raise ValueError("omega_fixed must be specified for ConstantSpeedLoad")
        #load = ConstantSpeedLoad(omega_fixed=(load_parameters["omega_fixed"]))
        load = ConstantSpeedLoad(omega_fixed=omega_fixed_temp)
    else:
        raise ValueError("Load type must be specified for ConstantSpeedLoad")

    env = gem.make(
        env_id,
        motor=motor,
        load=load
    )

    return env
