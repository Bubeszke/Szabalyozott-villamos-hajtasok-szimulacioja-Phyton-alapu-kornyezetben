import gym_electric_motor as gem
from gym_electric_motor.physical_systems import ConstantSpeedLoad, PolynomialStaticLoad
from gym_electric_motor.reference_generators import WienerProcessReferenceGenerator

def initialize_environment(config):
    motor_parameters = config.get("motor_parameters", {})
    env_id = config["env_id"]
    load_config = config.get("load", {})

    load = None
    load_type = load_config.get("type")
    load_parameters = load_config.get("load_parameters")

    if load_type == "ConstantSpeedLoad":
        if "omega_fixed" not in load_parameters:
            raise ValueError("omega_fixed must be specified for ConstantSpeedLoad")
        load = ConstantSpeedLoad(omega_fixed=(load_parameters["omega_fixed"] * 2 * 3.1416 / 60))
    elif load_type == "PolynomialStaticLoad":
        if not load_parameters:
            raise ValueError("Load parameters must be specified for PolynomialStaticLoad")
        load = PolynomialStaticLoad(load_parameter=load_parameters, limits=dict(omega=150.0))

    env = gem.make(
        env_id,
        motor_parameter=motor_parameters,
        reference_generator=WienerProcessReferenceGenerator(reference_state='i_sd', sigma_range=(0.001, 0.01)),
        load=load
    )

    return env
