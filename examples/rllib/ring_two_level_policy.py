"""
Example script for use of two-level fully connected network policy,
using the single-lane ring road setting.
"""

import cloudpickle

import ray
import ray.rllib.ppo as ppo
from ray.tune.registry import get_registry, register_env as register_rllib_env


def to_subpolicy_state(inputs):
    return inputs


def choose_policy(inputs):
    return 0


if __name__ == "__main__":
    config = ppo.DEFAULT_CONFIG.copy()
    horizon = 500
    num_cpus=3
    ray.init(num_cpus=num_cpus, redirect_output=True)
    config["num_workers"] = num_cpus
    config["timesteps_per_batch"] = horizon * 3
    config["num_sgd_iter"] = 10
    config["gamma"] = 0.999
    config["horizon"] = horizon

    config["model"].update(
        {"fcnet_hiddens": [[5, 3]] * 2})
    config["model"]["user_data"] = {}
    config["model"]["user_data"].update({"num_subpolicies": 2,
                                         "fn_choose_subpolicy": list(
                                             cloudpickle.dumps(choose_policy))})
    # config["model"].update(
    #     {"num_subpolicies": 2, "fcnet_hiddens": [[5, 3]] * 2,
    #      "choose_policy": marshal.dumps(choose_policy.__code__)})
    # config["model"].update({"num_subpolicies": 2, "fcnet_hiddens": [[5,
    # 3]] * 2,
    #                         "to_subpolicy_state": to_subpolicy_state,
    #                         "choose_policy": choose_policy})

    from examples.rllib.stabilizing_the_ring import make_create_env
    flow_env_name = "WaveAttenuationPOEnv"
    env_name = flow_env_name+'-v0'
    # Register as rllib env
    create_env, env_name = make_create_env()
    register_rllib_env(env_name, create_env)

    alg = ppo.PPOAgent(env=env_name, registry=get_registry(), config=config)
    for i in range(2):
        alg.train()
        if i % 20 == 0:
            print("XXXX checkpoint path is", alg.save())