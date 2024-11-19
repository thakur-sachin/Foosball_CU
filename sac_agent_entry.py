import os
from ai_agents.common.train.impl.protagonist_antagonist_training_engine import ProtagonistAntagonistTrainingEngine
from ai_agents.common.train.impl.generic_agent_manager import GenericAgentManager
from ai_agents.common.train.impl.sac_agent import SACFoosballAgent
#from ai_agents.v1.gym.full_information_protagonist_antagonist_gym import FoosballEnv
import sys
import argparse

from ai_agents.v1.gym.image_based_pa_gym import FoosballEnv

def sac_foosball_env_factory(antagonist_agent=None):
    env = FoosballEnv(antagonist_model=antagonist_agent)
    from stable_baselines3.common.monitor import Monitor
    env = Monitor(env)
    return env

if __name__ == '__main__':
    argparse = argparse.ArgumentParser(description='Train or test model.')
    argparse.add_argument('-t', '--test', help='Test mode', action='store_true')
    args = argparse.parse_args()


    model_dir = './models'
    log_dir = './logs'
    total_epochs = 10
    epoch_timesteps = int(1e6)

    agent_manager = GenericAgentManager(1, sac_foosball_env_factory, SACFoosballAgent)
    agent_manager.initialize_training_agents()
    agent_manager.initialize_frozen_best_models()

    engine = ProtagonistAntagonistTrainingEngine(
        agent_manager=agent_manager,
        environment_generator=sac_foosball_env_factory
    )

    # Start training
    if not args.test:
        engine.train(total_epochs=total_epochs, epoch_timesteps=epoch_timesteps, cycle_timesteps=10000)

    # Test the trained agent
    engine.test()