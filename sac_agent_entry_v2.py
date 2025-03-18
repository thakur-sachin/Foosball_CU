import os
from ai_agents.common.train.impl.protagonist_antagonist_training_engine import ProtagonistAntagonistTrainingEngine
from ai_agents.common.train.impl.generic_agent_manager import GenericAgentManager
from ai_agents.common.train.impl.sac_agent import SACFoosballAgent
import sys
import argparse
from stable_baselines3.common.monitor import Monitor

from ai_agents.common.train.impl.single_player_training_engine import SinglePlayerTrainingEngine
from ai_agents.v2.gym.full_information_protagonist_antagonist_gym import FoosballEnv


def sac_foosball_env_factory(x=None):
    env = FoosballEnv(antagonist_model=None)
    env = Monitor(env)
    return env

if __name__ == '__main__':
    argparse = argparse.ArgumentParser(description='Train or test model.')
    argparse.add_argument('-t', '--test', help='Test mode', action='store_true')
    args = argparse.parse_args()


    model_dir = './models'
    log_dir = './logs'
    total_epochs = 15
    epoch_timesteps = int(100000)

    agent_manager = GenericAgentManager(1, sac_foosball_env_factory, SACFoosballAgent)
    agent_manager.initialize_training_agents()
    agent_manager.initialize_frozen_best_models()

    engine = SinglePlayerTrainingEngine(
        agent_manager=agent_manager,
        environment_generator=sac_foosball_env_factory
    )

    # Start training
    if not args.test:
        engine.train(total_epochs=total_epochs, epoch_timesteps=epoch_timesteps, cycle_timesteps=10000)

    # Test the trained agent
    engine.test()