import os
from ai_agents.common.train.impl.protagonist_antagonist_training_engine import ProtagonistAntagonistTrainingEngine
from ai_agents.common.train.impl.sac_agent_manager import SACAgentManager
from ai_agents.v1.gym.full_information_protagonist_antagonist_gym import FoosballEnv


def sac_foosball_env_factory(antagonist_agent=None):
    env = FoosballEnv(antagonist_model=antagonist_agent)
    from stable_baselines3.common.monitor import Monitor
    env = Monitor(env)
    return env

if __name__ == '__main__':
    model_dir = './models'
    log_dir = './logs'
    total_epochs = 10
    epoch_timesteps = int(1e6)

    agent_manager = SACAgentManager(5, sac_foosball_env_factory)
    agent_manager.initialize_training_agents()

    engine = ProtagonistAntagonistTrainingEngine(
        agent_manager=agent_manager,
        environment_generator=sac_foosball_env_factory
    )

    # Start training
    engine.train(total_epochs=total_epochs, epoch_timesteps=epoch_timesteps)

    # Test the trained agent
    engine.test(num_episodes=5)