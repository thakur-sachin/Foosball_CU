from ai_agents.common.train.interface.agent_manager import AgentManager
from typing import List
from ai_agents.common.train.interface.training_engine import TrainingEngine

class ProtagonistAntagonistTrainingEngine(TrainingEngine):
    def __init__(
            self,
            agent_manager: AgentManager,
            environment_generator

    ):
        self.agent_manager = agent_manager
        self.current_epoch = 0
        self.best_models: List[str] = []
        self.num_agents_training = len(self.agent_manager.get_training_agents())
        self.environment_generator = environment_generator

    def train(self, total_epochs: int, epoch_timesteps: int, cycle_timesteps: int):
        for epoch in range(total_epochs):
            print(f"Starting epoch {epoch + 1}/{total_epochs}")
            for cycle in range(self.num_agents_training):
                print(f"Starting cycle {cycle + 1}/{self.num_agents_training}")

                ## Train the first protagonist agent for now.
                protagonist_agent = self.agent_manager.get_training_agents()[0]
                antagonist_agent = self.agent_manager.get_frozen_best_models()[cycle]

                env = self.environment_generator(antagonist_agent)
                protagonist_agent.change_env(env)
                protagonist_agent.learn(epoch_timesteps)

            self.current_epoch += 1


    def test(self, num_episodes: int = 10):
        protagonist = self.agent_manager.get_frozen_best_models()[0]
        env = self.environment_generator(protagonist)

        for episode in range(num_episodes):
            obs, _ = env.reset()
            done = False
            while not done:
                action = protagonist.predict(obs)
                obs, reward, terminated, truncated, info = env.step(action)
                env.render()
                done = terminated or truncated
            print(f"Episode {episode + 1}/{num_episodes} completed.")