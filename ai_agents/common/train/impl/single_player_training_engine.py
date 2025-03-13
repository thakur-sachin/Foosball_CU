from ai_agents.common.train.interface.agent_manager import AgentManager
from typing import List
from ai_agents.common.train.interface.training_engine import TrainingEngine

class SinglePlayerTrainingEngine(TrainingEngine):
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
            protagonist_agents = self.agent_manager.get_training_agents()
            self.agent_manager.initialize_frozen_best_models()
            antagonist_agents = self.agent_manager.get_frozen_best_models()

            protagonist_agent = protagonist_agents[0]
            env = self.environment_generator()
            protagonist_agent.change_env(env)
            protagonist_agent.learn(epoch_timesteps)

            self.current_epoch += 1


    def test(self, num_episodes: int = 100):
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