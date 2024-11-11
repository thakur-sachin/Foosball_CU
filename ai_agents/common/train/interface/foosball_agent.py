from abc import ABC, abstractmethod

class FoosballAgent(ABC):
    @abstractmethod
    def get_id(self):
        pass

    @abstractmethod
    def initialize_agent(self):
        pass

    @abstractmethod
    def predict(self, observation, deterministic=False):
        pass

    @abstractmethod
    def learn(self, total_timesteps):
        pass

    @abstractmethod
    def save(self):
        pass

    @abstractmethod
    def load(self):
        pass

    @abstractmethod
    def change_env(self, env):
        pass