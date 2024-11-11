from abc import ABC, abstractmethod

class AgentManager(ABC):
    @abstractmethod
    def get_training_agents(self):
        pass

    @abstractmethod
    def get_frozen_best_models(self):
        pass
