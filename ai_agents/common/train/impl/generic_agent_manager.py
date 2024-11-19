from ai_agents.common.train.impl.sac_agent import SACFoosballAgent
from ai_agents.common.train.interface.agent_manager import AgentManager

class GenericAgentManager(AgentManager):
    def __init__(self, num_agents: int, environment_generator, agent_class):
        self.num_agents = num_agents
        self.environment_generator = environment_generator
        self.initial_env = environment_generator()
        self.agent_class = agent_class
        self.training_agents = []
        self.frozen_best_models = []

    def initialize_training_agents(self):
        for i in range(self.num_agents):
            agent = self.agent_class(id=i, env=self.initial_env)
            agent.initialize_agent()
            self.training_agents.append(agent)

    def save_training_agents(self):
        for agent in self.training_agents:
            agent.save()

    def initialize_frozen_best_models(self):
        for i in range(self.num_agents):
            agent = self.agent_class(id=i, env=self.initial_env)
            agent.initialize_agent()
            self.frozen_best_models.append(agent)

    def get_training_agents(self):
        return self.training_agents

    def get_frozen_best_models(self):
        return self.frozen_best_models

    def set_agent_environment(self, id, env):
        self.training_agents[id].change_env(env)

    def set_training_agent(self, agent):
        self.training_agents.append(agent)

