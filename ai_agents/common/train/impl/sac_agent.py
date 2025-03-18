from stable_baselines3 import SAC
from ai_agents.common.train.interface.foosball_agent import FoosballAgent
from stable_baselines3.common.callbacks import EvalCallback
policy_kwargs = dict(net_arch=[512, 512, 512, 512])


class SACFoosballAgent(FoosballAgent):
    def __init__(self, id:int, env=None, log_dir='./logs', model_dir='./models'):
        self.env = env
        self.model = None
        self.id = id
        self.log_dir = log_dir
        self.model_dir = model_dir
        self.id_subdir = f'{model_dir}/{id}'

    def get_id(self):
        return self.id

    def initialize_agent(self):
        try:
            self.load()
        except Exception as e:
            print(f"Agent {self.id} could not load model. Initializing new model.")
            self.model = SAC('MlpPolicy', self.env, policy_kwargs=policy_kwargs, device='cuda', buffer_size=10000)
        print(f"Agent {self.id} initialized.")

    def predict(self, observation, deterministic=False):
        if self.model is None:
            raise ValueError("Model has not been initialized.")
        action, _ = self.model.predict(observation, deterministic=deterministic)
        return action

    def learn(self, total_timesteps):
        if self.model is None:
            self.model = SAC('MlpPolicy', self.env, policy_kwargs=policy_kwargs, buffer_size=10000)
        callback = self.create_callback(self.env)
        tb_log_name = f'sac_{self.id}'
        self.model.learn(total_timesteps=total_timesteps, callback=callback, tb_log_name=tb_log_name)

    def create_callback(self, env):
        eval_callback = EvalCallback(
            env,
            best_model_save_path=self.id_subdir + '/sac/best_model',
            log_path=self.log_dir,
            eval_freq=5000,
            n_eval_episodes=5,
            render=False,
        )
        return eval_callback

    def save(self):
        self.model.save(self.id_subdir + '/sac/best_model')

    def load(self):
        self.model = SAC.load(self.id_subdir + '/sac/best_model/best_model.zip')
        print(f"Agent {self.id} loaded model from {self.id_subdir}/sac/best_model/best_model.zip")

    def change_env(self, env):
        self.env = env
        self.model.set_env(env)