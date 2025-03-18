import gymnasium as gym
from gymnasium import spaces
import numpy as np
import mujoco
import os
import glfw

from ai_agents.v2.gym.mujoco_table_render_mixin import MujocoTableRenderMixin

DIRECTION_CHANGE = 1
TABLE_MAX_Y_DIM = 70
BALL_STOPPED_COUNT_THRESHOLD = 20
MAX_STEPS = 50
SIM_PATH = os.environ.get('SIM_PATH', '/Research/Foosball_CU/foosball_sim/v2/foosball_sim.xml')

RODS = ["_goal_", "_def_", "_mid_", "_attack_"]

class FoosballEnv( MujocoTableRenderMixin, gym.Env, ):
    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(self, antagonist_model=None, play_until_goal=False, verbose_mode=False):
        super(FoosballEnv, self).__init__()

        dir_path = os.path.dirname(os.path.realpath(__file__))
        xml_file = SIM_PATH

        self.model = mujoco.MjModel.from_xml_path(xml_file)
        self.data = mujoco.MjData(self.model)

        self.simulation_time = 0

        self.num_rods_per_player = 4
        self.num_players = 2
        self.num_rods = self.num_rods_per_player * self.num_players  # Total rods

        self.protagonist_action_size = self.num_rods_per_player * 2  # 8 actions for protagonist
        self.antagonist_action_size = self.num_rods_per_player * 2   # 8 actions for antagonist

        action_high = np.ones(self.protagonist_action_size)
        self.rotation_action_space = spaces.Box(
            low=-2.5 * action_high, high=2.5 * action_high, dtype=np.float32
        )

        self.goal_linear_action_space = spaces.Box(
            low=-10.0 * action_high, high=10.0 * action_high, dtype=np.float32
        )
        self.def_linear_action_space = spaces.Box(
            low=-20.0 * action_high, high=20.0 * action_high, dtype=np.float32
        )
        self.mid_linear_action_space = spaces.Box(
            low=-7.0 * action_high, high=7.0 * action_high, dtype=np.float32
        )
        self.attack_linear_action_space = spaces.Box(
            low=-12.0 * action_high, high=12.0 * action_high, dtype=np.float32
        )

        # TEMP
        self.action_space = spaces.Box(
            low=-20 * action_high, high=20 * action_high, dtype=np.float32
        )

        obs_dim = 38
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )

        self.viewer = None

        self._healthy_reward = 1.0
        self._ctrl_cost_weight = 0.1
        self._terminate_when_unhealthy = True
        self._healthy_z_range = (-80, 80)
        self.max_no_progress_steps = 20

        self.prev_ball_y = None
        self.no_progress_steps = 0
        self.ball_stopped_count = 0

        self.antagonist_model = antagonist_model
        self.play_until_goal = play_until_goal
        self.verbose_mode = verbose_mode

    def set_antagonist_model(self, antagonist_model):
        self.antagonist_model = antagonist_model

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)

        ball_x_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_x')
        ball_y_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_y')

        x_qpos_adr = self.model.jnt_qposadr[ball_x_id]
        y_qpos_adr = self.model.jnt_qposadr[ball_y_id]

        xy_random = np.random.normal(
            loc=[-0.5, 0.0],
            scale=[0.5, 0.5]
        )

        self.data.qpos[x_qpos_adr] = xy_random[0]
        self.data.qpos[y_qpos_adr] = xy_random[1]

        self.simulation_time = 0
        self.prev_ball_y = self.data.qpos[y_qpos_adr]
        self.no_progress_steps = 0
        self.ball_stopped_count = 0

        return self._get_obs(), {}

    def step(self, protagonist_action):
        protagonist_action = np.clip(protagonist_action, self.action_space.low, self.action_space.high)

        antagonist_observation = self._get_antagonist_obs()

        if self.antagonist_model is not None:
            antagonist_action = self.antagonist_model.predict(antagonist_observation)
            antagonist_action = np.clip(antagonist_action, -1.0, 1.0)

            antagonist_action = self._adjust_antagonist_action(antagonist_action)
        else:
            antagonist_action = np.zeros(self.antagonist_action_size)

        self.data.ctrl[:self.protagonist_action_size] = protagonist_action
        self.data.ctrl[self.protagonist_action_size:self.protagonist_action_size + self.antagonist_action_size] = antagonist_action

        mujoco.mj_step(self.model, self.data)
        self.simulation_time += self.model.opt.timestep

        obs = self._get_obs()

        reward = self.compute_reward(protagonist_action)
        terminated = self.terminated

        info = {}

        return obs, reward, terminated, False, info

    def _get_ball_obs(self):
        ball_x_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_x')
        ball_y_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_y')
        ball_z_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_z')
        x_qpos_adr = self.model.jnt_qposadr[ball_x_id]
        y_qpos_adr = self.model.jnt_qposadr[ball_y_id]
        x_qvel_adr = self.model.jnt_dofadr[ball_x_id]
        y_qvel_adr = self.model.jnt_dofadr[ball_y_id]
        z_qvel_adr = self.model.jnt_dofadr[ball_z_id]
        ball_pos = [
            self.data.qpos[x_qpos_adr],
            self.data.qpos[y_qpos_adr],
            self.data.qpos[ball_z_id]
        ]
        ball_vel = [
            self.data.qvel[x_qvel_adr],
            self.data.qvel[y_qvel_adr],
            self.data.qvel[z_qvel_adr]
        ]

        return ball_pos, ball_vel

    def _get_antagonist_obs(self):
        return None

    def _get_obs(self):
        ball_pos, ball_vel = self._get_ball_obs()

        rod_slide_positions = []
        rod_slide_velocities = []
        rod_rotate_positions = []
        rod_rotate_velocities = []

        # Collect observations for both players' rods
        for player in ['y', 'b']:
            for rod in RODS:
                # Linear joints
                slide_joint_name = f"{player}{rod}linear"
                slide_joint_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, slide_joint_name
                )
                slide_qpos_adr = self.model.jnt_qposadr[slide_joint_id]
                slide_qvel_adr = self.model.jnt_dofadr[slide_joint_id]
                rod_slide_positions.append(self.data.qpos[slide_qpos_adr])
                rod_slide_velocities.append(self.data.qvel[slide_qvel_adr])

                # Rotational joints
                rotate_joint_name = f"{player}{rod}rotation"
                rotate_joint_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, rotate_joint_name
                )
                rotate_qpos_adr = self.model.jnt_qposadr[rotate_joint_id]
                rotate_qvel_adr = self.model.jnt_dofadr[rotate_joint_id]
                rod_rotate_positions.append(self.data.qpos[rotate_qpos_adr])
                rod_rotate_velocities.append(self.data.qvel[rotate_qvel_adr])

        obs = np.concatenate([
            ball_pos,
            ball_vel,
            rod_slide_positions,
            rod_slide_velocities,
            rod_rotate_positions,
            rod_rotate_velocities
        ])

        assert obs.shape == self.observation_space.shape, (
            f"Observation shape {obs.shape} does not match observation space shape {self.observation_space.shape}"
        )

        return obs

    def _adjust_antagonist_action(self, antagonist_action):
        adjusted_action = -antagonist_action.copy()

        return adjusted_action

    def compute_reward(self, protagonist_action):
        """
        Compute the reward for the protagonist (Player A).
        """
        ball_y = self._get_ball_obs()[0][1]
        ball_x =

        ctrl_cost = self.control_cost(protagonist_action)

        victory = 10000 * DIRECTION_CHANGE if ball_y >  TABLE_MAX_Y_DIM else 0  # Ball in antagonist's goal
        loss = -10000 * DIRECTION_CHANGE if ball_y < -1.0 * TABLE_MAX_Y_DIM else 0  # Ball in protagonist's goal

        reward = loss + victory +  ctrl_cost + ball_y

        return reward

    @property
    def healthy_reward(self):
        return (
                float(self.is_healthy or self._terminate_when_unhealthy)
                * self._healthy_reward
        )

    def control_cost(self, action):
        # 2-norm
        #control_cost = self._ctrl_cost_weight * np.sum(np.square(action)) * -1.0

        # 1-norm
        control_cost = self._ctrl_cost_weight * np.sum(np.abs(action)) * -1.0

        # L0 norm
        #control_cost = self._ctrl_cost_weight * np.count_nonzero(action) * -1.0

        return control_cost

    @property
    def is_healthy(self):
        ball_z = self._get_ball_obs()[0][2]

        min_z, max_z = self._healthy_z_range
        is_healthy = min_z < ball_z < max_z

        return is_healthy

    def _is_ball_moving(self):
        ball_pos, ball_vel = self._get_ball_obs()

        return np.linalg.norm(ball_vel) > 0.5

    def _determine_progression(self):
        ball_y = self._get_ball_obs()[0][1]

        if self.prev_ball_y is not None:
            if ball_y > self.prev_ball_y:
                self.no_progress_steps = 0
            else:
                self.no_progress_steps += 1

        self.prev_ball_y = ball_y

    @property
    def terminated(self):
        self._determine_progression()

        self.ball_stopped_count = 0 if self._is_ball_moving() else self.ball_stopped_count + 1
        ball_stagnant = self.ball_stopped_count >= BALL_STOPPED_COUNT_THRESHOLD

        over_max_steps = self.simulation_time >= MAX_STEPS

        unhealthy = not self.is_healthy
        no_progress = self.no_progress_steps >= self.max_no_progress_steps

        ball_y = self._get_ball_obs()[0][1]
        ball_x = self._get_ball_obs()[0][0]

        victory = ball_y < -1 * TABLE_MAX_Y_DIM or ball_y > TABLE_MAX_Y_DIM  # Ball in any goal

        if victory:
            print("Victory")
            print(f"Ball x: {ball_x}, Ball y: {ball_y}")

        terminated = (
                unhealthy or (no_progress and not self.play_until_goal) or victory or ball_stagnant or over_max_steps
        ) if self._terminate_when_unhealthy else False

        if self.verbose_mode and terminated:
            print("Terminated")
            print(f"Unhealthy: {unhealthy}, No progress: {no_progress}, Victory: {victory}, Ball stagnant: {ball_stagnant}")
            print("x: ", ball_x, "y: ", ball_y)
        return terminated