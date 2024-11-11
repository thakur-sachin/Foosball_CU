import gymnasium as gym
from gymnasium import spaces
import numpy as np
import mujoco
import os
import glfw

TABLE_MAX_Y_DIM = 10
BALL_STOPPED_COUNT_THRESHOLD = 100
SIM_PATH = os.environ.get('SIM_PATH', '/Research/Foosball_CU/foosball_sim/v1/foosball_sim.xml')

class FoosballEnv(gym.Env):
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
        self.action_space = spaces.Box(
            low=-1.0 * action_high, high=1.0 * action_high, dtype=np.float32
        )

        obs_dim = 38
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )

        self.viewer = None

        self._healthy_reward = 1.0
        self._ctrl_cost_weight = 0.1
        self._terminate_when_unhealthy = True
        self._healthy_z_range = (-10, 10)
        self.max_no_progress_steps = 600

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

        ball_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_free_joint')
        qpos_adr = self.model.jnt_qposadr[ball_joint_id]

        # Randomize ball position at reset
        self.data.qpos[qpos_adr:qpos_adr + 3] = np.random.normal(
            loc=[-0.5, 0.0, 0.55], scale=[0.2, 0.2, 0.0]
        )

        self.simulation_time = 0
        self.prev_ball_y = self.data.qpos[qpos_adr + 1]
        self.no_progress_steps = 0
        self.ball_stopped_count = 0

        return self._get_obs(), {}

    def step(self, protagonist_action):
        protagonist_action = np.clip(protagonist_action, self.action_space.low, self.action_space.high)

        antagonist_observation = self._get_antagonist_obs()

        if self.antagonist_model is not None:
            antagonist_action, _state = self.antagonist_model.predict(antagonist_observation)
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

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    def _get_obs(self):
        ball_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_free_joint')
        qpos_adr = self.model.jnt_qposadr[ball_joint_id]
        qvel_adr = self.model.jnt_dofadr[ball_joint_id]
        ball_pos = self.data.qpos[qpos_adr:qpos_adr + 3]
        ball_vel = self.data.qvel[qvel_adr:qvel_adr + 3]

        rod_slide_positions = []
        rod_slide_velocities = []
        rod_rotate_positions = []
        rod_rotate_velocities = []

        # Collect observations for both players' rods
        for player in ['A', 'B']:
            for i in range(1, 5):
                # Linear joints
                slide_joint_name = f'lin_{player}_{i}'
                slide_joint_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, slide_joint_name
                )
                slide_qpos_adr = self.model.jnt_qposadr[slide_joint_id]
                slide_qvel_adr = self.model.jnt_dofadr[slide_joint_id]
                rod_slide_positions.append(self.data.qpos[slide_qpos_adr])
                rod_slide_velocities.append(self.data.qvel[slide_qvel_adr])

                # Rotational joints
                rotate_joint_name = f'rev_{player}_{i}'
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

    def _get_antagonist_obs(self):
        """
        Get the observation for the antagonist (Player B).
        Includes positions and velocities of both Player B's and Player A's rods.
        Adjusts the observations to match the antagonist's perspective.
        """
        # Ball position and velocity
        ball_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_free_joint')
        qpos_adr = self.model.jnt_qposadr[ball_joint_id]
        qvel_adr = self.model.jnt_dofadr[ball_joint_id]

        # Get ball position and velocity
        ball_pos = self.data.qpos[qpos_adr:qpos_adr + 3].copy()
        ball_vel = self.data.qvel[qvel_adr:qvel_adr + 3].copy()

        # Reverse the x-coordinate to mirror the ball position and velocity
        ball_pos[0] *= -1
        ball_vel[0] *= -1

        rod_slide_positions = []
        rod_slide_velocities = []
        rod_rotate_positions = []
        rod_rotate_velocities = []

        # Collect observations for both players' rods, adjusting for antagonist's perspective
        for player in ['B', 'A']:  # Note the order: antagonist's rods first
            for i in range(1, 5):
                # Linear joints
                slide_joint_name = f'lin_{player}_{i}'
                slide_joint_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, slide_joint_name
                )
                slide_qpos_adr = self.model.jnt_qposadr[slide_joint_id]
                slide_qvel_adr = self.model.jnt_dofadr[slide_joint_id]
                slide_pos = self.data.qpos[slide_qpos_adr]
                slide_vel = self.data.qvel[slide_qvel_adr]

                if player == 'A':
                    # Reverse positions and velocities for opponent's rods
                    slide_pos = -slide_pos
                    slide_vel = -slide_vel

                else:
                    # For own rods, reverse as necessary to match training perspective
                    slide_pos = -slide_pos
                    slide_vel = -slide_vel

                rod_slide_positions.append(slide_pos)
                rod_slide_velocities.append(slide_vel)

                # Rotational joints
                rotate_joint_name = f'rev_{player}_{i}'
                rotate_joint_id = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, rotate_joint_name
                )
                rotate_qpos_adr = self.model.jnt_qposadr[rotate_joint_id]
                rotate_qvel_adr = self.model.jnt_dofadr[rotate_joint_id]
                rotate_pos = self.data.qpos[rotate_qpos_adr]
                rotate_vel = self.data.qvel[rotate_qvel_adr]

                if player == 'A':
                    # Reverse positions and velocities for opponent's rods
                    rotate_pos = -rotate_pos
                    rotate_vel = -rotate_vel

                else:
                    # For own rods, reverse as necessary to match training perspective
                    rotate_pos = -rotate_pos
                    rotate_vel = -rotate_vel

                rod_rotate_positions.append(rotate_pos)
                rod_rotate_velocities.append(rotate_vel)

        antagonist_obs = np.concatenate([
            ball_pos,
            ball_vel,
            rod_slide_positions,
            rod_slide_velocities,
            rod_rotate_positions,
            rod_rotate_velocities
        ])

        return antagonist_obs

    def _adjust_antagonist_action(self, antagonist_action):
        adjusted_action = -antagonist_action.copy()

        return adjusted_action

    def compute_reward(self, protagonist_action):
        """
        Compute the reward for the protagonist (Player A).
        """
        ball_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_free_joint'
        )
        qpos_adr = self.model.jnt_qposadr[ball_joint_id]
        ball_y = self.data.qpos[qpos_adr + 1]

        if self.prev_ball_y is not None:
            progress = (ball_y - self.prev_ball_y) * -1.0
        else:
            progress = 0.0
        self.prev_ball_y = ball_y

        if abs(progress) <= 0.01:
            self.no_progress_steps += 1
        else:
            self.no_progress_steps = 0

        progress_reward = progress * 10.0

        healthy_reward = self.healthy_reward
        ctrl_cost = self.control_cost(protagonist_action)

        victory = 10000 if ball_y < -1 *  TABLE_MAX_Y_DIM else 0  # Ball in antagonist's goal
        loss = -10000 if ball_y > TABLE_MAX_Y_DIM else 0  # Ball in protagonist's goal

        progress_penalty = -100 if self.no_progress_steps >= 10 else 0

        reward = loss + victory + progress_reward + healthy_reward - ctrl_cost + progress_penalty

        return reward

    @property
    def healthy_reward(self):
        return (
                float(self.is_healthy or self._terminate_when_unhealthy)
                * self._healthy_reward
        )

    def control_cost(self, action):
        control_cost = self._ctrl_cost_weight * np.sum(np.square(action))
        return control_cost

    @property
    def is_healthy(self):
        ball_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_free_joint'
        )
        qpos_adr = self.model.jnt_qposadr[ball_joint_id]
        ball_z = self.data.qpos[qpos_adr + 2]

        min_z, max_z = self._healthy_z_range
        is_healthy = min_z < ball_z < max_z

        return is_healthy

    def _is_ball_moving(self):
        ball_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_free_joint'
        )
        qvel_adr = self.model.jnt_dofadr[ball_joint_id]
        ball_vel = self.data.qvel[qvel_adr:qvel_adr + 3]

        return np.linalg.norm(ball_vel) > 0.5

    @property
    def terminated(self):
        self.ball_stopped_count = 0 if self._is_ball_moving() else self.ball_stopped_count + 1
        ball_stagnant = self.ball_stopped_count >= BALL_STOPPED_COUNT_THRESHOLD

        unhealthy = not self.is_healthy
        no_progress = self.no_progress_steps >= self.max_no_progress_steps

        ball_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_free_joint'
        )
        qpos_adr = self.model.jnt_qposadr[ball_joint_id]
        ball_y = self.data.qpos[qpos_adr + 1]
        ball_x = self.data.qpos[qpos_adr]
        victory = ball_y < -1 * TABLE_MAX_Y_DIM or ball_y > TABLE_MAX_Y_DIM  # Ball in any goal

        terminated = (
                unhealthy or (no_progress and not self.play_until_goal) or victory or ball_stagnant
        ) if self._terminate_when_unhealthy else False

        if self.verbose_mode and terminated:
            print("Terminated")
            print(f"Unhealthy: {unhealthy}, No progress: {no_progress}, Victory: {victory}, Ball stagnant: {ball_stagnant}")
            print("x: ", ball_x, "y: ", ball_y)
        return terminated

    def seed(self, seed=None):
        np.random.seed(seed)

    def render(self, mode='human'):
        if self.viewer is None:
            if not glfw.init():
                raise Exception("Could not initialize GLFW")

            self.window = glfw.create_window(800, 600, "Foosball Simulation", None, None)
            if not self.window:
                glfw.terminate()
                raise Exception("Could not create GLFW window")

            glfw.make_context_current(self.window)

            self.cam = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(self.cam)
            self.cam.azimuth = 180.0
            self.cam.elevation = -70.0
            self.cam.distance = 25.0
            self.cam.lookat[:] = np.array([2, 0, 0])

            self.opt = mujoco.MjvOption()
            mujoco.mjv_defaultOption(self.opt)
            self.scn = mujoco.MjvScene(self.model, maxgeom=1000)

            self.ctx = mujoco.MjrContext(
                self.model, mujoco.mjtFontScale.mjFONTSCALE_150
            )

            self.viewer = True  # Flag to indicate that the viewer is initialized

        if not glfw.window_should_close(self.window):
            mujoco.mjv_updateScene(
                self.model,
                self.data,
                self.opt,
                None,
                self.cam,
                mujoco.mjtCatBit.mjCAT_ALL,  # 'catmask' argument
                self.scn
            )

            viewport_width, viewport_height = glfw.get_framebuffer_size(self.window)
            viewport = mujoco.MjrRect(0, 0, viewport_width, viewport_height)

            mujoco.mjr_render(viewport, self.scn, self.ctx)

            glfw.swap_buffers(self.window)
            glfw.poll_events()
        else:
            self.close()

    def close(self):
        if self.viewer is not None:
            glfw.destroy_window(self.window)
            glfw.terminate()
            self.viewer = None
