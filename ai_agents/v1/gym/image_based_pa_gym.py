import glfw
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import mujoco
import os
from PIL import Image

from ai_agents.v1.gym.mujoco_table_render_mixin import MujocoTableRenderMixin

DIRECTION_CHANGE = 1
TABLE_MAX_Y_DIM = 10
BALL_STOPPED_COUNT_THRESHOLD = 80
SIM_PATH = os.environ.get('SIM_PATH', '/Research/Foosball_CU/foosball_sim/v1/foosball_sim.xml')

# self.cam.azimuth = 180.0
# self.cam.elevation = -70.0
# self.cam.distance = 25.0
# self.cam.lookat[:] = np.array([2, 0, 0])
class CameraConfig:
    def __init__(self,
                 camera_id=None,
                 camera_name=None,
                 distance=25.0,
                 azimuth=180.0,
                 elevation=-70.0,
                 lookat=None,
                 tracking_point=None):
        self.camera_id = camera_id
        self.camera_name = camera_name
        self.distance = distance
        self.azimuth = azimuth
        self.elevation = elevation
        self.lookat = lookat if lookat is not None else [0.0, 0.0, 0.5]
        self.tracking_point = tracking_point

class FoosballEnv(gym.Env):
    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(
            self,
            antagonist_model,
            image_width=260,
            image_height=180,
            camera_config=None,
            play_until_goal=False,
            verbose_mode=False
    ):
        super(FoosballEnv, self).__init__()

        dir_path = os.path.dirname(os.path.realpath(__file__))
        xml_file = os.environ.get('SIM_PATH', '/Research/Foosball_CU/foosball_sim/v1/foosball_sim.xml')
        self.model = mujoco.MjModel.from_xml_path(xml_file)
        self.data = mujoco.MjData(self.model)

        self.image_width = image_width
        self.image_height = image_height

        self.camera_config = camera_config if camera_config else CameraConfig()

        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        self.cam = mujoco.MjvCamera()
        self.opt = mujoco.MjvOption()

        self.renderer = mujoco.Renderer(self.model, self.image_height, self.image_width)

        self.update_camera()

        self.num_rods_per_player = 4
        self.num_players = 2
        self.protagonist_action_size = self.num_rods_per_player * 2
        self.antagonist_action_size = self.num_rods_per_player * 2

        action_high = np.ones(self.protagonist_action_size)
        self.action_space = spaces.Box(
            low=-3.5 * action_high, high=3.5 * action_high, dtype=np.float32
        )

        self.observation_space = spaces.Box(
            low=0, high=255,
            shape=(self.image_height, self.image_width, 3),
            dtype=np.uint8
        )

        self.simulation_time = 0
        self._healthy_z_range = (-10, 10)
        self.max_no_progress_steps = 80
        self.prev_ball_y = None
        self.no_progress_steps = 0
        self.ball_stopped_count = 0
        self._terminate_when_unhealthy = True
        self._healthy_reward = 1.0
        self._ctrl_cost_weight = 0.1

        self.antagonist_model = antagonist_model
        self.play_until_goal = play_until_goal
        self.verbose_mode = verbose_mode

    def update_camera(self):
        if self.camera_config.camera_id is not None:
            mujoco.mjv_defaultCamera(self.cam)
            self.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            self.cam.fixedcamid = self.camera_config.camera_id

        elif self.camera_config.camera_name is not None:
            camera_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_config.camera_name
            )
            if camera_id >= 0:
                mujoco.mjv_defaultCamera(self.cam)
                self.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                self.cam.fixedcamid = camera_id
            else:
                raise ValueError(f"Camera '{self.camera_config.camera_name}' not found in model")

        else:
            mujoco.mjv_defaultCamera(self.cam)
            self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self.cam.distance = self.camera_config.distance
            self.cam.azimuth = self.camera_config.azimuth
            self.cam.elevation = self.camera_config.elevation
            self.cam.lookat[:] = self.camera_config.lookat

    def _render_image(self):
        mujoco.mjv_updateScene(
            self.model, self.data, self.opt, None, self.cam,
            mujoco.mjtCatBit.mjCAT_ALL, self.scene
        )

        if self.camera_config.tracking_point is not None:
            self.cam.lookat[:] = self.camera_config.tracking_point

        self.renderer.update_scene(self.data, self.cam)

        image = self.renderer.render()
        return image


    def set_camera_config(self, new_config):
        self.camera_config = new_config
        self.update_camera()

    def _get_obs(self):
        return self._render_image()

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)

        ball_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_free_joint')
        qpos_adr = self.model.jnt_qposadr[ball_joint_id]
        self.data.qpos[qpos_adr:qpos_adr + 3] = np.random.normal(
            loc=[-0.5, 0.0, 0.55], scale=[0.5, 0.5, 0.0]
        )

        self.simulation_time = 0
        self.prev_ball_y = self.data.qpos[qpos_adr + 1]
        self.no_progress_steps = 0
        self.ball_stopped_count = 0

        return self._get_obs(), {}

    def step(self, protagonist_action):
        protagonist_action = np.clip(protagonist_action, self.action_space.low, self.action_space.high)

        if self.antagonist_model is not None:
            antagonist_observation = self._get_obs()
            antagonist_action = self.antagonist_model.predict(antagonist_observation)
            antagonist_action = np.clip(antagonist_action, -1.0, 1.0)
            antagonist_action = -antagonist_action
        else:
            antagonist_action = np.zeros(self.antagonist_action_size)

        self.data.ctrl[:self.protagonist_action_size] = protagonist_action
        self.data.ctrl[self.protagonist_action_size:self.protagonist_action_size + self.antagonist_action_size] = antagonist_action

        mujoco.mj_step(self.model, self.data)
        self.simulation_time += self.model.opt.timestep

        obs = self._get_obs()

        reward = self.compute_reward(protagonist_action)
        terminated = self.terminated

        return obs, reward, terminated, False, {}

    def compute_reward(self, protagonist_action):
        """Compute the reward based on ball position and movement."""
        ball_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_free_joint')
        qpos_adr = self.model.jnt_qposadr[ball_joint_id]
        ball_y = self.data.qpos[qpos_adr + 1]

        if self.prev_ball_y is not None:
            progress = (ball_y - self.prev_ball_y) * DIRECTION_CHANGE
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

        victory = -10000 * DIRECTION_CHANGE if ball_y < -1 * TABLE_MAX_Y_DIM else 0
        loss = 10000 * DIRECTION_CHANGE if ball_y > TABLE_MAX_Y_DIM else 0
        progress_penalty = -100 if self.no_progress_steps >= 10 else 0
        continuation_reward = 110

        return loss + victory + progress_reward + healthy_reward + progress_penalty + continuation_reward

    @property
    def healthy_reward(self):
        return float(self.is_healthy or self._terminate_when_unhealthy) * self._healthy_reward

    def control_cost(self, action):
        return self._ctrl_cost_weight * np.sum(np.square(action))

    @property
    def is_healthy(self):
        ball_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_free_joint')
        qpos_adr = self.model.jnt_qposadr[ball_joint_id]
        ball_z = self.data.qpos[qpos_adr + 2]
        min_z, max_z = self._healthy_z_range
        return min_z < ball_z < max_z

    def _is_ball_moving(self):
        ball_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_free_joint')
        qvel_adr = self.model.jnt_dofadr[ball_joint_id]
        ball_vel = self.data.qvel[qvel_adr:qvel_adr + 3]
        return np.linalg.norm(ball_vel) > 0.5

    @property
    def terminated(self):
        self.ball_stopped_count = 0 if self._is_ball_moving() else self.ball_stopped_count + 1
        ball_stagnant = self.ball_stopped_count >= BALL_STOPPED_COUNT_THRESHOLD

        unhealthy = not self.is_healthy
        no_progress = self.no_progress_steps >= self.max_no_progress_steps

        ball_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'ball_free_joint')
        qpos_adr = self.model.jnt_qposadr[ball_joint_id]
        ball_y = self.data.qpos[qpos_adr + 1]
        victory = abs(ball_y) > TABLE_MAX_Y_DIM

        terminated = (unhealthy or (no_progress and not self.play_until_goal) or victory or ball_stagnant) if self._terminate_when_unhealthy else False

        if self.verbose_mode and terminated:
            print(f"Terminated: Unhealthy: {unhealthy}, No progress: {no_progress}, Victory: {victory}, Ball stagnant: {ball_stagnant}")

        return terminated

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
