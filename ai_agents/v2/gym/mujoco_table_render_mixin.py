import numpy as np
import mujoco
import glfw
import time

class MujocoTableRenderMixin():
    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(self):
        self.viewer = None
        self.window = None
        self._initialize_glfw()

    def _initialize_glfw(self):
        if not glfw.init():
            raise Exception("Could not initialize GLFW")

import numpy as np
import mujoco
import glfw
import time

class MujocoTableRenderMixin():
    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(self):
        self.viewer = None
        self.window = None
        self._initialize_glfw()
        self.first_render = True

    def _initialize_glfw(self):
        if not glfw.init():
            raise Exception("Could not initialize GLFW")

    def render(self, mode='human'):
        if self.first_render:
            # Initialize window
            self.window = glfw.create_window(1280, 720, "Foosball Simulation", None, None)
            if not self.window:
                glfw.terminate()
                raise Exception("Could not create GLFW window")

            # Center window
            monitor = glfw.get_primary_monitor()
            video_mode = glfw.get_video_mode(monitor)
            window_width, window_height = 1280, 720
            glfw.set_window_pos(
                self.window,
                (video_mode.size.width - window_width) // 2,
                (video_mode.size.height - window_height) // 2
            )

            glfw.make_context_current(self.window)
            glfw.swap_interval(1)

            # Initialize Mujoco rendering
            self.cam = mujoco.MjvCamera()
            self.opt = mujoco.MjvOption()

            # Initialize camera
            mujoco.mjv_defaultCamera(self.cam)
            mujoco.mjv_defaultOption(self.opt)

            self.cam.azimuth = 180.0
            self.cam.elevation = -40.0  # Changed for better viewing angle
            self.cam.distance = 100.0     # Moved closer
            self.cam.lookat[:] = np.array([0, 0, 1.0])  # Adjusted focus point

            self.scn = mujoco.MjvScene(self.model, maxgeom=10000)
            self.ctx = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)

            self.first_render = False

        if self.window and not glfw.window_should_close(self.window):
            glfw.make_context_current(self.window)

            viewport_width, viewport_height = glfw.get_framebuffer_size(self.window)
            viewport = mujoco.MjrRect(0, 0, viewport_width, viewport_height)

            mujoco.mjv_updateScene(
                self.model,
                self.data,
                self.opt,
                None,
                self.cam,
                mujoco.mjtCatBit.mjCAT_ALL,
                self.scn
            )

            # Render
            mujoco.mjr_render(viewport, self.scn, self.ctx)

            glfw.swap_buffers(self.window)
            glfw.poll_events()

            time.sleep(0.02)
        else:
            self.close()

    def close(self):
        if hasattr(self, 'ctx') and self.ctx is not None:
            self.ctx.free()
        if hasattr(self, 'scn') and self.scn is not None:
            self.scn.free()
        if self.window is not None:
            glfw.destroy_window(self.window)
            self.window = None
        glfw.terminate()