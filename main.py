import activVisionEnv

import gym
from gym import Env
from gym.spaces import Discrete, Box, Dict, Tuple, MultiBinary, MultiDiscrete
from gym.utils import seeding
import numpy as np
import random
import os
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import VecFrameStack
from stable_baselines3.common.evaluation import evaluate_policy

from typing import Final, Any
import math
from shapely.geometry.polygon import Polygon

import PIL.ImageDraw as ImageDraw
import PIL.Image as Image
import cv2


class ActivVisionEnv(Env):
    """
    ### Description
    This environment corresponds to a environment to adjust the position and angle of a camera,
    to optimize the area of view in regard to the objects, which are present in the field.
    With this environment it is possible to train a neural network to work on this task, which later can be put on the
    Robotino to test it in real life.

    ### Action Space
    The agent take a 1-element vector for actions.
    The action space is `(action)` in `[0, 1, 2, 3, 4]`, where `action` is used to move the robot or the camera.
    The Code can be easily altered to also provide the possibility to drive forewards and backwards, this is just
    commented out. Then the observation spaces had to be expanded.

    | Num | Action                 |
    |-----|------------------------|
    | 0   | accelerate right       |
    | 1   | accelerate left        |
    | 2   | turn cam right         |
    | 3   | turn cam left          |
    | 4   | do nothing             |




    ### Observation Space
    The observation is a `ndarray` with shape `(3 + num_objects * 4,)` where the elements correspond to the following:
    | Num  | Observation           | Min                  | Max                |
    |------|-----------------------|----------------------|--------------------|
    | 0    | Player Position X     | 0                    | 1                  |
    | 1    | Player Velocity X     | 0                    | 1                  |
    | 2    | Player Camera Angle   | 0                    | 1                  |
    |3+i*4 | Object Position X     | 0                    | 1                  |
    |4+i*4 | Object Position Y     | 0                    | 1                  |
    |5+i*4 | Object Velocity X     | 0                    | 1                  |
    |6+i*4 | Object Velocity Y     | 0                    | 1                  |


    ### Rewards
    Reward is 1 for every object in vision multiplied with a penalty for a non streight camera
    ### Starting State
    All Objects are in vision with random velocity. the agent is placed in the center with no velocity
    ### Episode Termination
    After a specific amount of time steps (3000)
    ### Arguments
    See init
    """

    def __init__(self, num_objects: int = 3,
                 simulation_frequency: float = 5,
                 width: int = 5000, height: int = 2000, max_velocity_player: int = 200, view: int = 78,
                 img_format: float = (1920 / 1080), is_random_enabled: bool = False):
        """

        :param num_objects:  number of objects to project
        :param simulation_frequency: control speed in Hz
        :param width: width of 2D Area in mm
        :param height: height of 2D Area in mm
        :param max_velocity_player: max velocity of player
        :param view: camera view angele
        :param img_format: pixel_width/pixel_height if pixel are square. Otherwise width [mm] / height [mm]
        :param is_random_enabled: is the movement deterministic or is there a random factor in it
        """

        # consts
        self._object_min_size: Final = 60  # in mm
        self._object_max_size: Final = 120  # in mm
        self._object_max_z_velocity: Final = 20  # in mm
        self._object_max_velocity: Final = 200  # in mm/s
        self._camera_height: Final = 390  # in mm
        self._camera_max_angle: Final = 20  # in degree
        self._velocity_player_per_step = 50  # max velocity change at each cycle
        self._camera_per_step = 1  # max camera angle change at each cycle
        # player is not allowed to leave
        self._player_max_z = width + 1
        self._player_min_z = 1
        self._player_pos_y = height - self._camera_height
        # one cycle goes for timer time steps
        self.timer = 3000
        self._object_max_rnd_force_p_sec = 10  # in 1/percent from max

        # params
        self._num_objects = num_objects
        self._simulation_frequency = simulation_frequency
        self._width = width
        self._height = height
        self._max_velocity_player = max_velocity_player
        self._view = view
        self._img_format = img_format
        self._height_view = (1 / img_format) * view
        self._random_force_enabled = is_random_enabled

        # Actions we can take, down, stay, up
        self.action_space = Discrete(5)
        # positions and velocities
        self.observation_space = Box(0, 1, shape=(3 + num_objects * 4,))
        # Set start state
        self.state = []

        # viewer
        self.viewer = None

        for _ in range(0, 5 + 6 * num_objects):
            self.state.append(0)

        # states for player and objects as dictionary
        self.player = {}
        self._objects = []

        # for drawing
        self._last_points = 0

        self.reset()

    def _get_normalized_state(self):
        """
        normalize values to 0...1
        :return: normalized observation state
        """
        """
        self.state[0] = self.player["pos_x"] / self._width
        self.state[1] = self.player["pos_z"] / self._player_max_z
        self.state[2] = (self.player["vel_x"] / (2 * self._max_velocity_player)) + 0.5
        self.state[3] = (self.player["vel_z"] / (2 * self._max_velocity_player)) + 0.5
        self.state[4] = (self.player["angle"] / (2 * self._camera_max_angle)) + 0.5

        for i in range(0, self._num_objects):
            self.state[5 + 6 * i] = self._objects[i]["pos_x"] / self._width
            self.state[6 + 6 * i] = self._objects[i]["pos_y"] / self._height
            self.state[7 + 6 * i] = (self._objects[i]["size"] - self._object_min_size) / (
                        self._object_max_size - self._object_min_size)
            self.state[8 + 6 * i] = (self._objects[i]["vel_x"] / (2 * self._object_max_velocity)) + 0.5
            self.state[9 + 6 * i] = (self._objects[i]["vel_y"] / (2 * self._object_max_velocity)) + 0.5
            self.state[10 + 6 * i] = (self._objects[i]["vel_z"] / (2 * self._object_max_z_velocity)) + 0.5
        """
        self.state[0] = self.player["pos_x"] / self._width
        self.state[1] = (self.player["vel_x"] / (2 * self._max_velocity_player)) + 0.5
        self.state[2] = (self.player["angle"] / (2 * self._camera_max_angle)) + 0.5

        for i in range(0, self._num_objects):
            self.state[3 + 4 * i] = self._objects[i]["pos_x"] / self._width
            self.state[4 + 4 * i] = self._objects[i]["pos_y"] / self._height
            self.state[5 + 4 * i] = (self._objects[i]["vel_x"] / (2 * self._object_max_velocity)) + 0.5
            self.state[6 + 4 * i] = (self._objects[i]["vel_y"] / (2 * self._object_max_velocity)) + 0.5

        return self.state

    def step(self, action):
        """
        progress for one frame
        :param action: action to take in this step
        :return: observation space after the step
        """
        # actions:
        # 0 go right
        # 1 go left

        # 2 turn cam right
        # 3 turn cam left

        # 4 do nothing

        # --- go front
        # --- go back

        if action == 0:
            if self.player["vel_x"] < self._max_velocity_player:
                self.player["vel_x"] += self._velocity_player_per_step
            if self.player["vel_x"] > self._max_velocity_player:
                self.player["vel_x"] = self._max_velocity_player

        if action == 1:
            if self.player["vel_x"] > -self._max_velocity_player:
                self.player["vel_x"] -= self._velocity_player_per_step
            if self.player["vel_x"] < -self._max_velocity_player:
                self.player["vel_x"] = -self._max_velocity_player
        """
        if action == 2:
            if self.player["vel_z"] < self._max_velocity_player:
                self.player["vel_z"] += self._velocity_player_per_step
            if self.player["vel_z"] > self._max_velocity_player:
                self.player["vel_z"] = self._max_velocity_player

        if action == 3:
            if self.player["vel_z"] > -self._max_velocity_player:
                self.player["vel_z"] -= self._velocity_player_per_step
            if self.player["vel_z"] < -self._max_velocity_player:
                self.player["vel_z"] = -self._max_velocity_player

        """

        if action == 2:
            if self.player["angle"] < self._camera_max_angle:
                self.player["angle"] += self._camera_per_step
            if self.player["angle"] > self._camera_max_angle:
                self.player["angle"] = self._camera_max_angle

        if action == 3:
            if self.player["angle"] > -self._camera_max_angle:
                self.player["angle"] -= self._camera_per_step
            if self.player["angle"] < -self._camera_max_angle:
                self.player["angle"] = -self._camera_max_angle

        # calc next frame
        self._progress_simulation()
        reward = self._add_points()

        # Reduce timer by one step
        self.timer -= 1

        # Calculate reward
        reward = self._add_points()
        reward *= (1 - (abs(self.player["angle"] / self._camera_max_angle)) * (1 / self._num_objects))

        # reward -= abs(self.player["angle"] / self._camera_max_angle) * 0.05

        """
        if self.player["angle"] > self._camera_per_step and action == 2:
            reward -= (1/self._camera_max_angle+1)
        if self.player["angle"] > self._camera_per_step and action == 3:
            reward += 0.5 * (1/self._camera_max_angle+1)

        if self.player["angle"] < -self._camera_per_step and action == 3:
            reward -= (1/self._camera_max_angle+1)
        if self.player["angle"] < -self._camera_per_step and action == 2:
            reward += 0.5 * (1/self._camera_max_angle+1)
        """

        # Check if shower is done
        if self.timer <= 0:
            done = True
        else:
            done = False

        # Set placeholder for info
        info = {}

        self._get_normalized_state()

        # Return step information
        self._last_points = reward
        return np.asarray(self.state), reward, done, info

    def _progress_simulation(self):
        """
        does the math to process one step all objects
        :return:
        """
        seconds_passed = 1 / self._simulation_frequency

        # move player
        self.player["pos_x"] = self.player["pos_x"] + seconds_passed * self.player["vel_x"]
        self.player["pos_z"] = self.player["pos_z"] + seconds_passed * self.player["vel_z"]

        # push in constraints
        if self.player["pos_x"] < 0:
            self.player["pos_x"] = 0
        if self.player["pos_x"] > self._width:
            self.player["pos_x"] = self._width
        if self.player["pos_z"] < self._player_min_z:
            self.player["pos_z"] = self._player_min_z
        if self.player["pos_z"] > self._player_max_z:
            self.player["pos_z"] = self._player_max_z

        # move objects
        for i in range(0, self._num_objects):
            self._objects[i]["pos_x"] = self._objects[i]["pos_x"] + seconds_passed * self._objects[i]["vel_x"]
            self._objects[i]["pos_y"] = self._objects[i]["pos_y"] + seconds_passed * self._objects[i]["vel_y"]
            self._objects[i]["size"] = self._objects[i]["size"] + seconds_passed * self._objects[i]["vel_z"]

        # collision with wall
        for i in range(0, self._num_objects):
            if self._objects[i]["pos_x"] - self._objects[i]["size"] / 2 <= 0:
                self._objects[i]["vel_x"] = -self._objects[i]["vel_x"]
                self._objects[i]["pos_x"] = self._objects[i]["size"] / 2 + 1
                self._objects[i]["vel_z"] = 0

            if self._objects[i]["pos_x"] + self._objects[i]["size"] / 2 >= self._width:
                self._objects[i]["vel_x"] = -self._objects[i]["vel_x"]
                self._objects[i]["pos_x"] = self._width - self._objects[i]["size"] / 2 - 1
                self._objects[i]["vel_z"] = 0

            if self._objects[i]["pos_y"] - self._objects[i]["size"] / 2 <= 0:
                self._objects[i]["vel_y"] = -self._objects[i]["vel_y"]
                self._objects[i]["pos_y"] = self._objects[i]["size"] / 2 + 1
                self._objects[i]["vel_z"] = 0

            if self._objects[i]["pos_y"] + self._objects[i]["size"] / 2 >= self._height:
                self._objects[i]["vel_y"] = -self._objects[i]["vel_y"]
                self._objects[i]["pos_y"] = self._height - self._objects[i]["size"] / 2 - 1
                self._objects[i]["vel_z"] = 0

            if self._objects[i]["size"] <= self._object_min_size:
                self._objects[i]["vel_z"] = -self._objects[i]["vel_z"]
                self._objects[i]["size"] = self._object_min_size + 1

            if self._objects[i]["size"] >= self._object_max_size:
                self._objects[i]["vel_z"] = -self._objects[i]["vel_z"]
                self._objects[i]["size"] = self._object_max_size - 1

        # object collision
        for i in range(0, self._num_objects):
            for j in range(i + 1, self._num_objects):
                if i == j:
                    continue
                polygon_i = Polygon([(self._objects[i]["pos_x"] - self._objects[i]["size"] / 2,
                                      self._objects[i]["pos_y"] - self._objects[i]["size"] / 2),
                                     (self._objects[i]["pos_x"] + self._objects[i]["size"] / 2,
                                      self._objects[i]["pos_y"] - self._objects[i]["size"] / 2),
                                     (self._objects[i]["pos_x"] - self._objects[i]["size"] / 2,
                                      self._objects[i]["pos_y"] + self._objects[i]["size"] / 2),
                                     (self._objects[i]["pos_x"] + self._objects[i]["size"] / 2,
                                      self._objects[i]["pos_y"] + self._objects[i]["size"] / 2)])

                polygon_j = Polygon([(self._objects[j]["pos_x"] - self._objects[j]["size"] / 2,
                                      self._objects[j]["pos_y"] - self._objects[j]["size"] / 2),
                                     (self._objects[j]["pos_x"] + self._objects[j]["size"] / 2,
                                      self._objects[j]["pos_y"] - self._objects[j]["size"] / 2),
                                     (self._objects[j]["pos_x"] - self._objects[j]["size"] / 2,
                                      self._objects[j]["pos_y"] + self._objects[j]["size"] / 2),
                                     (self._objects[j]["pos_x"] + self._objects[j]["size"] / 2,
                                      self._objects[j]["pos_y"] + self._objects[j]["size"] / 2)])

                if polygon_i.intersects(polygon_j):
                    v_x = self._objects[i]["vel_x"]
                    self._objects[i]["vel_x"] = self._objects[j]["vel_x"]
                    self._objects[j]["vel_x"] = v_x

                    v_y = self._objects[i]["vel_y"]
                    self._objects[i]["vel_y"] = self._objects[j]["vel_y"]
                    self._objects[j]["vel_y"] = v_y

                    v_z = self._objects[i]["vel_z"]
                    self._objects[i]["vel_z"] = self._objects[j]["vel_z"]
                    self._objects[j]["vel_z"] = v_z

        if self._random_force_enabled:
            # apply force
            for i in range(0, self._num_objects):
                dvx = random.randint(-self._object_max_velocity, self._object_max_velocity) / \
                      (self._simulation_frequency * self._object_max_rnd_force_p_sec)
                dvy = random.randint(-self._object_max_velocity, self._object_max_velocity) / \
                      (self._simulation_frequency * self._object_max_rnd_force_p_sec)
                dvz = random.randint(-self._object_max_z_velocity, self._object_max_z_velocity) / \
                      (self._simulation_frequency * self._object_max_rnd_force_p_sec)

                self._objects[i]["vel_x"] += dvx
                self._objects[i]["vel_y"] += dvy
                self._objects[i]["vel_z"] += dvz

                # push it in the constraints
                if self._objects[i]["vel_x"] < -self._object_max_velocity:
                    self._objects[i]["vel_x"] = -self._object_max_velocity

                if self._objects[i]["vel_x"] > self._object_max_velocity:
                    self._objects[i]["vel_x"] = self._object_max_velocity

                if self._objects[i]["vel_y"] < -self._object_max_velocity:
                    self._objects[i]["vel_y"] = -self._object_max_velocity

                if self._objects[i]["vel_y"] > self._object_max_velocity:
                    self._objects[i]["vel_y"] = self._object_max_velocity

                if self._objects[i]["vel_z"] < -self._object_max_z_velocity:
                    self._objects[i]["vel_z"] = -self._object_max_z_velocity

                if self._objects[i]["vel_z"] > self._object_max_z_velocity:
                    self._objects[i]["vel_z"] = self._object_max_z_velocity

    def _add_points(self):
        """
        reward calculation
        :return:
        """
        corners_view = self._get_view()
        polygon = Polygon([corners_view["left_top"], corners_view["right_top"],
                           corners_view["right_bot"], corners_view["left_bot"]])

        intersections_sum = 0
        intersections_num = 0

        area_view = polygon.area
        for i in range(0, self._num_objects):
            size = self._objects[i]["size"]
            obj_x = self._objects[i]["pos_x"]
            obj_y = self._objects[i]["pos_y"]
            polygon_object = Polygon([
                (round(obj_x - size / 2), round(obj_y - size / 2)),
                (round(obj_x + size / 2), round(obj_y - size / 2)),
                (round(obj_x + size / 2), round(obj_y + size / 2)),
                (round(obj_x - size / 2), round(obj_y + size / 2))
            ])
            if area_view > 1:
                intersection = (polygon.intersection(polygon_object)).area
                if intersection > (size / 4):
                    intersections_sum += intersection
                    intersections_num += 1

        return intersections_num

    def _get_view(self) -> Any:
        """
        calculates the four corner points of the view of the camera in the big canvas in points in mm from top left
        :return: dict of the four points
        """

        # calc left , right border
        right_border = self.player["pos_x"] + self.player["pos_z"] * \
                       math.tan((self._view / 2 - self.player["angle"]) * (math.pi / 180))
        left_border = self.player["pos_x"] - self.player["pos_z"] * \
                      math.tan((self._view / 2 + self.player["angle"]) * (math.pi / 180))

        # calc top and bot borders, thinking of distortion
        left_height_diff = math.tan((self._height_view / 2) * (math.pi / 180)) * \
                           (self.player["pos_z"] / math.sin((self._view / 2 - self.player["angle"]) * (math.pi / 180)))

        right_height_diff = math.tan((self._height_view / 2) * (math.pi / 180)) * \
                            (self.player["pos_z"] / math.sin((self._view / 2 + self.player["angle"]) * (math.pi / 180)))

        right_top = self.player["pos_y"] - right_height_diff
        right_bot = self.player["pos_y"] + right_height_diff
        left_top = self.player["pos_y"] - left_height_diff
        left_bot = self.player["pos_y"] + left_height_diff

        left_border = round(left_border)
        left_top = round(left_top)
        right_border = round(right_border)
        right_top = round(right_top)
        left_bot = round(left_bot)
        right_bot = round(right_bot)

        return {
            "left_top": (left_border, left_top),
            "right_top": (right_border, right_top),
            "left_bot": (left_border, left_bot),
            "right_bot": (right_border, right_bot)
        }

    def render(self, mode: str = "human"):
        """
        renders the env
        :param mode: - not used - used cause override
        :return: - not used - used cause override
        """
        image = Image.new("RGB", (self._width, self._height))
        draw = ImageDraw.Draw(image)

        corners_view = self._get_view()
        points = (corners_view["left_top"], corners_view["right_top"], corners_view["right_bot"],
                  corners_view["left_bot"], corners_view["left_top"])

        draw.line(points, fill="white", width=20)
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (120, 120, 0), (0, 120, 120), (120, 0, 120)]
        for i in range(0, self._num_objects):
            point_1 = (round(self._objects[i]["pos_x"]) - 20, round(self._objects[i]["pos_y"]) - 20)
            point_2 = (round(self._objects[i]["pos_x"]) + 20, round(self._objects[i]["pos_y"]) + 20)
            draw.ellipse([point_1, point_2], fill=colors[i % len(colors)])
            size = self._objects[i]["size"]
            left_top_x = self._objects[i]["pos_x"]
            left_top_y = self._objects[i]["pos_y"]

            poly = [(round(left_top_x - size / 2), round(left_top_y - size / 2)),
                    (round(left_top_x + size / 2), round(left_top_y - size / 2)),
                    (round(left_top_x + size / 2), round(left_top_y + size / 2)),
                    (round(left_top_x - size / 2), round(left_top_y + size / 2)),
                    (round(left_top_x - size / 2), round(left_top_y - size / 2))]
            draw.line(poly, fill=colors[i % len(colors)], width=20)

        open_cv_image = np.array(image)
        open_cv_image = open_cv_image[:, :, ::-1].copy()
        open_cv_image = cv2.resize(open_cv_image, (900, round(640 / self._img_format)))
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = str("%.2f" % round(self._last_points, 2))
        cv2.putText(open_cv_image, text, (10, 50), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.imshow("simulation", open_cv_image)
        cv2.waitKey(1)

    def close(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None
            cv2.destroyAllWindows()

    def reset(self):
        """
        resets to default
        :return: observation space
        """
        self.points = 0.0
        self.timer = 3000

        self.player = {
            "pos_x": self._width / 2 * 1.0,
            "pos_y": self._height - self._camera_height * 1.0,
            "pos_z": 1200.0,
            "vel_x": 0.0,
            "vel_z": 0.0,
            "angle": 0
        }
        self._objects = []
        for i in range(0, self._num_objects):
            dict_to_append = {
                "pos_x": (self._width / 2) + (i - math.floor(self._num_objects / 2)) * self._object_max_size,
                "pos_y": (self._height / 2) + i * self._object_max_size,
                "vel_x": random.randint(-self._object_max_velocity, self._object_max_velocity) * 0.5,
                "vel_y": random.randint(-self._object_max_velocity, self._object_max_velocity) * 0.3,
                "vel_z": random.randint(-self._object_max_z_velocity, self._object_max_z_velocity) * 0.3,
                "size": random.randint(self._object_min_size, self._object_max_size) * 1.0
            }
            self._objects.append(dict_to_append)

        self._get_normalized_state()
        return np.asarray(self.state, dtype=np.float32)

        # state:
        # 0 pos player x
        # 1 pos player z
        # 2 vel player x
        # 3 vel player z
        # 4 angle

        # 5 pos obj 1 x
        # 6 pos obj 1 y
        # 7 pos obj 1 z
        # 8 vel obj 1 x
        # 9 vel obj 1 y
        # 10 vel obj 1 z
        # the 5-10 in repeat for every object


if __name__ == '__main__':
    env = activVisionEnv.ActivVisionEnv()
    for _ in range(0, 1000):
        model = PPO.load(PPO_path, env=env)

        # print("lets see")
        env.render("mode")
        env.step([0,0,0,0,0,0,0])

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
