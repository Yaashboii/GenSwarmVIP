"""
Copyright (c) 2024 WindyLab of Westlake University, China
All rights reserved.

This software is provided "as is" without warranty of any kind, either
express or implied, including but not limited to the warranties of
merchantability, fitness for a particular purpose, or non-infringement.
In no event shall the authors or copyright holders be liable for any
claim, damages, or other liability, whether in an action of contract,
tort, or otherwise, arising from, out of, or in connection with the
software or the use or other dealings in the software.
"""

import argparse
import pickle

# from transformers import SEWDModel

from modules.file import File
from modules.framework.code import FunctionTree
from modules.framework.constraint import ConstraintPool

from .context import Context
from modules.prompt import robot_api


class WorkflowContext(Context):
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._instance._initialize(*args, **kwargs)

        return cls._instance

    def _initialize(self,args=None):
        self.user_command = File(name="command.md")
        self.feedbacks = []
        self.run_code = File(name="run.py")
        self.args = args or argparse.Namespace()  # ✅ 使用传入的 args
        self._constraint_pool = ConstraintPool()
        # TODO:所有的命名统一化，比如这里的global skill tree和local skill tree (@Jiwenkang 10-4)
        if args:
            task_name = self.args.run_experiment_name[0]
            self.global_robot_api = robot_api.get_api_prompt(task_name, scope="global")
            self.local_robot_api = robot_api.get_api_prompt(task_name, scope="local")
            self.robotarium_py = ROBOTARIUM_PY
            self.tranformation_py = TRANSFORMATION_PY
            global_import_list = robot_api.get_api_prompt(
                task_name, scope="global", only_names=True
            )
            local_import_list = robot_api.get_api_prompt(task_name, scope="local", only_names=True)
            self.local_import_list = (
                local_import_list.split("\n\n")
                if isinstance(local_import_list, str)
                else local_import_list
            )
            self.local_import_list.append("get_assigned_task")
            self._global_skill_tree = FunctionTree(
                name="global_skill",
                init_import_list={
                    f"from global_apis import {','.join(global_import_list)}"
                },
            )
            self._local_skill_tree = FunctionTree(
                name="local_skill",
                init_import_list={
                    f"from apis import initialize_ros_node, {','.join(self.local_import_list)}"
                },
            )
        self.global_run_result = File(name="allocate_result.pkl")
        self.scoop = "global"
        self.vlm = False

    def save_to_file(self, file_path):
        with open(file_path, "wb") as file:
            pickle.dump(self._instance, file)
    @classmethod
    def load_from_file(cls, file_path):
        with open(file_path, "rb") as file:
            instance = pickle.load(file)
            cls._instance = instance
            return instance

    def set_root_for_files(self, root_value):
        for file_attr in vars(self).values():
            if isinstance(file_attr, File):
                file_attr.root = root_value
            if isinstance(file_attr, FunctionTree):
                file_attr.file.root = root_value

    @property
    def command(self):
        return self._instance.user_command.message

    @command.setter
    def command(self, value):
        self._instance.user_command.message = value

    @property
    def global_skill_tree(self) -> FunctionTree:
        return self._instance._global_skill_tree

    @property
    def local_skill_tree(self) -> FunctionTree:
        return self._instance._local_skill_tree

    @property
    def constraint_pool(self) -> ConstraintPool:
        return self._instance._constraint_pool

ROBOTARIUM_PY = """
import math
import time

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from rps.robotarium_abc import *

# Robotarium This object provides routines to interface with the Robotarium.
#
# THIS CLASS SHOULD NEVER BE MODIFIED OR SUBMITTED

class Robotarium(RobotariumABC):

        def __init__(self, number_of_robots=-1, show_figure=True, sim_in_real_time = True, initial_conditions=np.array([])):
            super().__init__(number_of_robots, show_figure, sim_in_real_time, initial_conditions)

            #Initialize some rendering variables
            self.previous_render_time = time.time()
            self.sim_in_real_time = sim_in_real_time

            #Initialize checks for step and get poses calls
            self._called_step_already = True
            self._checked_poses_already = False

            #Initialization of error collection.
            self._errors = {}

            #Initialize steps
            self._iterations = 0 

        def get_poses(self):
            #Returns the states of the agents.

            #-> 3xN numpy array (of robot poses)
            

            assert(not self._checked_poses_already), "Can only call get_poses() once per call of step()."
            # Allow step() to be called again.
            self._called_step_already = False
            self._checked_poses_already = True 

            return self.poses

        def call_at_scripts_end(self):
            #Call this function at the end of scripts to display potentail errors.  
            #Even if you don't want to print the errors, calling this function at the
            #end of your script will enable execution on the Robotarium testbed.
            print('##### DEBUG OUTPUT #####')
            print('Your simulation will take approximately {0} real seconds when deployed on the Robotarium. \n'.format(math.ceil(self._iterations*self.time_step)))
            # TODO: check collision string and boundary string
            if bool(self._errors):
                if "boundary" in self._errors:
                    boundary_violations = max(self._errors["boundary"].values())
                    print('\t Simulation had {0} {1}\n'.format(boundary_violations, self._errors["boundary_string"]))
                if "collision" in self._errors:
                    collision_violations = max(self._errors["collision"].values())
                    print('\t Simulation had {0} {1}\n'.format(collision_violations, self._errors["collision_string"]))
                if "actuator" in self._errors:
                    print('\t Simulation had {0} {1}'.format(self._errors["actuator"], self._errors["actuator_string"]))
            else:
                print('No errors in your simulation! Acceptance of your experiment is likely!')

            return

        def step(self):
            # Increments the simulation by updating the dynamics.
            
            assert(not self._called_step_already), "Make sure to call get_poses before calling step() again."
            
            # Allow get_poses function to be called again.
            self._called_step_already = True
            self._checked_poses_already = False

            # Validate before thresholding velocities
            self._errors = self._validate()
            self._iterations += 1

            #Perform Thresholding of Motors
            self.velocities = self._threshold(self.velocities)

            # Update dynamics of agents
            self.poses[0, :] = self.poses[0, :] + self.time_step*np.cos(self.poses[2,:])*self.velocities[0, :]
            self.poses[1, :] = self.poses[1, :] + self.time_step*np.sin(self.poses[2,:])*self.velocities[0, :]
            self.poses[2, :] = self.poses[2, :] + self.time_step*self.velocities[1, :]
            # Ensure angles are wrapped
            self.poses[2, :] = np.arctan2(np.sin(self.poses[2, :]), np.cos(self.poses[2, :]))

            # Update graphics
            if(self.show_figure):
                for i in range(self.number_of_robots):
                    self.chassis_patches[i].xy = self.poses[:2, i]+self.robot_length/2*np.array((np.cos(self.poses[2, i]+math.pi/2), np.sin(self.poses[2, i]+math.pi/2)))+\
                                            0.04*np.array((-np.sin(self.poses[2, i]+math.pi/2), np.cos(self.poses[2, i]+math.pi/2)))  + self.robot_length/2*np.array((np.cos(self.poses[2, i]), np.sin(self.poses[2, i])))
                    
                    self.chassis_patches[i].angle = (self.poses[2, i] - math.pi/2) * 180/math.pi

                    self.chassis_patches[i].zorder = 2

                    self.right_wheel_patches[i].center = self.poses[:2, i]+self.robot_length/2*np.array((np.cos(self.poses[2, i]+math.pi/2), np.sin(self.poses[2, i]+math.pi/2)))+\
                                            0.04*np.array((-np.sin(self.poses[2, i]+math.pi/2), np.cos(self.poses[2, i]+math.pi/2)))  + self.robot_length/2*np.array((np.cos(self.poses[2, i]), np.sin(self.poses[2, i])))
                    self.right_wheel_patches[i].orientation = self.poses[2, i] + math.pi/4

                    self.right_wheel_patches[i].zorder = 2

                    self.left_wheel_patches[i].center = self.poses[:2, i]+self.robot_length/2*np.array((np.cos(self.poses[2, i]-math.pi/2), np.sin(self.poses[2, i]-math.pi/2)))+\
                                            0.04*np.array((-np.sin(self.poses[2, i]+math.pi/2), np.cos(self.poses[2, i]+math.pi/2))) + self.robot_length/2*np.array((np.cos(self.poses[2, i]), np.sin(self.poses[2, i])))
                    self.left_wheel_patches[i].orientation = self.poses[2,i] + math.pi/4

                    self.left_wheel_patches[i].zorder = 2
                    
                    self.right_led_patches[i].center = self.poses[:2, i]+0.75*self.robot_length/2*np.array((np.cos(self.poses[2,i]), np.sin(self.poses[2,i])))-\
                                    0.04*np.array((-np.sin(self.poses[2, i]), np.cos(self.poses[2, i]))) + self.robot_length/2*np.array((np.cos(self.poses[2, i]), np.sin(self.poses[2, i])))
                    self.left_led_patches[i].center = self.poses[:2, i]+0.75*self.robot_length/2*np.array((np.cos(self.poses[2,i]), np.sin(self.poses[2,i])))-\
                                    0.015*np.array((-np.sin(self.poses[2, i]), np.cos(self.poses[2, i]))) + self.robot_length/2*np.array((np.cos(self.poses[2, i]), np.sin(self.poses[2, i])))
                    self.left_led_patches[i].zorder = 2
                    self.right_led_patches[i].zorder = 2 

                self.figure.canvas.draw_idle()
                self.figure.canvas.flush_events()

            if(self.sim_in_real_time):
                t = time.time()
                while(t - self.previous_render_time < self.time_step):
                    t=time.time()
                self.previous_render_time = t
"""

TRANSFORMATION_PY = """
import numpy as np

def create_si_to_uni_dynamics(linear_velocity_gain=1, angular_velocity_limit=np.pi):
    # Returns a function mapping from single-integrator to unicycle dynamics with angular velocity magnitude restrictions.

    #    linear_velocity_gain: Gain for unicycle linear velocity
    #    angular_velocity_limit: Limit for angular velocity (i.e., |w| < angular_velocity_limit)

    #   -> function
    

    #Check user input types
    assert isinstance(linear_velocity_gain, (int, float)), "In the function create_si_to_uni_dynamics, the linear velocity gain (linear_velocity_gain) must be an integer or float. Recieved type %r." % type(linear_velocity_gain).__name__
    assert isinstance(angular_velocity_limit, (int, float)), "In the function create_si_to_uni_dynamics, the angular velocity limit (angular_velocity_limit) must be an integer or float. Recieved type %r." % type(angular_velocity_limit).__name__

    #Check user input ranges/sizes
    assert linear_velocity_gain > 0, "In the function create_si_to_uni_dynamics, the linear velocity gain (linear_velocity_gain) must be positive. Recieved %r." % linear_velocity_gain
    assert angular_velocity_limit >= 0, "In the function create_si_to_uni_dynamics, the angular velocity limit (angular_velocity_limit) must not be negative. Recieved %r." % angular_velocity_limit
    

    def si_to_uni_dyn(dxi, poses):
        #A mapping from single-integrator to unicycle dynamics.

        #dxi: 2xN numpy array with single-integrator control inputs
        #poses: 2xN numpy array with single-integrator poses

        #-> 2xN numpy array of unicycle control inputs
        

        #Check user input types
        assert isinstance(dxi, np.ndarray), "In the si_to_uni_dyn function created by the create_si_to_uni_dynamics function, the single integrator velocity inputs (dxi) must be a numpy array. Recieved type %r." % type(dxi).__name__
        assert isinstance(poses, np.ndarray), "In the si_to_uni_dyn function created by the create_si_to_uni_dynamics function, the current robot poses (poses) must be a numpy array. Recieved type %r." % type(poses).__name__

        #Check user input ranges/sizes
        assert dxi.shape[0] == 2, "In the si_to_uni_dyn function created by the create_si_to_uni_dynamics function, the dimension of the single integrator velocity inputs (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension %r." % dxi.shape[0]
        assert poses.shape[0] == 3, "In the si_to_uni_dyn function created by the create_si_to_uni_dynamics function, the dimension of the current pose of each robot must be 3 ([x;y;theta]). Recieved dimension %r." % poses.shape[0]
        assert dxi.shape[1] == poses.shape[1], "In the si_to_uni_dyn function created by the create_si_to_uni_dynamics function, the number of single integrator velocity inputs must be equal to the number of current robot poses. Recieved a single integrator velocity input array of size %r x %r and current pose array of size %r x %r." % (dxi.shape[0], dxi.shape[1], poses.shape[0], poses.shape[1])

        M,N = np.shape(dxi)

        a = np.cos(poses[2, :])
        b = np.sin(poses[2, :])

        dxu = np.zeros((2, N))
        dxu[0, :] = linear_velocity_gain*(a*dxi[0, :] + b*dxi[1, :])
        dxu[1, :] = angular_velocity_limit*np.arctan2(-b*dxi[0, :] + a*dxi[1, :], dxu[0, :])/(np.pi/2)

        return dxu

    return si_to_uni_dyn

def create_si_to_uni_dynamics_with_backwards_motion(linear_velocity_gain=1, angular_velocity_limit=np.pi):
    # Returns a function mapping from single-integrator dynamics to unicycle dynamics. This implementation of 
    #the mapping allows for robots to drive backwards if that direction of linear velocity requires less rotation.

    #   linear_velocity_gain: Gain for unicycle linear velocity
    #   angular_velocity_limit: Limit for angular velocity (i.e., |w| < angular_velocity_limit)


    # TODO: Backwards motion is the same as without backwards motion.

    #Check user input types
    assert isinstance(linear_velocity_gain, (int, float)), "In the function create_si_to_uni_dynamics, the linear velocity gain (linear_velocity_gain) must be an integer or float. Recieved type %r." % type(linear_velocity_gain).__name__
    assert isinstance(angular_velocity_limit, (int, float)), "In the function create_si_to_uni_dynamics, the angular velocity limit (angular_velocity_limit) must be an integer or float. Recieved type %r." % type(angular_velocity_limit).__name__

    #Check user input ranges/sizes
    assert linear_velocity_gain > 0, "In the function create_si_to_uni_dynamics, the linear velocity gain (linear_velocity_gain) must be positive. Recieved %r." % linear_velocity_gain
    assert angular_velocity_limit >= 0, "In the function create_si_to_uni_dynamics, the angular velocity limit (angular_velocity_limit) must not be negative. Recieved %r." % angular_velocity_limit
    

    def si_to_uni_dyn(dxi, poses):
        #A mapping from single-integrator to unicycle dynamics.

        #dxi: 2xN numpy array with single-integrator control inputs
        #poses: 2xN numpy array with single-integrator poses

        #-> 2xN numpy array of unicycle control inputs
        

        #Check user input types
        assert isinstance(dxi, np.ndarray), "In the si_to_uni_dyn function created by the create_si_to_uni_dynamics_with_backwards_motion function, the single integrator velocity inputs (dxi) must be a numpy array. Recieved type %r." % type(dxi).__name__
        assert isinstance(poses, np.ndarray), "In the si_to_uni_dyn function created by the create_si_to_uni_dynamics_with_backwards_motion function, the current robot poses (poses) must be a numpy array. Recieved type %r." % type(poses).__name__

        #Check user input ranges/sizes
        assert dxi.shape[0] == 2, "In the si_to_uni_dyn function created by the create_si_to_uni_dynamics_with_backwards_motion function, the dimension of the single integrator velocity inputs (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension %r." % dxi.shape[0]
        assert poses.shape[0] == 3, "In the si_to_uni_dyn function created by the create_si_to_uni_dynamics_with_backwards_motion function, the dimension of the current pose of each robot must be 3 ([x;y;theta]). Recieved dimension %r." % poses.shape[0]
        assert dxi.shape[1] == poses.shape[1], "In the si_to_uni_dyn function created by the create_si_to_uni_dynamics_with_backwards_motion function, the number of single integrator velocity inputs must be equal to the number of current robot poses. Recieved a single integrator velocity input array of size %r x %r and current pose array of size %r x %r." % (dxi.shape[0], dxi.shape[1], poses.shape[0], poses.shape[1])

        M,N = np.shape(dxi)

        a = np.cos(poses[2, :])
        b = np.sin(poses[2, :])

        dxu = np.zeros((2, N))
        dxu[0, :] = linear_velocity_gain*(a*dxi[0, :] + b*dxi[1, :])
        dxu[1, :] = angular_velocity_limit*np.arctan2(-b*dxi[0, :] + a*dxi[1, :], dxu[0, :])/(np.pi/2)

        return dxu

    return si_to_uni_dyn

def create_si_to_uni_mapping(projection_distance=0.05, angular_velocity_limit = np.pi):
    #Creates two functions for mapping from single integrator dynamics to 
    #unicycle dynamics and unicycle states to single integrator states. 
    
    #This mapping is done by placing a virtual control "point" in front of 
    #the unicycle.

    #projection_distance: How far ahead to place the point
    #angular_velocity_limit: The maximum angular velocity that can be provided

    #-> (function, function)
    

    #Check user input types
    assert isinstance(projection_distance, (int, float)), "In the function create_si_to_uni_mapping, the projection distance of the new control point (projection_distance) must be an integer or float. Recieved type %r." % type(projection_distance).__name__
    assert isinstance(angular_velocity_limit, (int, float)), "In the function create_si_to_uni_mapping, the maximum angular velocity command (angular_velocity_limit) must be an integer or float. Recieved type %r." % type(angular_velocity_limit).__name__
    
    #Check user input ranges/sizes
    assert projection_distance > 0, "In the function create_si_to_uni_mapping, the projection distance of the new control point (projection_distance) must be positive. Recieved %r." % projection_distance
    assert projection_distance >= 0, "In the function create_si_to_uni_mapping, the maximum angular velocity command (angular_velocity_limit) must be greater than or equal to zero. Recieved %r." % angular_velocity_limit

    def si_to_uni_dyn(dxi, poses):
        #Takes single-integrator velocities and transforms them to unicycle
        #control inputs.

        #dxi: 2xN numpy array of single-integrator control inputs
        #poses: 3xN numpy array of unicycle poses

        #-> 2xN numpy array of unicycle control inputs
        

        #Check user input types
        assert isinstance(dxi, np.ndarray), "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the single integrator velocity inputs (dxi) must be a numpy array. Recieved type %r." % type(dxi).__name__
        assert isinstance(poses, np.ndarray), "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the current robot poses (poses) must be a numpy array. Recieved type %r." % type(poses).__name__

        #Check user input ranges/sizes
        assert dxi.shape[0] == 2, "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the dimension of the single integrator velocity inputs (dxi) must be 2 ([x_dot;y_dot]). Recieved dimension %r." % dxi.shape[0]
        assert poses.shape[0] == 3, "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the dimension of the current pose of each robot must be 3 ([x;y;theta]). Recieved dimension %r." % poses.shape[0]
        assert dxi.shape[1] == poses.shape[1], "In the si_to_uni_dyn function created by the create_si_to_uni_mapping function, the number of single integrator velocity inputs must be equal to the number of current robot poses. Recieved a single integrator velocity input array of size %r x %r and current pose array of size %r x %r." % (dxi.shape[0], dxi.shape[1], poses.shape[0], poses.shape[1])


        M,N = np.shape(dxi)

        cs = np.cos(poses[2, :])
        ss = np.sin(poses[2, :])

        dxu = np.zeros((2, N))
        dxu[0, :] = (cs*dxi[0, :] + ss*dxi[1, :])
        dxu[1, :] = (1/projection_distance)*(-ss*dxi[0, :] + cs*dxi[1, :])

        #Impose angular velocity cap.
        dxu[1,dxu[1,:]>angular_velocity_limit] = angular_velocity_limit
        dxu[1,dxu[1,:]<-angular_velocity_limit] = -angular_velocity_limit 

        return dxu

    def uni_to_si_states(poses):
        #Takes unicycle states and returns single-integrator states

        #poses: 3xN numpy array of unicycle states

        #-> 2xN numpy array of single-integrator states
        

        _,N = np.shape(poses)

        si_states = np.zeros((2, N))
        si_states[0, :] = poses[0, :] + projection_distance*np.cos(poses[2, :])
        si_states[1, :] = poses[1, :] + projection_distance*np.sin(poses[2, :])

        return si_states

    return si_to_uni_dyn, uni_to_si_states

def create_uni_to_si_dynamics(projection_distance=0.05):
    #Creates two functions for mapping from unicycle dynamics to single 
    #integrator dynamics and single integrator states to unicycle states. 
    
    #This mapping is done by placing a virtual control "point" in front of 
    #the unicycle.

    #projection_distance: How far ahead to place the point

    #-> function
    

    #Check user input types
    assert isinstance(projection_distance, (int, float)), "In the function create_uni_to_si_dynamics, the projection distance of the new control point (projection_distance) must be an integer or float. Recieved type %r." % type(projection_distance).__name__
    
    #Check user input ranges/sizes
    assert projection_distance > 0, "In the function create_uni_to_si_dynamics, the projection distance of the new control point (projection_distance) must be positive. Recieved %r." % projection_distance
    

    def uni_to_si_dyn(dxu, poses):
        #A function for converting from unicycle to single-integrator dynamics.
        #Utilizes a virtual point placed in front of the unicycle.

        #dxu: 2xN numpy array of unicycle control inputs
        #poses: 3xN numpy array of unicycle poses
        #projection_distance: How far ahead of the unicycle model to place the point

        #-> 2xN numpy array of single-integrator control inputs
    

        #Check user input types
        assert isinstance(dxu, np.ndarray), "In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the unicycle velocity inputs (dxu) must be a numpy array. Recieved type %r." % type(dxi).__name__
        assert isinstance(poses, np.ndarray), "In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the current robot poses (poses) must be a numpy array. Recieved type %r." % type(poses).__name__

        #Check user input ranges/sizes
        assert dxu.shape[0] == 2, "In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the dimension of the unicycle velocity inputs (dxu) must be 2 ([v;w]). Recieved dimension %r." % dxu.shape[0]
        assert poses.shape[0] == 3, "In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the dimension of the current pose of each robot must be 3 ([x;y;theta]). Recieved dimension %r." % poses.shape[0]
        assert dxu.shape[1] == poses.shape[1], "In the uni_to_si_dyn function created by the create_uni_to_si_dynamics function, the number of unicycle velocity inputs must be equal to the number of current robot poses. Recieved a unicycle velocity input array of size %r x %r and current pose array of size %r x %r." % (dxu.shape[0], dxu.shape[1], poses.shape[0], poses.shape[1])

        
        M,N = np.shape(dxu)

        cs = np.cos(poses[2, :])
        ss = np.sin(poses[2, :])

        dxi = np.zeros((2, N))
        dxi[0, :] = (cs*dxu[0, :] - projection_distance*ss*dxu[1, :])
        dxi[1, :] = (ss*dxu[0, :] + projection_distance*cs*dxu[1, :])

        return dxi

    return uni_to_si_dyn

"""