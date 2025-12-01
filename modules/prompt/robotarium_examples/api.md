# Robotarium API (minimal)

Purpose: concise reference of functions, constructors, controllers, transformations, and barrier certificates extracted from Robotarium_Python_Guide.pdf

## Robotarium class and common methods
Robotarium(number_of_robots, show_figure=True, initial_conditions=None, sim_in_real_time=False)
Common instance methods:
- get_poses()
- set_velocities()
- step()
- call_at_scripts_end()
Notes: get_poses() -> 3xN array (x, y, theta). set_velocities(ids (1xN), dxu (2xN)). step() advances simulation.

## Controllers (single-integrator and unicycle)
Function: create_clf_unicycle_pose_controller
- context: create_clf_unicycle_pose_controller() # Create unicycle pose controller 18 19 uni_barrier_cert = create_unicycle_barrier_certificate() # Create barrier certificates to avoid collision 20 21 x = r.get_poses() # Get the poses of Robots 22 23 r.step() # Iterate t

Function: create_clf_unicycle_position_controller
- context: create_clf_unicycle_position_controller() # Create unicycle position controller 18 19 uni_barrier_cert = create_unicycle_barrier_certificate() # Create barrier certificates to avoid collision 20 21 x = r.get_poses() # Get the poses of the Robots 22 23 r.step()

Function: create_hybrid_unicycle_pose_controller
- context: create_hybrid_unicycle_pose_controller() # Create unicycle hybrid pose controller 18 19 uni_barrier_cert = create_unicycle_barrier_certificate() # Create barrier certificates to avoid collision 20 21 x = r.get_poses() # Get the poses of Robots 22 23 r.step() #

Function: create_si_position_controller
- context: create_si_position_controller() # Single integrator position controller 18 19 si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary() # Barrier certificates to avoid collision 20 21 _, uni_to_si_states = create_si_to_uni_mapping() # Unic

Function: create_si_to_uni_dynamics
- context: create_si_to_uni_dynamics_with_backwards_motion() # Single Integrator to Unicycle Velocity Commands 24 25 x = r.get_poses() # Get poses of the Robots 26 27 x_si = uni_to_si_states(x) # Transform poses to Single Integrator 28 29 r.step() # Iterate the simulatio

Function: create_si_to_uni_dynamics_with_backwards_motion
- context: create_si_to_uni_dynamics_with_backwards_motion() # Single Integrator to Unicycle Velocity Commands 24 25 x = r.get_poses() # Get poses of the Robots 26 27 x_si = uni_to_si_states(x) # Transform poses to Single Integrator 28 29 r.step() # Iterate the simulatio

Function: create_si_to_uni_mapping
- context: create_si_to_uni_mapping() # Unicycle to Single Integrator States Mapping 22 23 si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion() # Single Integrator to Unicycle Velocity Commands 24 25 x = r.get_poses() # Get poses of the Robots 26 27 x_si = u

Function: create_single_integrator_barrier_certificate
- context: create_single_integrator_barrier_certificate_with_boundary() # Barrier certificates to avoid collision 20 21 _, uni_to_si_states = create_si_to_uni_mapping() # Unicycle to Single Integrator States Mapping 22 23 si_to_uni_dyn = create_si_to_uni_dynamics_with_ba

Function: create_single_integrator_barrier_certificate2
- context: create_single_integrator_barrier_certificate2(barrier_gain=100, unsafe_barrier_gain=1e6, safety_radius=0.17, magnitude_limit=0.2): Listing 46: Barrier Certificate for a Single Integrator with Dynamic Gains Function This function creates a barrier certificate f

Function: create_single_integrator_barrier_certificate_with_boundary
- context: create_single_integrator_barrier_certificate_with_boundary() # Barrier certificates to avoid collision 20 21 _, uni_to_si_states = create_si_to_uni_mapping() # Unicycle to Single Integrator States Mapping 22 23 si_to_uni_dyn = create_si_to_uni_dynamics_with_ba

Function: create_unicycle_barrier_certificate
- context: create_unicycle_barrier_certificate() # Create barrier certificates to avoid collision 20 21 x = r.get_poses() # Get the poses of the Robots 22 23 r.step() # Iterate the simulation 24 25 # While all Robots are not in the goal points... 26 while (np.size(at_pos

Function: create_unicycle_barrier_certificate2
- context: create_unicycle_barrier_certificate2(barrier_gain=500, unsafe_barrier_gain=1e6, safety_radius=0.12, projection_distance=0.05, magnitude_limit=0.2): Listing 52: Barrier Certificate for a Unicycle Model with Dynamic Gains Function This function creates a barrier

Function: create_unicycle_barrier_certificate_with_boundary
- context: create_unicycle_barrier_certificate_with_boundary() # Create barrier certificates to avoid collision 19 20 x = r.get_poses() # Get the poses of robots 21 22 r.step() # Iterate the simulation 23 24 for i in range(iterations): 25 26 x = r.get_poses() # Get the p

Function: create_unicycle_differential_drive_barrier_certificate
- context: create_unicycle_differential_drive_barrier_certificate(max_num_obstacle_points=100, max_num_robots=30, disturbance=5, wheel_vel_limit=12.5, base_length=0.105, wheel_radius=0.016, projection_distance=0.05, barrier_gain=150, safety_radius=0.17): Listing 54: Unic

Function: create_unicycle_differential_drive_barrier_certificate_with_boundary
- context: create_unicycle_differential_drive_barrier_certificate_with_boundary( max_num_obstacle_points=100, max_num_robots=30, disturbance=5, wheel_vel_limit=12.5, base_length=0.105, wheel_radius=0.016, projection_distance=0.05, barrier_gain=150, safety_radius=0.17, bo

Function: pose_uni_clf_controller
- context: pose_uni_clf_controller(states, poses): Listing 39: Unicycle Model Position Controller Returned Function The inputs required by this function are: •states : A 3×Nnumpy array of unicycle states ( x, y, θ ). •poses : A 3×Nnumpy array of the desired positions ( x

Function: pose_uni_hybrid_controller
- context: pose_uni_hybrid_controller(states, poses, input_approach_state=np.empty([0, 0])): Listing 41: Unicycle Model Position Controller Based on Hybrid Controller Returned Function The inputs required by this function are: •states : A 3×Nnumpy array of unicycle state

Function: position_uni_clf_controller
- context: position_uni_clf_controller(states, positions): Listing 37: Unicycle Model Pose Controller Returned Function The inputs required by this function are: •states : A 3×Nnumpy array of unicycle states ( x, y, θ ). •positions : A 3×Nnumpy array of the desired posit

Function: si_position_controller
- context: si_position_controller() # Single integrator position controller 18 19 si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary() # Barrier certificates to avoid collision 20 21 _, uni_to_si_states = create_si_to_uni_mapping() # Unicycle to

## Barrier certificates
Function: create_robust_barriers
- context: create_robust_barriers(max_num_obstacles=100, max_num_robots=30, d=5, wheel_vel_limit=12.5, base_length=0.105, wheel_radius=0.016, projection_distance=0.05, gamma=150, safety_radius=0.12): Listing 58: Robust Barriers for Unicycle Differential Drive with Dynami

Function: create_single_integrator_barrier_certificate
- context: create_single_integrator_barrier_certificate_with_boundary() # Barrier certificates to avoid collision 20 21 _, uni_to_si_states = create_si_to_uni_mapping() # Unicycle to Single Integrator States Mapping 22 23 si_to_uni_dyn = create_si_to_uni_dynamics_with_ba

Function: create_single_integrator_barrier_certificate2
- context: create_single_integrator_barrier_certificate2(barrier_gain=100, unsafe_barrier_gain=1e6, safety_radius=0.17, magnitude_limit=0.2): Listing 46: Barrier Certificate for a Single Integrator with Dynamic Gains Function This function creates a barrier certificate f

Function: create_single_integrator_barrier_certificate_with_boundary
- context: create_single_integrator_barrier_certificate_with_boundary() # Barrier certificates to avoid collision 20 21 _, uni_to_si_states = create_si_to_uni_mapping() # Unicycle to Single Integrator States Mapping 22 23 si_to_uni_dyn = create_si_to_uni_dynamics_with_ba

Function: create_unicycle_barrier_certificate
- context: create_unicycle_barrier_certificate() # Create barrier certificates to avoid collision 20 21 x = r.get_poses() # Get the poses of the Robots 22 23 r.step() # Iterate the simulation 24 25 # While all Robots are not in the goal points... 26 while (np.size(at_pos

Function: create_unicycle_barrier_certificate2
- context: create_unicycle_barrier_certificate2(barrier_gain=500, unsafe_barrier_gain=1e6, safety_radius=0.12, projection_distance=0.05, magnitude_limit=0.2): Listing 52: Barrier Certificate for a Unicycle Model with Dynamic Gains Function This function creates a barrier

Function: create_unicycle_barrier_certificate_with_boundary
- context: create_unicycle_barrier_certificate_with_boundary() # Create barrier certificates to avoid collision 19 20 x = r.get_poses() # Get the poses of robots 21 22 r.step() # Iterate the simulation 23 24 for i in range(iterations): 25 26 x = r.get_poses() # Get the p

Function: create_unicycle_differential_drive_barrier_certificate
- context: create_unicycle_differential_drive_barrier_certificate(max_num_obstacle_points=100, max_num_robots=30, disturbance=5, wheel_vel_limit=12.5, base_length=0.105, wheel_radius=0.016, projection_distance=0.05, barrier_gain=150, safety_radius=0.17): Listing 54: Unic

Function: create_unicycle_differential_drive_barrier_certificate_with_boundary
- context: create_unicycle_differential_drive_barrier_certificate_with_boundary( max_num_obstacle_points=100, max_num_robots=30, disturbance=5, wheel_vel_limit=12.5, base_length=0.105, wheel_radius=0.016, projection_distance=0.05, barrier_gain=150, safety_radius=0.17, bo

Function: robust_barriers
- context: robust_barriers(max_num_obstacles=100, max_num_robots=30, d=5, wheel_vel_limit=12.5, base_length=0.105, wheel_radius=0.016, projection_distance=0.05, gamma=150, safety_radius=0.12): Listing 58: Robust Barriers for Unicycle Differential Drive with Dynamic Gains

Function: si_barrier_cert
- context: si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary() # Barrier certificates to avoid collision 20 21 _, uni_to_si_states = create_si_to_uni_mapping() # Unicycle to Single Integrator States Mapping 22 23 si_to_uni_dyn = create_si_to_un

Function: uni_barrier_cert
- context: uni_barrier_cert = create_unicycle_barrier_certificate() # Create barrier certificates to avoid collision 20 21 x = r.get_poses() # Get the poses of the Robots 22 23 r.step() # Iterate the simulation 24 25 # While all Robots are not in the goal points... 26 wh

## Transformations and mappings (SI <-> UNI)
Function: create_si_to_uni_dynamics
- context: create_si_to_uni_dynamics_with_backwards_motion() # Single Integrator to Unicycle Velocity Commands 24 25 x = r.get_poses() # Get poses of the Robots 26 27 x_si = uni_to_si_states(x) # Transform poses to Single Integrator 28 29 r.step() # Iterate the simulatio

Function: create_si_to_uni_dynamics_with_backwards_motion
- context: create_si_to_uni_dynamics_with_backwards_motion() # Single Integrator to Unicycle Velocity Commands 24 25 x = r.get_poses() # Get poses of the Robots 26 27 x_si = uni_to_si_states(x) # Transform poses to Single Integrator 28 29 r.step() # Iterate the simulatio

Function: create_si_to_uni_mapping
- context: create_si_to_uni_mapping() # Unicycle to Single Integrator States Mapping 22 23 si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion() # Single Integrator to Unicycle Velocity Commands 24 25 x = r.get_poses() # Get poses of the Robots 26 27 x_si = u

Function: create_uni_to_si_dynamics
- context: create_uni_to_si_dynamics(projection_distance=0.05): Listing 32: Function for Mapping from Unicycle Dynamics to Single Integrator Using Projection Distance This function creates a mapping function from unicycle dynamics to single-integrator dynamics. def uni_t

Function: si_to_uni_dyn
- context: si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion() # Single Integrator to Unicycle Velocity Commands 24 25 x = r.get_poses() # Get poses of the Robots 26 27 x_si = uni_to_si_states(x) # Transform poses to Single Integrator 28 29 r.step() # Itera

Function: uni_to_si_dyn
- context: uni_to_si_dynamics(projection_distance=0.05): Listing 32: Function for Mapping from Unicycle Dynamics to Single Integrator Using Projection Distance This function creates a mapping function from unicycle dynamics to single-integrator dynamics. def uni_to_si_dy

Function: uni_to_si_states
- context: uni_to_si_states = create_si_to_uni_mapping() # Unicycle to Single Integrator States Mapping 22 23 si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion() # Single Integrator to Unicycle Velocity Commands 24 25 x = r.get_poses() # Get poses of the Ro

## Utilities and helpers
- completeGL (utility found in PDF)
- create_si_position_controller (utility found in PDF)
- determine_font_size (utility found in PDF)
- determine_marker_size (utility found in PDF)
- generate_initial_conditions (utility found in PDF)
- topological_neighbors (utility found in PDF)

## Common usage pattern (recommended)
Sequence:
- r = Robotarium(...)
- x = r.get_poses()    # 3xN
- x_si = uni_to_si_states(x)    # 2xN single-integrator states
- dxi = si_controller(x_si, goals)   # 2xN
- dxi_safe = si_barrier_cert(dxi, x_si)   # 2xN safe in SI space
- dxu = si_to_uni_dyn(dxi_safe, x)   # 2xN unicycle commands
- r.set_velocities(np.arange(N), dxu)
- r.step()
Notes: Do NOT apply single-integrator barrier certificates to unicycle velocity commands. Use the matching barrier certificate for the control space.

## Constraints, shapes, and limits
- Poses: 3xN arrays [x; y; theta].
- Single-integrator states: 2xN arrays [x; y].
- Controllers typically: inputs (states, goals) -> outputs 2xN velocities.
- Safety radius minimum: Sr >= 0.12 (meters).
- Unicycle magnitude limit default: |v| <= 0.2 (m/s) (as referenced).
- Barrier certificate functions often accept a 'barrier_gain' and 'safety_radius' parameters.

## Source
Extracted from: Robotarium_Python_Guide.pdf
File path: /mnt/data/Robotarium_Python_Guide.pdf