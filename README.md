# dorm
Code for running the D-Optimality Roadmap (DORM) using a determinant bound for belief space planning. The code may also be used for running the belief roadmap (BRM) and the belief roadmap search (BRMS).

# Related Publications #
Please cite the following paper if you make use of our algorithm or code in any of your own endeavors:

* **[A Determinant Bound on the Covariance Matrix for Path Planning in Gaussian Belief Space](...)**, *J. Strader and Y. Gu*

# Requirements #
MATLAB R2017b or higher (earlier versions will likely work as well)

# Reproducing the Figures in the Paper #
## Short Range Experiments ##
To reproduce the figures for the short range experiments in the paper, add all files to the path, then run `exp_edge_SI.m` and `exp_edge_DI.m`.

## Long Range Experiments ##
To reproduce the figures for the long range experiments in the paper, add all files to the path, then run `exp_offline_SI.m` and `exp_online_SI.m` for the single integrator and `exp_offline_DI.m` and `exp_online_DI.m` for the double integrator. 

The offline phase saves a file with named with the following format: `bsp_mm-dd-yyyy HH-MM.mat`. Before running the scripts for the online phase, change the `load(...)` commands at the beginning of the online scripts to the name of the file generated in the offline phase.

**Note:** The offline phase may be skipped by downloading the data generated for the experiments [here](https://drive.google.com/drive/folders/1zgiV1a3GxgNoxA_QA_tgA3r1ysOj_V37?usp=sharing). You must download `bsp_58_SI.mat` and `bsp_58_DI.mat` and add to the path. Then, run`exp_online_SI.m` for the single integrator and `exp_online_DI.m` for the double integrator. The data generated for the offline phase for additional seeds is provided at the link. However, if using the data for a different seed, change the `load(...)` command at the beginning of `exp_online_SI.m` and `exp_online_DI.m`.

# Usage #
## Robot Model ##
To apply implemented algorithms (e.g., DORM, BRMS, or BRM) for a robot model not already implemented in the models folder, a model of the robot must be defined, which inherits the @GenericStateSpaceModel. See the @SingleIntegrator2DwithGPS and @DoubleIntegrator2DwithGPS for examples on implementing models.

The functions that are required to be implemented in the @GenericStateSpaceModel are the following:  
`Q = get_process_noise_covariance(obj, x, u)`  
`A = get_process_jacobian(obj, x, u)`  
`A = get_control_jacobian(obj, x, u)`  
`L = get_process_noise_jacobian(obj, x, u)`  
`is_acquired = is_measurement_acquired(obj, x)`  
`R = get_measurement_noise_covariance(obj, x)`  
`H = get_measurement_jacobian(obj, x)`  
`M = get_measurement_noise_jacobian(obj, x)`  
`[x, u, K] = get_states_and_control_inputs(obj, xi, xf)`  

## Offline Phase ##
To run the offline phase, a belief space planning object must be defined, which takes a robot model (inherited from @GenericStateSpaceModel), configuration space (state that are not derivatives), velocity space (states that are derivatives), obstacles (represented using the @Rectangle class), and the connection radius (Euclidean distance between configurations for connective vertices) as input.

After initializing the `@BSP` object, run `@BSP.run_offline(N)` where `N` is the number of iterations for sampling vertices to construct the roadmap. Use `Help BSP.run_offline` for additional details.

## Online Phase ##
To run the online phase, the required inputs are the initial mean of the belief, initial covariance matrix of the belief, the goal state, the parameter epsilon for the partial ordering, the guessed maximum eigenvalue occuring along the path (use -1 for to determine automatically), and the method (1=DORM, 2=BRMS, 3=BRM). Note a best-first search is used for DORM, BRMS, and BRM during the online phase. Use `Help BSP.run_online` for additional details.

