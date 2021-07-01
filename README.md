# dorm
Code for running the D-Optimality Roadmap (DORM) using a determinant bound for belief space planning.

# Related Publications #
Please cite the following papers if you make use of our algorithm or code in any of your own endeavors:

* **[A Determinant Bound on the Covariance Matrix for Path Planning in Gaussian Belief Space](...)**, *J. Strader and Y. Gu*

# Usage #
## Short Range Experiments ##
To reproduce the figures for the short range experiments in the paper, add the files to the path, then run `exp_edge_SI.m` and `exp_edge_DI.m`.

## Long Range Experiments ##
To reproduce the figures for the long range experiments in the paper, add the files to the path, then run `exp_offline_SI.m` and `exp_online_SI.m` for the single integrator and `exp_offline_DI.m` and `exp_online_DI.m` for the double integrator.

*Note:* The offline phase may be skipped by downloading the data generated for the experiments [here](https://drive.google.com/drive/folders/1zgiV1a3GxgNoxA_QA_tgA3r1ysOj_V37?usp=sharing). You must download `bsp_22_SI.mat` and `bsp_22_DI.mat` and add to the path. Then, run`exp_online_SI.m` for the single integrator and `exp_online_DI.m` for the double integrator.

# Requirements #
MATLAB R2017b or higher (earlier versions will likely work as well)
