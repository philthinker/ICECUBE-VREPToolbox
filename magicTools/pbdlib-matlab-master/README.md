# pbdlib-matlab

PbDlib is a set of tools combining statistical learning, dynamical systems and optimal control approaches for programming-by-demonstration applications.<br>
Other versions of the library are available at http://www.idiap.ch/software/pbdlib/ (in C++ or Python, with currently fewer functionalities). 

### References

Did you find PbDLib useful for your research? Please acknowledge the authors in any academic publications that used parts of these codes.

#### Ref. [1] 
**Tutorial (GMM, TP-GMM, MFA, MPPCA, GMR, LWR, GPR, MPC, LQR, trajGMM)**<br>
[Link to publication](http://calinon.ch/papers/Calinon-JIST2015.pdf)
```
@article{Calinon16JIST,
	author="Calinon, S.",
	title="A Tutorial on Task-Parameterized Movement Learning and Retrieval",
	journal="Intelligent Service Robotics",
	publisher="Springer Berlin Heidelberg",
	year="2016",
	volume="9",
	number="1",
	pages="1--29",
	doi="10.1007/s11370-015-0187-9",
}
```

#### Ref. [2] 
**HMM, HSMM**<br>
[Link to publication](http://calinon.ch/papers/Rozo-Frontiers2016.pdf)
```
@article{Rozo16Frontiers,
	author="Rozo, L. and Silv\'erio, J. and Calinon, S. and Caldwell, D. G.",
	title="Learning Controllers for Reactive and Proactive Behaviors in Human-Robot Collaboration",
	journal="Frontiers in Robotics and {AI}",
	year="2016",
	month="June",
	volume="3",
	number="30",
	pages="1--11",
	doi="10.3389/frobt.2016.00030"
}
```

#### Ref. [3] 
**Riemannian manifolds (S2,S3)**<br>
[Link to publication](http://calinon.ch/papers/Zeestraten-RAL2017.pdf)
```
@article{Zeestraten17RAL,
	author="Zeestraten, M. J. A. and Havoutis, I. and Silv\'erio, J. and Calinon, S. and Caldwell, D. G.",
	title="An Approach for Imitation Learning on {R}iemannian Manifolds",
	journal="{IEEE} Robotics and Automation Letters ({RA-L})",
	year="2017",
	month="June",
	volume="2",
	number="3",
	pages="1240--1247"
	doi="10.1109/LRA.2017.2657001",
}
```

#### Ref. [4] 
**Riemannian manifolds (S+)**<br>
[Link to publication](http://calinon.ch/papers/Jaquier-IROS2017.pdf)
```
@inproceedings{Jaquier17IROS,
	author="Jaquier, N. and Calinon, S.", 
	title="Gaussian Mixture Regression on Symmetric Positive Definite Matrices Manifolds: Application to Wrist Motion Estimation with {sEMG}",
	booktitle="Proc. {IEEE/RSJ} Intl Conf. on Intelligent Robots and Systems ({IROS})",
	year="2017",
	month="September",
	address="Vancouver, Canada",
	pages="59--64"
}
```

#### Ref. [5] 
**Semi-tied GMM**<br>
[Link to publication](http://calinon.ch/papers/Tanwani-RAL2016.pdf)
```
@article{Tanwani16RAL,
	author="Tanwani, A. K. and Calinon, S.",
	title="Learning Robot Manipulation Tasks with Task-Parameterized Semi-Tied Hidden Semi-{M}arkov Model",
	journal="{IEEE} Robotics and Automation Letters ({RA-L})",
	year="2016",
	month="January",
	volume="1",
	number="1",
	pages="235--242",
	doi="10.1109/LRA.2016.2517825"
}
```

#### Ref. [6] 
**DP-means**<br>
[Link to publication](http://calinon.ch/papers/Bruno-AURO2017.pdf)
```
@article{Bruno17AURO,
	author="Bruno, D. and Calinon, S. and Caldwell, D. G.",
	title="Learning Autonomous Behaviours for the Body of a Flexible Surgical Robot",
	journal="Autonomous Robots",
	year="2017",
	month="February",
	volume="41",
	number="2",
	pages="333--347",
	doi="10.1007/s10514-016-9544-6"
}
```

#### Ref. [7] 
**Manipulability ellispoids**<br>
[Link to publication](http://calinon.ch/papers/Rozo-IROS2017.pdf)
```
@inproceedings{Rozo17IROS,
	author="Rozo, L. and Jaquier, N. and Calinon, S. and Caldwell, D. G.", 
	title="Learning Manipulability Ellipsoids for Task Compatibility in Robot Manipulation",
	booktitle="Proc. {IEEE/RSJ} Intl Conf. on Intelligent Robots and Systems ({IROS})",
	year="2017",
	month="September",
	address="Vancouver, Canada",
	pages="3183--3189"
}
```

#### Ref. [8] 
**Keypoint-based motion through MPC**<br>
[Link to publication](http://calinon.ch/papers/Berio-GI2017.pdf)
```
@inproceedings{Berio17GI,
	author="Berio, D. and Calinon, S. and Fol Leymarie, F.",
	title="Generating Calligraphic Trajectories with Model Predictive Control",
	booktitle="Proc. 43rd Conf. on Graphics Interface",
	year="2017",
	month="May",
	address="Edmonton, AL, Canada",
	pages="132--139",
	doi="10.20380/GI2017.17"
}
```

#### Ref. [9] 
**Manipulability ellipsoids tracking**<br>
[Link to publication](http://calinon.ch/paper3066.htm)
```
@incollection{@article{Jaquier18RSS,
  	author="Jaquier, N and Rozo, L. and Caldwell, D. G. and Calinon, S.",
  	title="Geometry-aware Tracking of Manipulability Ellipsoids",
  	year="2018",
	booktitle = "Robotics: Science and Systems ({R:SS})",
	address = "Pittsburgh, USA"
}
```

### List of examples

All the examples are located in the main folder, and the functions are located in the `m_fcts` folder.

| Filename | Ref. | Description |
|----------|------|-------------|
| [benchmark_DS_GP_GMM01.m](./demos/benchmark_DS_GP_GMM01.m) | [[1]](#ref-1) | Benchmark of task-parameterized model based on Gaussian process regression, with trajectory model (Gaussian mixture model encoding), and DS-GMR used for reproduction |
| [benchmark_DS_GP_raw01.m](./demos/benchmark_DS_GP_raw01.m) | [[1]](#ref-1) | Benchmark of task-parameterized model based on Gaussian process regression, with raw trajectory, and spring-damper system used for reproduction |
| [benchmark_DS_PGMM01.m](./demos/benchmark_DS_PGMM01.m) | [[1]](#ref-1) | Benchmark of task-parameterized model based on parametric Gaussian mixture model, and DS-GMR used for reproduction |
| [benchmark_DS_TP_GMM01.m](./demos/benchmark_DS_TP_GMM01.m) | [[1]](#ref-1) | Benchmark of task-parameterized Gaussian mixture model (TP-GMM), with DS-GMR used for reproduction |
| [benchmark_DS_TP_GP01.m](./demos/benchmark_DS_TP_GP01.m) | [[1]](#ref-1) | Benchmark of task-parameterized Gaussian process (nonparametric task-parameterized method) |
| [benchmark_DS_TP_LWR01.m](./demos/benchmark_DS_TP_LWR01.m) | [[1]](#ref-1) | Benchmark of task-parameterized locally weighted regression (nonparametric task-parameterized method) |
| [benchmark_DS_TP_MFA01.m](./demos/benchmark_DS_TP_MFA01.m) | [[1]](#ref-1) | Benchmark of task-parameterized mixture of factor analyzers (TP-MFA), with DS-GMR used for reproduction |
| [benchmark_DS_TP_trajGMM01.m](./demos/benchmark_DS_TP_trajGMM01.m) | [[1]](#ref-1) | Benchmark of task-parameterized Gaussian mixture model (TP-GMM), with DS-GMR used for reproduction |
| [demo_affineTransform01.m](./demos/demo_affineTransform01.m) | [[1]](#ref-1) | Affine transformations of raw data as pre-processing step to train a task-parameterized model |
| [demo_batchLQR01.m](./demos/demo_batchLQR01.m) | [[1]](#ref-1) | Controller retrieval through a batch solution of linear quadratic optimal control (unconstrained linear MPC), by relying on a Gaussian mixture model (GMM) encoding of position and velocity data (see also demo_iterativeLQR01) |
| [demo_batchLQR02.m](./demos/demo_batchLQR02.m) | [[1]](#ref-1) | Same as demo_batchLQR01 but with only position data |
| [demo_batchLQR_augmSigma01.m](./demos/demo_batchLQR_augmSigma01.m) | [[1]](#ref-1) | Batch LQR with augmented covariance to transform a tracking problem to a regulation problem |
| [demo_batchLQR_online01.m](./demos/demo_batchLQR_online01.m) | [[1]](#ref-1) | Batch solution of linear quadratic optimal control (unconstrained linear MPC) computed in an online manner, by relying on a GMM encoding of position and velocity data |
| [demo_batchLQR_viapoints01.m](./demos/demo_batchLQR_viapoints01.m) | [[8]](#ref-8) | Keypoint-based motion through MPC, with a GMM encoding of position and velocity |
| [demo_batchLQR_viapoints02.m](./demos/demo_batchLQR_viapoints02.m) | [[8]](#ref-8) | Same as demo_batchLQR_viapoints01 with only position encoding |
| [demo_batchLQR_viapoints03.m](./demos/demo_batchLQR_viapoints03.m) | [[8]](#ref-8) | Equivalence between cubic Bezier curve and batch LQR with double integrator |
| [demo_DMP01.m](./demos/demo_DMP01.m) | [[1]](#ref-1) | Dynamic movement primitive (DMP) encoding with radial basis functions |
| [demo_DMP02.m](./demos/demo_DMP02.m) | [[1]](#ref-1) | Generalization of dynamic movement primitive (DMP) with polynomial fitting using radial basis functions |
| [demo_DMP_GMR01.m](./demos/demo_DMP_GMR01.m) | [[1]](#ref-1) | Emulation of a standard dynamic movement primitive (DMP) by using a GMM with diagonal covariance matrix, and retrieval computed through Gaussian mixture regression (GMR) | 
| [demo_DMP_GMR02.m](./demos/demo_DMP_GMR02.m) | [[1]](#ref-1) | Same as demo_DMP_GMR01 but with full covariance matrices coordinating the different variables | 
| [demo_DMP_GMR03.m](./demos/demo_DMP_GMR03.m) | [[1]](#ref-1) | Same as demo_DMP_GMR02 but with GMR used to regenerate the path of a spring-damper system instead of encoding the nonlinear forcing term | 
| [demo_DMP_GMR04.m](./demos/demo_DMP_GMR04.m) | [[1]](#ref-1) | Same as demo_DMP_GMR03 by using the task-parameterized model formalism | 
| [demo_DMP_GMR_LQR01.m](./demos/demo_DMP_GMR_LQR01.m) | [[1]](#ref-1) | Same as demo_DMP_GMR04 but with LQR used to refine the parameters of the spring-damper system | 
| [demo_DMP_GMR_LQR02.m](./demos/demo_DMP_GMR_LQR02.m) | [[1]](#ref-1) | Same as demo_DMP_GMR_LQR01 with perturbations added to show the benefit of full covariance to coordinate disturbance rejection | 
| [demo_DPMeans_Online01.m](./demos/demo_DPMeans_Online01.m) | [[6]](#ref-6) | Online clustering with DP-means algorithm |
| [demo_DSGMR01.m](./demos/demo_DSGMR01.m) | [[1]](#ref-1) | Gaussian mixture model (GMM), with Gaussian mixture regression(GMR) and dynamical systems used for reproduction, with decay variable used as input (as in DMP) |
| [demo_DTW01.m](./demos/demo_DTW01.m) | [[1]](#ref-1) | Trajectory realignment through dynamic time warping (DTW) |
| [demo_Gaussian01.m](./demos/demo_Gaussian01.m) | [[1]](#ref-1) | Use of Chi-square values to determine the percentage of data within the contour of a multivariate normal distribution |
| [demo_Gaussian02.m](./demos/demo_Gaussian02.m) | [[1]](#ref-1) | Conditional probability with a multivariate normal distribution |
| [demo_Gaussian03.m](./demos/demo_Gaussian03.m) | [[1]](#ref-1) | Gaussian conditioning with uncertain inputs |
| [demo_GMM01.m](./demos/demo_GMM01.m) | [[1]](#ref-1) | Gaussian mixture model (GMM) parameters estimation |
| [demo_GMM02.m](./demos/demo_GMM02.m) | [[1]](#ref-1) | GMM with different covariance structures |
| [demo_GMR01.m](./demos/demo_GMR01.m) | [[1]](#ref-1) | GMM and time-based Gaussian mixture regression (GMR) used for reproduction |
| [demo_GMR02.m](./demos/demo_GMR02.m) | [[1]](#ref-1) | GMR computed with precision matrices instead of covariances |
| [demo_GMR03.m](./demos/demo_GMR03.m) | [[1]](#ref-1) | Chain rule with Gaussian conditioning |
| [demo_GMR_3Dviz01.m](./demos/demo_GMR_3Dviz01.m) | [[1]](#ref-1) | 3D visualization of a GMM with time-based GMR used for reproduction |
| [demo_GMR_polyFit01.m](./demos/demo_GMR_polyFit01.m) | [[1]](#ref-1) | Polynomial fitting with multivariate GMR |
| [demo_GMR_polyFit02.m](./demos/demo_GMR_polyFit02.m) | [[1]](#ref-1) | Polynomial fitting of handwriting motion with multivariate GMR |
| [demo_GPR01.m](./demos/demo_GPR01.m) | [[1]](#ref-1) | Gaussian process regression (GPR) |
| [demo_GPR02.m](./demos/demo_GPR02.m) | [[1]](#ref-1) | GPR with stochastic samples from the prior and the posterior |
| [demo_GPR03.m](./demos/demo_GPR03.m) | [[1]](#ref-1) | GPR with periodic kernel function |
| [demo_GPR_TP01.m](./demos/demo_GPR_TP01.m) | [[1]](#ref-1) | Use of GPR as a task-parameterized model, with DS-GMR used to retrieve continuous movements |
| [demo_grabData01.m](./demos/demo_grabData01.m) | [[1]](#ref-1) | Collect movement data from mouse cursor |
| [demo_HDDC01.m](./demos/demo_HDDC01.m) | [[1]](#ref-1) | High Dimensional Data Clustering (HDDC, or HD-GMM) |
| [demo_HMM01.m](./demos/demo_HMM01.m) | [[2]](#ref-2) | Hidden Markov model (HMM) with single Gaussian as emission distribution |
| [demo_HMM_Viterbi01.m](./demos/demo_HMM_Viterbi01.m) | [[2]](#ref-2) | Viterbi decoding in HMM to estimate best state sequence from observations |
| [demo_HSMM01.m](./demos/demo_HSMM01.m) | [[2]](#ref-2) | Variable duration model implemented as a hidden semi-Markov model (HSMM), by encoding the state duration after EM |
| [demo_iterativeLQR01.m](./demos/demo_iterativeLQR01.m) | [[1]](#ref-1) | Iterative solution of linear quadratic tracking problem (finite horizon, unconstrained linear MPC), by relying on a GMM encoding of position and velocity data (see also demo_batchLQR01) |
| [demo_iterativeLQR02.m](./demos/demo_iterativeLQR02.m) | [[1]](#ref-1) | Same as demo_iterativeLQR01 with only position data |
| [demo_iterativeLQR_augmSigma01.m](./demos/demo_iterativeLQR_augmSigma01.m) | [[1]](#ref-1) | Iterative LQR with augmented covariance to transform the tracking problem to a regulation problem |
| [demo_iterativeLQR_augmSigma_online01.m](./demos/demo_iterativeLQR_augmSigma_online01.m) | [[1]](#ref-1) | Same as demo_iterativeLQR_augmSigma01 but recomputed in an online manner |
| [demo_LQR_infHor01.m](./demos/demo_LQR_infHor01.m) | [[1]](#ref-1) | Continuous infinite horizon linear quadratic tracking, by relying on a GMM encoding of position and velocity data |
| [demo_LQR_infHor02.m](./demos/demo_LQR_infHor02.m) | [[1]](#ref-1) | Discrete infinite horizon linear quadratic tracking, by relying on a GMM encoding of position and velocity data |
| [demo_manipulabilityTracking_mainTask01.m](./demos/demo_manipulabilityControl_mainTask01.m) | [[9]](#ref-9) | Tracking of a desired manipulability ellipsoid as the main task |
| [demo_manipulabilityTracking_mainTask02.m](./demos/demo_manipulabilityControl_mainTask02.m) | [[9]](#ref-9) | Tracking of a desired manipulability ellipsoid as the main task using precision matrice as controller gain |
| [demo_manipulabilityTracking_secondaryTask01.m](./demos/demo_manipulabilityControl_secondTask01.m) | [[9]](#ref-9) | Tracking of a desired manipulability ellipsoid as the secondary task with position tracking as main task |
| [demo_manipulabilityTransfer01.m](./demos/demo_manipulabilityTransfer01.m) | [[7]](#ref-7) | Use of robot redundancy to track desired manipulability ellipsoid |
| [demo_manipulabilityTransfer02.m](./demos/demo_manipulabilityTransfer02.m) | [[7]](#ref-7) | Learning and reproduction of manipulability ellipsoid profiles |
| [demo_manipulabilityTransfer02b.m](./demos/demo_manipulabilityTransfer02b.m) | [[7]](#ref-7) | Learning and reproduction of manipulability ellipsoid profiles (numerical version) |
| [demo_MFA01.m](./demos/demo_MFA01.m) | [[1]](#ref-1) | Mixture of factor analyzers (MFA) |
| [demo_MPPCA01.m](./demos/demo_MPPCA01.m) | [[1]](#ref-1) | Mixture of probabilistic principal component analyzers (MPPCA) |
| [demo_regularization01.m](./demos/demo_regularization01.m) | [[1]](#ref-1) | Regularization of GMM parameters with minimum admissible eigenvalue |
| [demo_regularization02.m](./demos/demo_regularization02.m) | [[1]](#ref-1) | Regularization of GMM parameters with the addition of a small circular covariance |
| [demo_Riemannian_cov_GMM01.m](./demos/demo_Riemannian_cov_GMM01.m) | [[4]](#ref-4) | GMM for covariance data by relying on Riemannian manifold |
| [demo_Riemannian_cov_GMR01.m](./demos/demo_Riemannian_cov_GMR01.m) | [[4]](#ref-4) | GMR with time as input and covariance data as output by relying on Riemannian manifold |
| [demo_Riemannian_cov_GMR02.m](./demos/demo_Riemannian_cov_GMR02.m) | [[4]](#ref-4) | GMR with time as input and position vector as output with comparison between computation in vector and matrix forms |
| [demo_Riemannian_cov_GMR03.m](./demos/demo_Riemannian_cov_GMR03.m) | [[4]](#ref-4) | GMR with vector as input and covariance data as output by relying on Riemannian manifold |
| [demo_Riemannian_cov_interp01.m](./demos/demo_Riemannian_cov_interp01.m) | [[4]](#ref-4) | Covariance interpolation on Riemannian manifold |
| [demo_Riemannian_cov_interp02.m](./demos/demo_Riemannian_cov_interp02.m) | [[4]](#ref-4) | Covariance interpolation on Riemannian manifold from a GMM with augmented covariances |
| [demo_Riemannian_cov_interp03.m](./demos/demo_Riemannian_cov_interp03.m) | [[4]](#ref-4) | Trajectory morphing through covariance interpolation on Riemannian manifold (with augmented Gaussian trajectory distribution) |
| [demo_Riemannian_cov_search01.m](./demos/demo_Riemannian_cov_search01.m) | [[4]](#ref-4) | EM-based stochastic optimization of covariance on Riemannian manifold |
| [demo_Riemannian_cov_vecTransp01.m](./demos/demo_Riemannian_cov_vecTransp01.m) | [[4]](#ref-4) | Verification of angle conservation in parallel transport on the symmetric positive definite |
| [demo_Riemannian_sphere_GaussProd01.m](./demos/demo_Riemannian_sphere_GaussProd01.m) | [[3]](#ref-3) | Product of Gaussians on a sphere by relying on Riemannian manifold |
| [demo_Riemannian_sphere_GMM01.m](./demos/demo_Riemannian_sphere_GMM01.m) | [[3]](#ref-3) | GMM for data on a sphere by relying on Riemannian manifold |
| [demo_Riemannian_sphere_GMR01.m](./demos/demo_Riemannian_sphere_GMR01.m) | [[3]](#ref-3) | GMR with input and output data on a sphere by relying on Riemannian manifold |
| [demo_Riemannian_sphere_GMR02.m](./demos/demo_Riemannian_sphere_GMR02.m) | [[3]](#ref-3) | GMR with time as input and spherical data as output by relying on Riemannian manifold |
| [demo_Riemannian_sphere_GMR03.m](./demos/demo_Riemannian_sphere_GMR03.m) | [[3]](#ref-3) | GMR with 3D Euclidean data as input and spherical data as output by relying on Riemannian manifold |
| [demo_Riemannian_sphere_GMR04.m](./demos/demo_Riemannian_sphere_GMR04.m) | [[3]](#ref-3) | GMR with input data on a sphere and output data in Eudlidean space by relying on Riemannian manifold |
| [demo_Riemannian_sphere_TPGMM01.m](./demos/demo_Riemannian_sphere_TPGMM01.m) | [[3]](#ref-3) | TP-GMM for data on a sphere by relying on Riemannian manifold (with single frame) |
| [demo_Riemannian_sphere_TPGMM02.m](./demos/demo_Riemannian_sphere_TPGMM02.m) | [[3]](#ref-3) | TP-GMM for data on a sphere by relying on Riemannian manifold (with two frames) |
| [demo_Riemannian_sphere_vecTransp01.m](./demos/demo_Riemannian_sphere_vecTransp01.m) | [[3]](#ref-3) | Parallel transport on a sphere |
| [demo_Riemannian_quat_GMM01.m](./demos/demo_Riemannian_quat_GMM01.m) | [[3]](#ref-3) | GMM for unit quaternion data by relying on Riemannian manifold |
| [demo_Riemannian_quat_GMR01.m](./demos/demo_Riemannian_quat_GMR01.m) | [[3]](#ref-3) | GMR with unit quaternions as input and output data by relying on Riemannian manifold |
| [demo_Riemannian_quat_GMR02.m](./demos/demo_Riemannian_quat_GMR02.m) | [[3]](#ref-3) | GMR with time as input and unit quaternion as output by relying on Riemannian manifold |
| [demo_Riemannian_quat_vecTransp01.m](./demos/demo_Riemannian_quat_vecTransp01.m) | [[3]](#ref-3) | Parallel transport for unit quaternions |
| [demo_SEDS01.m](./demos/demo_SEDS01.m) | [[1]](#ref-1) | Continuous autonomous dynamical system with state-space encoding using GMM, with GMR used for reproduction by using a constrained optimization similar to the SEDS approach |
| [demo_SEDS_discrete01.m](./demos/demo_SEDS_discrete01.m) | [[1]](#ref-1) | Discrete autonomous dynamical system with state-space encoding using GMM, with GMR used for reproduction by using a constrained optimization similar to the SEDS approach |
| [demo_semitiedGMM01.m](./demos/demo_semitiedGMM01.m) | [[5]](#ref-5) | Semi-tied Gaussian Mixture Model by tying the covariance matrices of a GMM with a set of common basis vectors |
| [demo_stdPGMM01.m](./demos/demo_stdPGMM01.m) | [[1]](#ref-1) | Parametric Gaussian mixture model (PGMM) used as a task-parameterized model, with DS-GMR employed to retrieve continuous movements |
| [demo_testDampingRatio01.m](./demos/demo_testDampingRatio01.m) | [[1]](#ref-1) | Test with critically damped system and ideal underdamped system |
| [demo_testLQR01.m](./demos/demo_testLQR01.m) | [[1]](#ref-1) | Test of linear quadratic regulation (LQR) with different variance in the data |
| [demo_testLQR02.m](./demos/demo_testLQR02.m) | [[1]](#ref-1) | Test of LQR with evaluation of the damping ratio found by the system |
| [demo_testLQR03.m](./demos/demo_testLQR03.m) | [[1]](#ref-1) | Comparison of LQR with finite and infinite time horizons |
| [demo_testLQR04.m](./demos/demo_testLQR04.m) | [[1]](#ref-1) | Demonstration of the coordination capability of linear quadratic optimal control (unconstrained linear MPC) when combined with full precision matrices |
| [demo_TPbatchLQR01.m](./demos/demo_TPbatchLQR01.m) | [[1]](#ref-1) | Linear quadratic control (unconstrained linear MPC) acting in multiple frames, which is equivalent to a product of Gaussian controllers through a TP-GMM representation |
| [demo_TPGMM01.m](./demos/demo_TPGMM01.m) | [[1]](#ref-1) | Task-parameterized Gaussian mixture model (TP-GMM) encoding |
| [demo_TPGMR01.m](./demos/demo_TPGMR01.m) | [[1]](#ref-1) | TP-GMM with GMR used for reproduction (without dynamical system) |
| [demo_TPGMR_DS01.m](./demos/demo_TPGMR_DS01.m) | [[1]](#ref-1) | Dynamical system with constant gains used with a task-parameterized model |
| [demo_TPGMR_LQR01.m](./demos/demo_TPGMR_LQR01.m) | [[1]](#ref-1) | Finite horizon LQR used with a task-parameterized model | 
| [demo_TPGMR_LQR02.m](./demos/demo_TPGMR_LQR02.m) | [[1]](#ref-1) | Infinite horizon LQR used with a task-parameterized model | 
| [demo_TPGP01.m](./demos/demo_TPGP01.m) | [[1]](#ref-1) | Task-parameterized Gaussian process regression (TP-GPR) |
| [demo_TPHDDC01.m](./demos/demo_TPHDDC01.m) | [[1]](#ref-1) | Task-parameterized high dimensional data clustering (TP-HDDC) |
| [demo_TPMFA01.m](./demos/demo_TPMFA01.m) | [[1]](#ref-1) | Task-parameterized mixture of factor analyzers (TP-MFA), without motion retrieval |
| [demo_TPMPC01.m](./demos/demo_TPMPC01.m) | [[1]](#ref-1) | Task-parameterized model encoding position data, with MPC used to track the associated stepwise reference path |
| [demo_TPMPC02.m](./demos/demo_TPMPC02.m) | [[1]](#ref-1) | Same as demo_TPMPC01 with a generalized version of MPC used to track associated stepwise reference paths in multiple frames |
| [demo_TPMPPCA01.m](./demos/demo_TPMPPCA01.m) | [[1]](#ref-1) | Task-parameterized mixture of probabilistic principal component analyzers (TP-MPPCA) |
| [demo_TPproMP01.m](./demos/demo_TPproMP01.m) | [[1]](#ref-1) | Task-parameterized probabilistic movement primitives (TP-ProMP) |
| [demo_TPtrajDistrib01.m](./demos/demo_TPtrajDistrib01.m) | [[1]](#ref-1) | Task-parameterized model with trajectory distribution and eigendecomposition |
| [demo_TPtrajGMM01.m](./demos/demo_TPtrajGMM01.m) | [[1]](#ref-1) | Task-parameterized model with trajectory-GMM encoding |
| [demo_trajDistrib01.m](./demos/demo_trajDistrib01.m) | [[1]](#ref-1) | Stochastic sampling with Gaussian trajectory distribution |
| [demo_trajGMM01.m](./demos/demo_trajGMM01.m) | [[1]](#ref-1) | Reproduction of trajectory with a GMM with dynamic features (trajectory-GMM) |
| [demo_trajHSMM01.m](./demos/demo_trajHSMM01.m) | [[2]](#ref-2) | Trajectory synthesis with an HSMM with dynamic features (trajectory-HSMM) |
| [demo_trajMFA01.m](./demos/demo_trajMFA01.m) | [[1]](#ref-1) | Trajectory model with either a mixture of factor analysers (MFA), a mixture of probabilistic principal component analyzers (MPPCA), or a high-dimensional data clustering approach (HD-GMM) |
| [demoIK_nullspace_TPGMM01.m](./demos/demoIK_nullspace_TPGMM01.m) | [[1]](#ref-1) | IK with nullspace treated with task-parameterized GMM (bimanual tracking task, version with 4 frames) |
| [demoIK_pointing_TPGMM01.m](./demos/demoIK_pointing_TPGMM01.m) | [[1]](#ref-1) | Task-parameterized GMM to encode pointing direction by considering nullspace constraint (4 frames) (example with two objects and robot frame, starting from the same initial pose (nullspace constraint), by using a single Euler orientation angle and 3 DOFs robot) |

### Usage

Examples starting with `demo_` can be run as examples. The codes are compatible with both Matlab and GNU Octave.

### Gallery

[demo\_GMR\_3Dviz01.m](./demos/demo_GMR_3Dviz01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-matlab/raw/master/images/demo_GMR_3Dviz01.png)

***

[demo\_GMRpolyFit02.m](./demos/demo_GMRpolyFit02.m)

![](https://gitlab.idiap.ch/rli/pbdlib-matlab/raw/master/images/demo_GMRpolyFit02.png)

***

[demo\_HMM01.m](./demos/demo_HMM01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-matlab/raw/master/images/demo_HMM01.png)

***

[demo\_iterativeLQR01.m](./demos/demo_iterativeLQR01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-matlab/raw/master/images/demo_iterativeLQR01.png)

***

[demo\_Riemannian\_cov\_GMR01.m](./demos/demo_Riemannian_cov_GMR01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-matlab/raw/master/images/demo_Riemannian_cov_GMR01.png)

***

[demo\_Riemannian\_cov\_interp01.m](./demos/demo_Riemannian_cov_interp01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-matlab/raw/master/images/demo_Riemannian_cov_interp01.png)

***

[demo\_Riemannian\_cov\_interp02.m](./demos/demo_Riemannian_cov_interp02.m)

![](https://gitlab.idiap.ch/rli/pbdlib-matlab/raw/master/images/demo_Riemannian_cov_interp02.png)

***

[demo\_Riemannian\_cov\_interp03.m](./demos/demo_Riemannian_cov_interp03.m)

![](https://gitlab.idiap.ch/rli/pbdlib-matlab/raw/master/images/demo_Riemannian_cov_interp03.png)

***

[demo\_Riemannian\_sphere\_GMM01.m](./demos/demo_Riemannian_sphere_GMM01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-matlab/raw/master/images/demo_Riemannian_sphere_GMM01.png)

***

[demo\_semitiedGMM01.m](./demos/demo_semitiedGMM01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-matlab/raw/master/images/demo_semitiedGMM01.png)

***

[demo\_TPbatchLQR01.m](./demos/demo_TPbatchLQR01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-matlab/raw/master/images/demo_TPbatchLQR01.png)

***

[demo\_TPGMR01.m](./demos/demo_TPGMR01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-matlab/raw/master/images/demo_TPGMR01.png)

***

[demo\_TPproMP01.m](./demos/demo_TPproMP01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-matlab/raw/master/images/demo_TPproMP01.png)

***

[demo\_trajHSMM01.m](./demos/demo_trajHSMM01.m)

![](https://gitlab.idiap.ch/rli/pbdlib-matlab/raw/master/images/demo_trajHSMM01.png)

### License

The Matlab/GNU Octave version of PbDlib is currently maintained by Sylvain Calinon, Idiap Research Institute. 

Copyright (c) 2015 Idiap Research Institute, http://idiap.ch/

PbDlib is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License version 3 as published by the Free Software Foundation.

PbDlib is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with PbDlib. If not, see <http://www.gnu.org/licenses/>.

