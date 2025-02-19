# StochasticOptimalControl
# Test

Path integral optimal control is a methodology to solve optimal control problems by transforming the
value function for a specific problem into an expectation over all possible trajectories of the system being
controlled [3]. This transformation is accomplished through the use of the Feynman-Kac lemma [1]. The path
integral formulation being studied in this thesis is derived from an information-theoretic view of stochastic
optimal control that makes a connection between Kullback-Leibler and Path Integral control, as specified in
[1] and [2].

In this thesis, we study the use of incremental sampling based planners, such as Rapidly-exploring
Random Trees (RRT) [3], within the iterative scheme employed in [1] and [2] to find collision-free paths in
a configuration space with obstacles. The challenge with using the path integral framework in this manner
is the presence of local optima, owing to the non-convex nature of the state space [4]. Hence, the overall
quality of the output path from the path integral framework largely hinges on the initial control policy,
i.e., the RRT, that is used as a template to sample local trajectories for optimization. The objective of the
proposed research will be to investigate the use of the path integral control framework with variations of
RRT to find an optimal sampling based planner to use within this framework. This work will extend on the
work presented in [5] by (a) exploring the use of a pure pursuit controller to track steering of trajectories
within RRT in an attempt to produce dynamically feasible paths for systems that can be characterized by
a unicycle model, such as a vehicle [7] and (b) using an asymptotically optimal variation of RRT, such as
RRT* [9].

References
[1] E.A. Theodorou and E. Todorov. Relative entropy and free energy dualities: Connections to path integral
and kl control. In the Proceedings of IEEE Conference on Decision and Control, pages 1466–1473, Dec
2012.

[2] E. Theodorou, D. Krishnamurthy, and E. Todorov. From information theoretic dualities to path integral and kullback-leibler control: Continuous and discrete time formulations. In The Sixteenth Yale
Workshop on Adaptive and Learning Systems.

[3] S. M. LaValle. Rapidly-exploring random trees: A new tool for path planning. TR 98-11, Computer
Science Dept., Iowa State Univ. ¡http://janowiec.cs.iastate.edu/papers/rrt.ps¿, Oct. 1998.

[4] J.-S. Ha and H.-L. Choi, “A topology-guided path integral approach for stochastic optimal control,” in
IEEE International Conference on Robotics and Automation (ICRA), 2016.

[5] O. Arslan, E. A. Theodorou, P. Tsiotras, ”Information-theoretic stochastic optimal control via incremental sampling-based algorithms”, IEEE Symp. Adaptive Dynamic Programming and Reinforcement
Learning, Dec. 2014.

[6] G Williams, A Aldrich, and E Theodorou. Model predictive path integral control using covariance
variable importance sampling. arXiv preprint arXiv:1509.01149, 2015

[7] B. Paden, M. p, S. Z. Yong, D. Yershov, and E. Frazzoli, “A survey of motion planning and control
techniques for self-driving urban vehicles,” IEEE Transactions on Intelligent Vehicles, vol. 1, no. 1, pp.
33–55, March 2016

[8] E. Theodorou, D. Krishnamurthy, and E. Todorov. From information theoretic dualities to path integral and kullback-leibler control: Continuous and discrete time formulations. In The Sixteenth Yale
Workshop on Adaptive and Learning Systems.

[9] S. Karaman and E. Frazzoli, “Incremental sampling-based algorithms for optimal motion planning,” in
Proc. of Robot.: Sci. and Syst.,Zaragoza, Spain, June 2010.
