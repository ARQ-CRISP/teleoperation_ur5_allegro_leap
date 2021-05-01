import os

from independentGP_learner import IndependentGPTrajectoryLearner
from trajectory_collection import Trajectory_Collection_Learner
from trajectory_learner import Trajectory_Learner

prelearnt_trajectories_path = os.path.abspath(__file__).split('__init__.py')[0] + 'prelearnt/'