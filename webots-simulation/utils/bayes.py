"""
# bayesian updating approach to updating actions
# adapted from simple-n-armed bandit code 
# ---> converted to base python for HPC 

# source code: 
https://github.com/iosband/ts_tutorial/blob/master/ts_tutorial_intro.ipynb

"""

import random
import math

class NArmedBandit:

    def __init__(self, probs):
        self.probs = probs
        assert all(p >= 0 for p in self.probs)
        assert all(p <= 1 for p in self.probs)

        self.optimal_reward = max(self.probs)
        self.n_arm = len(self.probs)

    def get_observation(self):
        return self.n_arm
    
    def get_optimal_reward(self):
        return max(self.probs)
    
    def get_expected_reward(self, action):
        return self.probs[action]
    
    def get_stochastic_reward(self, action):
        return int(random.random() < self.probs[action])

class NArmedBanditDrift(NArmedBandit):
    def __init__(self, n_arm, a0=1., b0=1., gamma=0.01):  # self.gamma = 0 means no drift
        self.n_arm = n_arm 
        self.a0 = a0
        self.b0 = b0 

        self.prior_success = [a0 for _ in range(n_arm)]
        self.prior_failure = [b0 for _ in range(n_arm)]

        self.gamma = gamma 
        self.probs = [random.betavariate(a0, b0) for _ in range(n_arm)]

    def set_prior(self, prior_success, prior_failure):
        self.prior_success = prior_success
        self.prior_failure = prior_failure

    def get_optimal_reward(self):
        return max(self.probs)
    
    def advance(self, action, reward):
        self.prior_success = [s * (1 - self.gamma) + self.a0 * self.gamma for s in self.prior_success]
        self.prior_failure = [f * (1 - self.gamma) + self.b0 * self.gamma for f in self.prior_failure]

        self.prior_success[action] += reward
        self.prior_failure[action] += 1 - reward

        # resample posterior probabilities 
        self.probs = [random.betavariate(self.prior_success[a], self.prior_failure[a]) for a in range(self.n_arm)]

    def sample_action(self):
        return random.choices(range(self.n_arm), weights=self.probs)[0]