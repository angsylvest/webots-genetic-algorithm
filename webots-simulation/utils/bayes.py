"""
# bayesian updating approach to updating actions
# adapted from simple-n-armed bandit code 
# ---> converted to base python for HPC 

# source code: 
https://github.com/iosband/ts_tutorial/blob/master/ts_tutorial_intro.ipynb

"""

import random
import math

# ensure that we can access utils package to streamline tasks 
import utils.globals as globals

use_thompson = globals.thompson_sampling

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

    def decay_gamma(self, decay_rate = 0.95):
        self.gamma = max(self.gamma * decay_rate, 0.001)  # Ensure gamma doesn't become too small

    def reset_prior(self):
        self.prior_success = [self.a0 for _ in range(self.n_arm)]
        self.prior_failure = [self.b0 for _ in range(self.n_arm)]

    def set_prior(self, prior_success, prior_failure):
        self.prior_success = prior_success
        self.prior_failure = prior_failure

    def get_optimal_reward(self):
        return max(self.probs)
    
    def advance(self, action, reward):

        if globals.decay: 
            self.decay_gamma()

        self.prior_success = [s * (1 - self.gamma) + self.a0 * self.gamma for s in self.prior_success]
        self.prior_failure = [f * (1 - self.gamma) + self.b0 * self.gamma for f in self.prior_failure]

        self.prior_success[action] += reward
        self.prior_failure[action] += 1 - reward
        
        # Ensure that prior_success and prior_failure are always greater than 0.0
        self.prior_success = [max(s, 0.001) for s in self.prior_success]
        self.prior_failure = [max(f, 0.001) for f in self.prior_failure]


        # resample posterior probabilities 
        self.probs = [random.betavariate(self.prior_success[a], self.prior_failure[a]) for a in range(self.n_arm)]

    def sample_action(self):
        if use_thompson:
            # Use Thompson Sampling
            sampled_theta = [random.betavariate(self.prior_success[a], self.prior_failure[a]) for a in range(self.n_arm)]
            return max(range(self.n_arm), key=lambda a: sampled_theta[a])
        
        else: 
            # use categorical sampling 
            return random.choices(range(self.n_arm), weights=self.probs)[0]