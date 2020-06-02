import random
import numpy as np
from collections import defaultdict
import os
import json
import ast
import yaml
import pickle


def getQ(self, state, action):
        return self.q.get((state, action), 0.0)

def learnQ(self, state, action, reward, value):
        '''
        Q-learning:
            Q(s, a) += alpha * (reward(s,a) + max(Q(s') - Q(s,a))
        '''
        oldv = self.q.get((state, action), None)
        if oldv is None:
            self.q[(state, action)] = reward
        else:
            self.q[(state, action)] = oldv + self.alpha * (value - oldv)

def chooseAction(self, state, return_q=False):
        q = [self.getQ(state, a) for a in self.actions]
        maxQ = max(q)

        if random.random() < self.epsilon:
            minQ = min(q); mag = max(abs(minQ), abs(maxQ))
            # add random values to all the actions, recalculate maxQ
            q = [q[i] + random.random() * mag - .5 * mag for i in range(len(self.actions))]
            maxQ = max(q)

        count = q.count(maxQ)
        # In case there're several state-action max values
        # we select a random one among them
        if count > 1:
            best = [i for i in range(len(self.actions)) if q[i] == maxQ]
            i = random.choice(best)
        else:
            i = q.index(maxQ)

        action = self.actions[i]
        if return_q: # if they want it, give it!
            return action, q
        return action

def learn(self, state1, action1, reward, state2):
        maxqnew = max([self.getQ(state2, a) for a in self.actions])
        self.learnQ(state1, action1, reward, reward + self.gamma*maxqnew)
        # print(self.q)

if __name__ == '__main__':

    SIZE = 10
    outdir = '/home/robil/catkin_ws/src/robil_lihi/training_results'

    if os.path.isfile(outdir + '/data.pickle'):
            q = np.load(outdir + '/data.npy', allow_pickle='TRUE').item()


    else:
        q = {}
        for i in range(-SIZE + 1, SIZE):
            for ii in range(-SIZE + 1, SIZE):
                for iii in range(-SIZE + 1, SIZE):
                    for iiii in range(-SIZE + 1, SIZE):
                        q[((i, ii), (iii, iiii))] = [np.random.uniform(-5, 0) for i in range(4)]

    with open(outdir + "qtable-{int(time.time())}.pickle", "wb") as f:
        pickle.dump(q, f)

    print (q)
