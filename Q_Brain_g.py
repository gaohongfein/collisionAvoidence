import numpy as np
import pandas as pd
import rospy
import matplotlib.pyplot as plt
from pylab import *
from Sa_Env import Sa_Env





class QLearningTable:
    def __init__(self, actions, learning_rate=0.01, reward_decay=0.9, e_greedy=1):
        self.actions = actions  # a list
        self.lr = learning_rate
        self.gamma = reward_decay
        self.epsilon = e_greedy
        self.q_table = pd.DataFrame(columns=self.actions, dtype=np.float64)

    def check_state_exist(self, state):
        if state not in self.q_table.index:
            # append new state to q table
            self.q_table = self.q_table.append(
                pd.Series(
                    [0]*len(self.actions),
                    index=self.q_table.columns,
                    name=state,
                )
            )

    def choose_action(self,state):
        self.check_state_exist(state)
        '''state_action = self.q_table.ix[state, :]
        if (np.random.uniform() > self.epsilon) or (state_action.all() == 0):
            action = np.random.choice(self.actions)
        else:
            action=state_action.argmax()
        return action'''

        q = self.q_table.ix[state, :]
        maxQ = max(q)

        if np.random.random() < self.epsilon:
            minQ = min(q);
            mag = max(abs(minQ), abs(maxQ))
            # add random values to all the actions, recalculate maxQ
            q = [q[i] + np.random.random() * mag - .5 * mag for i in range(len(self.actions))]
            maxQ = max(q)

        count = q.count(maxQ)
        # In case there're several state-action max values
        # we select a random one among them
        if count > 1:
            best = [i for i in range(len(self.actions)) if q[i] == maxQ]
            i = np.random.choice(best)
        else:
            i = q.index(maxQ)

        action = self.actions[i]
        return action



    def learn(self, s, a, r, s_):
        self.check_state_exist(s_)
        q_predict = self.q_table.ix[s, a]
        if s_ != 'terminal':
            q_target = r + self.gamma * self.q_table.ix[s_, :].max()  # next state is not terminal
        else:
            q_target = r  # next state is terminal
        self.q_table.ix[s, a] += self.lr * (q_target - q_predict)  # update







if __name__=="__main__":
    env=Sa_Env()
    RL = QLearningTable(actions=list(range(env.action_n)))

    priode=0
    max_reward=0
    maxreward_plot=[]
    for x in range(100):
        done = False
        state=[6, 6, 6, 6]
        state=env._reset()
        sum_reward = 0
        #print("a:",env.values_list)
        priode += 1
        print("priode", priode)
        #print("max_reward",max_reward)
        for y in range(1000):
            print('state_now:',state)

            action = RL.choose_action(str(state))
            #print(RL.q_table)
            #RL.q_table.to_excel('Qt.xlsx',sheet_name='Sheet1')
            #print("action",action)
            reward, done, state_ = env._step(action)

            #print("einmal reward:",reward,"action:",action)
            sum_reward = sum_reward + reward
            if sum_reward>max_reward:
                max_reward=sum_reward;
            print("sum_reward:", sum_reward, "action:", action)
            print("max_reward", max_reward)
            print("state_",state_)
            RL.learn(str(state), action, reward, str(state_))

            
           
            rospy.sleep(1.)
            if done:
                done = False
                break
            elif not done:
                state = state_

        maxreward_plot.append(sum_reward)

    x=np.arange(0,1000,1)
    y=maxreward_plot
    plt.figure(1)
    plt.plot(x, y)
    plt.show()



