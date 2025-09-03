from agent import DQNAgent
import numpy as np

# Dummy environment interaction
class DummyEnv:
    def reset(self):
        return np.zeros(6)
    def step(self, action):
        next_state = np.random.randn(6)
        reward = np.random.rand()
        done = False
        return next_state, reward, done, {}

agent = DQNAgent(state_dim=6, action_dim=4)
env = DummyEnv()

for episode in range(100):
    state = env.reset()
    done = False
    while not done:
        action = agent.act(state)
        next_state, reward, done, _ = env.step(action)
        agent.remember((state, action, reward, next_state, done))
        agent.train()
        state = next_state