# main.py (example)
from model import PointerNetPolicy, Critic
from env import PackingEnv
from train import train_step
import torch

feature_dim = 7
hidden_dim = 128
n_objects = 6

policy = PointerNetPolicy(feature_dim, hidden_dim)
critic = Critic(feature_dim, hidden_dim)
env = PackingEnv(n_objects, feature_dim)

opt_p = torch.optim.Adam(policy.parameters(), lr=3e-4)
opt_c = torch.optim.Adam(critic.parameters(), lr=3e-4)

for step in range(1000):
    r, la, lc = train_step(policy, critic, env, opt_p, opt_c)
    if step % 50 == 0:
        print(f"step {step}: reward={r:.3f}, actor_loss={la:.3f}, critic_loss={lc:.3f}")
