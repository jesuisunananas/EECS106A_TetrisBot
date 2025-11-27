# train.py
import torch
import torch.nn.functional as F  

def train_step(policy, critic, env,
               optimizer_policy, optimizer_critic,
               entropy_coeff=0.01):
    """
    Single actor-critic update with batch size 1.
    """

    # ----- 1. Sample an episode -----
    X = env.reset()  # (1, N, F)

    indices, log_probs, entropies = policy(
        X,
        training=True,
        return_log_probs=True,
        return_entropy=True,
    )  # indices, log_probs, entropies: (1, N, *)

    ordering = indices[0]           # (N,)
    reward_scalar = env.step(ordering)
    reward = torch.tensor([reward_scalar], dtype=torch.float32, device=X.device)  # (1,)

    # ----- 2. Critic -----
    value = critic(X)  # (1,)
    advantage = reward - value  # (1,)

    # ----- 3. Losses -----

    # Actor: - E[ advantage * sum_t log π(a_t | X) ]
    log_prob_sum = log_probs.sum(dim=1)  # (1,)
    actor_loss = -(advantage.detach() * log_prob_sum).mean()

    # Entropy regularization (encourage exploration)
    if entropies is not None and entropy_coeff is not None and entropy_coeff > 0.0:
        entropy_term = entropies.sum(dim=1).mean()
        actor_loss -= entropy_coeff * entropy_term

    # Critic: MSE between reward and value
    critic_loss = advantage.pow(2).mean()

    # ----- 4. Optimize -----
    optimizer_policy.zero_grad()
    actor_loss.backward()
    optimizer_policy.step()

    optimizer_critic.zero_grad()
    critic_loss.backward()
    optimizer_critic.step()

    return reward_scalar, actor_loss.item(), critic_loss.item()
