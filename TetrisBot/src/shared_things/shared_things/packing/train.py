import torch
import torch.nn.functional as F  

def train_step(policy, critic, env,
               optimizer_policy, optimizer_critic,
               entropy_coeff=0.01):
    """
    Single actor-critic update with batch size 1.
    """

    X = env.reset()

    indices, log_probs, entropies = policy(
        X,
        training=True,
        return_log_probs=True,
        return_entropy=True,
    )

    ordering = indices[0]
    reward_scalar = env.step(ordering)
    reward = torch.tensor([reward_scalar], dtype=torch.float32, device=X.device)

    value = critic(X)
    advantage = reward - value
    log_prob_sum = log_probs.sum(dim=1)
    actor_loss = -(advantage.detach() * log_prob_sum).mean()

    if entropies is not None and entropy_coeff is not None and entropy_coeff > 0.0:
        entropy_term = entropies.sum(dim=1).mean()
        actor_loss -= entropy_coeff * entropy_term

    critic_loss = advantage.pow(2).mean()

    optimizer_policy.zero_grad()
    actor_loss.backward()
    optimizer_policy.step()

    optimizer_critic.zero_grad()
    critic_loss.backward()
    optimizer_critic.step()

    return reward_scalar, actor_loss.item(), critic_loss.item()
