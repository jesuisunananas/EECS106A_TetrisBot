import torch # type: ignore

def train_step(policy, critic, env, optimizer_policy, optimizer_critic):

    X = env.reset()                     # (1, N, F)
    selections = policy(X, training=True)

    # selections is list length N, each tensor shape (1,)
    indices = torch.stack(selections, dim=1)  # (1, N)

    # Compute log-probs
    log_probs = []
    B, N, H = 1, len(selections), None

    # Rerun the decoder manually to record log_probs
    enc = policy.encoder(X)
    mask = torch.zeros(1, env.n_objects).bool()
    prev = torch.zeros(1, enc.size(-1))
    h = torch.zeros(1, enc.size(-1))
    
    for t, idx in enumerate(selections):
        h = policy.decoder.gru(prev, h)
        W1e = policy.decoder.W1(enc)
        W2h = policy.decoder.W2(h).unsqueeze(1)
        u = policy.decoder.v(torch.tanh(W1e + W2h)).squeeze(-1)
        u.masked_fill_(mask, -1e9)
        probs = F.softmax(u, dim=-1)
        
        log_probs.append(torch.log(probs[0, idx]))
        
        mask[0, idx] = True
        prev = enc[0, idx]

    log_probs = torch.stack(log_probs).sum()

    # Compute reward
    reward = env.step(indices[0])

    # Critic value
    value = critic(X)  # scalar

    advantage = reward - value

    # Losses
    actor_loss = - advantage * log_probs
    critic_loss = advantage.pow(2)

    # Update
    optimizer_policy.zero_grad()
    actor_loss.backward()
    optimizer_policy.step()

    optimizer_critic.zero_grad()
    critic_loss.backward()
    optimizer_critic.step()

    return reward, actor_loss.item(), critic_loss.item()
