import torch # pyright: ignore[reportMissingImports]
import torch.nn as nn # type: ignore
import torch.nn.functional as F # type: ignore

class Encoder(nn.Module):
    def __init__(self, feature_dim, hidden_dim):
        super().__init__()
        self.conv = nn.Conv1d (
            in_channels = feature_dim,
            out_channels = hidden_dim,
            kernel_size = 1
        )
    
    def forward(self, x):
        x = x.transpose(1, 2)
        e = self.conv(x)
        e = e.transpose(1, 2)
        return e

class Decoder(nn.Module):
    def __init__(self, hidden_dim):
        super().__init__()
        self.gru = nn.GRUCell(
            input_size = hidden_dim,
            hidden_size = hidden_dim
        )

        self.W1 = nn.Linear(hidden_dim, hidden_dim, bias=False)
        self.W2 = nn.Linear(hidden_dim, hidden_dim, bias=False)
        self.v = nn.Linear(hidden_dim, 1, bias=False)

    def forward(self, encoder_outputs, decode_steps, training=True, return_log_probs=False, return_entropy=False):
        B, N, H = encoder_outputs.shape
        device = encoder_outputs.device
        mask = torch.zeros(B, N, dtype=torch.bool, device=device)
        prev_embed = torch.zeros(B, H, device=device) 
        h_t = torch.zeros(B, H, device=device)
        indices_list = []
        logp_list = []
        entropy_list = []
        batch_indices = torch.arange(B, device=device)
        for t in range(decode_steps):
            h_t = self.gru(prev_embed, h_t)  # (B, H)

            W1e = self.W1(encoder_outputs)         # (B, N, H)
            W2h = self.W2(h_t).unsqueeze(1)        # (B, 1, H)
            u   = self.v(torch.tanh(W1e + W2h)).squeeze(-1)  # (B, N)
            mask_for_logits = mask.clone()
            u = u.masked_fill(mask_for_logits, -1e9)
            probs = F.softmax(u, dim=-1)           # (B, N)

            if training:
                idx = torch.multinomial(probs, 1).squeeze(-1)  # (B,)
            else:
                idx = probs.argmax(dim=-1)                     # (B,)

            indices_list.append(idx)

            if return_log_probs:
                step_logp = torch.log(
                    probs.gather(1, idx.unsqueeze(1)).squeeze(1) + 1e-9
                )  # (B,)
                logp_list.append(step_logp)

            if return_entropy:
                step_entropy = -(probs * (probs + 1e-9).log()).sum(dim=-1)  # (B,)
                entropy_list.append(step_entropy)

            mask[batch_indices, idx] = True
            prev_embed = encoder_outputs[batch_indices, idx]  # (B, H)

        indices = torch.stack(indices_list, dim=1)  # (B, decode_steps)

        if return_log_probs:
            log_probs = torch.stack(logp_list, dim=1)  # (B, decode_steps)
        else:
            log_probs = None

        if return_entropy:
            entropies = torch.stack(entropy_list, dim=1)  # (B, decode_steps)
        else:
            entropies = None

        return indices, log_probs, entropies
        
            
class PointerNetPolicy(nn.Module):
    def __init__(self, feature_dim, hidden_dim):
        super().__init__()
        self.encoder = Encoder(feature_dim, hidden_dim)
        self.decoder = Decoder(hidden_dim)

    def forward(self, x, training=True,
                return_log_probs=False, return_entropy=False):
        enc = self.encoder(x)
        indices, log_probs, entropies = self.decoder(
            enc,
            decode_steps=x.size(1),
            training=training,
            return_log_probs=return_log_probs,
            return_entropy=return_entropy,
        )
        return indices, log_probs, entropies

class Critic(nn.Module):
    def __init__(self, feature_dim, hidden_dim):
        super().__init__()
        self.encoder = Encoder(feature_dim, hidden_dim)
        self.value_head = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )

    def forward(self, x):
        e = self.encoder(x)        # (B, N, H)
        pooled = e.mean(dim=1)     # (B, H)
        return self.value_head(pooled).squeeze(-1)  # (B,)