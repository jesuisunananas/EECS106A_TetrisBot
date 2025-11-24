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

    def forward(self, encoder_outputs, decode_steps, training=True):
        B, N, H = encoder_outputs.shape
        device = encoder_outputs.device
        mask = torch.zeros(B, N, dtype=torch.bool, device=device)
        prev_embed = torch.zeros(B, H, device=device)
        h_t = torch.zeros(B, H, device=device)
        selections = []
        for t in range(decode_steps):
            h_t = self.gru(prev_embed, h_t)
            W1e = self.W1(encoder_outputs)
            W2h = self.W2(h_t).unsqueeze(1)
            u = self.v(torch.tanh(W1e + W2h)).squeeze(-1)
            u.masked_fill_(mask, -1e9)
            probs = F.softmax(u, dim=-1)
            if training:
                idx = torch.multinomial(probs, 1).squeeze(-1)
            else:
                idx = probs.argmax(dim=-1)
            selections.append(idx)
            mask[torch.arange(B), idx] = True
            prev_embed = encoder_outputs[torch.arange(B), idx]

        return selections
            
class PointerNetPolicy(nn.Module):
    def __init__(self, feature_dim, hidden_dim):
        super().__init__()
        self.encoder = Encoder(feature_dim, hidden_dim)
        self.decoder = Decoder(hidden_dim)

    def forward(self, x, training=True):
        B, N, F = x.shape
        enc = self.encoder(x)
        selections = self.decoder(enc, N, training = training)
        return selections

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