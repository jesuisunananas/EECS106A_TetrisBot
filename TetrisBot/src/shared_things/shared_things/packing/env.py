import torch # type: ignore
import numpy as np # type: ignore
from box import Box, Bin
import heuristics

class RewardScaler:
    def __init__(self, momentum=0.99, eps=1e-6):
        self.momentum = momentum
        self.eps = eps
        self.mean_A = None
        self.mean_F = None

    def update(self, A, F):
        if self.mean_A is None:
            self.mean_A = A
            self.mean_F = F
        else:
            self.mean_A = self.momentum * self.mean_A + (1 - self.momentum) * A
            self.mean_F = self.momentum * self.mean_F + (1 - self.momentum) * F

    def normalize(self, A, F):
        A_norm = A / (self.mean_A + self.eps)
        F_norm = F / (self.mean_F + self.eps)
        return A_norm, F_norm


class PackingEnv:
    def __init__(self, n_objects, config):
        self.n_objects = n_objects
        self.config = config
        self.feature_dim = config.feature_dim
        self.bin_dims = config.bin_dims
        self.scaler = RewardScaler()

    def reset(self):
        # sample random boxes
        self.boxes = []
        for _ in range(self.n_objects):
            L = np.random.randint(1,3)
            W = np.random.randint(1,3)
            H = np.random.randint(1,4)
            frag = np.random.rand()
            self.boxes.append(Box(L, W, H, fragility=frag))

        # Build feature matrix (batch_size=1)
        X = []
        for b in self.boxes:
            X.append([
                b.length, b.width, b.height,
                b.volume,
                b.fragility,
                b.length / self.bin_dims[0],
                b.width  / self.bin_dims[1]
            ])
        
        X = torch.tensor(X, dtype=torch.float32).unsqueeze(0)
        return X  # shape (1, N, F)

    def step(self, ordering):
        config = self.config
        # Create an empty bin
        b = Bin(*self.bin_dims)

        # Apply placement heuristic
        success = True
        for idx in ordering:
            box = self.boxes[int(idx)]
            placed = heuristics.place_box_with_rule(box, b)
            if placed is None:
                success = False
                break
        
        # Compute reward
        if not success:
            return -1.0  # simple fail penalty

        C = heuristics.compute_compactness(b)
        P = heuristics.compute_pyramid(b)
        A = heuristics.compute_access_cost(b)
        F = heuristics.compute_fragility_penalty(
            b, 
            config.base_scaling, 
            config.heavy_factor, 
            config.fragile_quantile, 
            config.alpha_capacity
        )

        self.scaler.update(A, F)
        A_norm, F_norm = self.scaler.normalize(A, F)

        # Combine your metrics
        reward = C + P - self.config.lambda_access*A_norm - self.config.lambda_fragility*F_norm
        #print(f"C={C:.3f}, P={P:.3f}, A={A:.3f}, F={F:.3f}, R={reward:.3f}")
        return reward