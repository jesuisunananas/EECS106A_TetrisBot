from dataclasses import dataclass

@dataclass
class PackingConfig:
    bin_dims: tuple = (4,4,8)
    feature_dim: int = 7
    hidden_dim: int = 128
    n_objects: int = 20

    lambda_access: float = 0.7
    lambda_fragility: float = 1.0

    base_scaling: float = 1.0
    alpha_capacity: float = 0.9
    heavy_factor: float = 10.0
    fragile_quantile: float = 0.25

    n_steps: int = 2000