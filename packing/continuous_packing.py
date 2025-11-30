import torch
from collections import deque

from box import Box, Bin
import heuristics
from model import PointerNetPolicy, Critic
from env import PackingEnv
from train import train_step
import random
import pybullet as p
from math import sqrt
import pybullet_data
import time
from config import PackingConfig
import argparse
import numpy as np

def run_continuous_packing(config: PackingConfig):
    """
    Simulates a conveyor belt scenario:
    1. A single Bin is initialized.
    2. Multiple batches of boxes arrive.
    3. The RL Policy orders the *current* batch.
    4. Heuristics place the current batch *on top* of the previous state.
    """
    feature_dim = config.feature_dim
    hidden_dim = config.hidden_dim
    
    # Load Policy
    print("\n=== Continuous/Sequential Packing Demo ===")
    policy = PointerNetPolicy(feature_dim=feature_dim, hidden_dim=hidden_dim)
    try:
        policy.load_state_dict(torch.load("policy.pt", map_location="cpu"))
        print("Loaded trained policy.")
    except FileNotFoundError:
        print("No 'policy.pt' found. Using random initialized policy.")
    
    policy.eval()

    # 1. Initialize THE Master Bin
    # Note: Using Env logic just to get dimensions, but we manage the Bin manually
    temp_env = PackingEnv(config.n_objects, config)
    master_bin = Bin(*temp_env.bin_dims)
    print(f"Master Bin Initialized: {master_bin.length}x{master_bin.width}x{master_bin.height}")

    # 2. Simulate Batches
    num_batches = 3
    boxes_per_batch = 5

    for batch_id in range(num_batches):
        print(f"\n--- Processing Batch {batch_id+1}/{num_batches} ---")
        
        # A. Generate boxes for this batch
        batch_boxes = generate_random_batch(boxes_per_batch, batch_id)
        
        # B. Convert to Tensor for Policy (Mimics Env.reset() observation)
        # We need to manually construct the observation X that the policy expects.
        # Assuming X is shape (1, seq_len, feature_dim)
        # Features: [length, width, height, fragility, area, volume] (based on typical packing implementations)
        
        obs_list = []
        for box in batch_boxes:
             # Normalize roughly by max bin dim (e.g., 10) for stability, or raw values if Policy expects raw
            obs_list.append([
                box.length, box.width, box.height, box.fragility, 
                box.length * box.width, box.length * box.width * box.height
            ])
        
        # Pad if policy expects fixed input size, or pass dynamic if PointerNet allows.
        # Here we assume dynamic length is fine for PointerNet.
        X = torch.tensor([obs_list], dtype=torch.float32)

        # C. Get Ordering from Policy
        with torch.no_grad():
            indices, _, _ = policy(X, training=False)
            ordering = indices[0].tolist()
        
        print(f"Policy suggested order: {ordering}")

        # D. Place boxes into the Master Bin (State Persists!)
        placed_count = 0
        for idx in ordering:
            box = batch_boxes[int(idx)]
            
            # CRITICAL STEP: We pass the EXISTING master_bin
            # The heuristic will see the non-zero height_map from the previous batch
            placed = heuristics.place_box_with_rule(box, master_bin)
            
            if placed:
                placed_count += 1
                entry = master_bin.boxes[box.name]
                print(f"  Placed {box.name} at z={entry['z']}")
            else:
                print(f"  Failed to place {box.name} (Bin might be full)")

        print(f"Batch {batch_id+1} complete. Height Map Snapshot:")
        print(master_bin.height_map)

    # 3. Final Visualization
    print("\nAll batches processed. Launching visualization...")
    visualize_bin_pybullet(master_bin, cell_size=0.05, gui=True)
