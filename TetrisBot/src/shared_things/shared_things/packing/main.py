# main.py

import torch
import os
from collections import deque

# from shared_things import Box, Bin
from .box import Bin, Box
from .heuristics import *
from .model import PointerNetPolicy, Critic
from .env import PackingEnv
from .train import train_step
import random
import pybullet as p
from math import sqrt
import pybullet_data
import time
from .config import PackingConfig
import argparse
import numpy as np

current_path = os.getcwd()


def evaluate_random(env, n_episodes=20):
    rewards = []
    for _ in range(n_episodes):
        X = env.reset()
        indices = list(range(env.n_objects))
        random.shuffle(indices)
        r = env.step(indices)
        rewards.append(r)
    return sum(rewards) / len(rewards)

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('-m', "--mode", choices=["train", "eval"], default="eval")
    return ap.parse_args()


def demo_heuristic():
    """
    Just to sanity-check the heuristic packing logic
    before we even touch RL.
    """
    b = Bin(4, 4, 5, id=7)

    # Some fixed boxes to visualize
    b0 = Box(1, 1, 5, id=1, fragility=0.2, name="b0")
    b1 = Box(1, 2, 2, id=2, fragility=0.5, name="b1")
    b2 = Box(2, 3, 1, id=3, fragility=0.8, name="b2")
    b3 = Box(2, 2, 2, id=4, fragility=0.4, name="b3")

    print("Initial:")
    print(b.height_map)

    for bx in [b0, b1, b2, b3]:
        # placement = heuristics.place_box_with_rule(bx, b)
        placement = place_box_with_rule(bx, b)
        print(f"Placement: {placement}")

    print("After:")
    print(b.height_map)


def evaluate(policy, env, n_episodes=10):
    """
    Run greedy (argmax) policy for a few episodes and return
    average reward. This tells us if the learned policy is improving.
    """
    policy.eval()
    rewards = []
    with torch.no_grad():
        for _ in range(n_episodes):
            X = env.reset()
            # training=False => decoder uses argmax instead of sampling
            indices, _, _ = policy(X, training=False)
            ordering = indices[0]
            r = env.step(ordering)
            rewards.append(r)
    policy.train()
    return sum(rewards) / len(rewards)

def setup_pybullet(gui=True):
    if gui:
        client = p.connect(p.GUI)
    else:
        client = p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    p.resetDebugVisualizerCamera(
        cameraDistance=0.4,
        cameraYaw=45,
        cameraPitch=-45,
        cameraTargetPosition=[0.1, 0.1, 0.1],
    )
    return client

def create_bin_visual(bin_dims, cell_size=0.05):
    L_cells, W_cells, H_cells = bin_dims
    L = L_cells * cell_size
    W = W_cells * cell_size
    H = H_cells * cell_size

    # Floor as a thin box
    # floor_half_extents = [L/2, W/2, 0.005]
    # floor_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=floor_half_extents)
    # floor_visual    = p.createVisualShape(p.GEOM_BOX, halfExtents=floor_half_extents,
    #                                       rgbaColor=[0.8, 0.8, 0.8, 1])
    # p.createMultiBody(
    #     baseMass=0,
    #     baseCollisionShapeIndex=floor_collision,
    #     baseVisualShapeIndex=floor_visual,
    #     basePosition=[L/2, W/2, 0.0]
    # )

    wall_thickness = 0.005
    wall_height = H

    # Four walls: +x, -x, +y, -y
    def wall(half_extents, pos):
        #col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents,
                                  rgbaColor=[0.7, 0.7, 1.0, 0.5])
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=vis,
            basePosition=pos
        )

    # X walls
    wall([wall_thickness, W/2, wall_height/2], [0.0, W/2, wall_height/2])     # at x=0
    wall([wall_thickness, W/2, wall_height/2], [L,   W/2, wall_height/2])     # at x=L

    # Y walls
    wall([L/2, wall_thickness, wall_height/2], [L/2, 0.0, wall_height/2])     # at y=0
    wall([L/2, wall_thickness, wall_height/2], [L/2, W,   wall_height/2])     # at y=W


def compute_ray_from_mouse(mouse_x, mouse_y):
    """
    Given mouse (x, y) in pixels from a PyBullet MOUSE_MOVE event,
    return (ray_from, ray_to) in world coordinates.
    """
    width, height, viewMat, projMat, _, _, _, _, _, _, _, _ = p.getDebugVisualizerCamera()

    # PyBullet returns flat lists; reshape to 4x4 and transpose to match math convention
    viewMat = np.array(viewMat).reshape(4, 4).T
    projMat = np.array(projMat).reshape(4, 4).T

    # Convert to normalized device coordinates [-1, 1]
    x_ndc = (2.0 * mouse_x / width) - 1.0
    y_ndc = 1.0 - (2.0 * mouse_y / height)  # flip Y

    # Points on near and far clip planes in clip space
    near_clip = np.array([x_ndc, y_ndc, -1.0, 1.0])
    far_clip  = np.array([x_ndc, y_ndc,  1.0, 1.0])

    inv = np.linalg.inv(projMat @ viewMat)

    near_world = inv @ near_clip
    far_world  = inv @ far_clip

    near_world /= near_world[3]
    far_world  /= far_world[3]

    ray_from = near_world[:3]
    ray_to   = far_world[:3]

    return ray_from, ray_to


CELL_SIZE = 0.05  # meters per grid cell
#body_to_name = {}

def visualize_bin_pybullet(b: Bin, cell_size=CELL_SIZE, gui=True):
    client = setup_pybullet(gui=gui)
    create_bin_visual(
        bin_dims=(b.length, b.width, b.height),
        cell_size=cell_size,
    )
    def color_from_name(name: str):
        h = abs(hash(name)) % 360
        # quick and dirty HSV->RGB-ish hack: vary R,G,B
        r = ( (h      ) % 255 ) / 255.0
        g = ( (h + 85 ) % 255 ) / 255.0
        b = ( (h + 170) % 255 ) / 255.0
        return [r, g, b, 1]

    body_to_name = {}
    for name, entry in b.boxes.items():
        box = entry["box"]
        x_cell = entry["x"]
        y_cell = entry["y"]
        z_cell = entry["z"]

        # Sizes in meters
        sx = box.length * cell_size
        sy = box.width  * cell_size
        sz = box.height * cell_size

        # Center position in meters
        x_center = (x_cell + box.length / 2.0) * cell_size
        y_center = (y_cell + box.width  / 2.0) * cell_size
        z_center = (z_cell + box.height / 2.0) * cell_size

        half_extents = [sx/2, sy/2, sz/2]

        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=half_extents,
            rgbaColor=color_from_name(name),
        )

        body_id = p.createMultiBody(
            baseMass=0,  # static / kinematic, we already placed them
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[x_center, y_center, z_center],
        )

        body_to_name[body_id] = name
    
    clicked_text_id = None

    while True:
        p.stepSimulation()
        events = p.getMouseEvents()
        for e in events:
            state = e[4]
            if e[0] and state == 3:
                mouse_x = e[1]
                mouse_y = e[2]

                ray_from, ray_to = compute_ray_from_mouse(mouse_x=mouse_x, mouse_y=mouse_y)
                hit = p.rayTest(ray_from, ray_to)[0]
                hit_body_id = hit[0]
                if clicked_text_id is not None:
                    p.removeUserDebugItem(clicked_text_id)
                    clicked_text_id = None
                
                if hit_body_id in body_to_name:
                    name = body_to_name[hit_body_id]
                    entry = b.boxes[name]
                    box = b.boxes[name]["box"]
                    print(f"Clicked box '{name}':")
                    print(f"  dims      = {box.length} x {box.width} x {box.height}")
                    print(f"  fragility = {box.fragility}")
                    print(f"  grid pos  = (x={entry['x']}, y={entry['y']}, z={entry['z']})")
                    text = f"{name} | f={box.fragility:.2f} | {box.length}l x {box.width}w x {box.height}h"
                    hit_pos = hit[3]
                    label_pos = [hit_pos[0], hit_pos[1], hit_pos[2] + 0.02]
                    clicked_text_id = p.addUserDebugText (
                        text = text,
                        textPosition = label_pos,
                        textColorRGB = [0,0,0],
                        textSize = 1,
                    )
                    #hovered_body_id = hit_body_id
        time.sleep(1.0/240.0)

def packing_with_priors(config=PackingConfig(), box_list=None, vis=True):
    feature_dim = config.feature_dim
    hidden_dim = config.hidden_dim
    n_objects = config.n_objects
    policy = PointerNetPolicy(feature_dim=feature_dim, hidden_dim=hidden_dim)
    policy.load_state_dict(torch.load("src/shared_things/shared_things/packing/policy.pt", map_location="cpu"))
    policy.eval()
    env = PackingEnv(n_objects, config)
    X = env.reset(box_list=box_list)
    indices, _, _ = policy(X, training=False)
    ordering = indices[0].tolist()
    print("Ordering (indicies):", ordering)

    b = Bin(*env.bin_dims)
    for idx in ordering:
        box = env.boxes[int(idx)]
        # placed = heuristics.place_box_with_rule(box, b)
        placed = place_box_with_rule(box, b)

        print(f"Placed box {box.id}: {placed}, l: {box.length}, w: {box.width}, h: {box.height}")
        print(b.height_map)

    print("Final height_map:")
    print(b.height_map)

    # C = heuristics.compute_compactness(b)
    # P = heuristics.compute_pyramid(b)
    # A = heuristics.compute_access_cost(b)
    # F = heuristics.compute_fragility_penalty(
    #     b, 
    #     config.base_scaling, 
    #     config.heavy_factor, 
    #     config.fragile_quantile, 
    #     config.alpha_capacity
    # )
    C = compute_compactness(b)
    P = compute_pyramid(b)
    A = compute_access_cost(b)
    F = compute_fragility_penalty(
        b, 
        config.base_scaling, 
        config.heavy_factor, 
        config.fragile_quantile, 
        config.alpha_capacity
    )

    print(f"\nFinal metrics: C={C:.3f}, P={P:.3f}, A={A:.3f}, F={F:.3f}")

    print("\nPer-box placement (sorted by fragility):")
    # build list of (name, fragility, base_z, top_z, x, y)
    box_info = []
    for name, entry in b.boxes.items():
        box = entry["box"]
        z_base = entry["z"]
        z_top = z_base + box.height
        box_info.append((box.id, name, box.fragility, z_base, z_top, entry["x"], entry["y"]))

    # sort fragile → tough (fragility ascending)
    print('Box info presort:')
    print(box_info)
    # box_info.sort(key=lambda t: t[1])
    # print('Box info postsort:')
    # print(box_info)

    for id, name, frag, z_base, z_top, x, y in box_info:
        print(
            f"{name}: frag={frag:.3f}, base_z={z_base}, top_z={z_top}, "
            f"pos=({x}, {y})", f"id={id}"
        )

    if vis:
        visualize_bin_pybullet(b, cell_size=0.05, gui=True)

    return box_info


def main(config: PackingConfig):
    demo_heuristic()
    args = parse_args()

    # Hyperparameters
    feature_dim = config.feature_dim
    hidden_dim = config.hidden_dim
    n_objects = config.n_objects
    lr = 3e-4
    #lr = 1e-4
    entropy_coeff = 0.01

    if args.mode == "train":
        n_steps = config.n_steps

        # Models
        policy = PointerNetPolicy(feature_dim, hidden_dim)
        critic = Critic(feature_dim, hidden_dim)

        # Environment
        env = PackingEnv(n_objects, config)

        # Optimizers
        opt_p = torch.optim.Adam(policy.parameters(), lr=lr)
        opt_c = torch.optim.Adam(critic.parameters(), lr=lr)

        # For a running average of training reward
        avg_window = 100
        reward_hist = deque(maxlen=avg_window)

        print("Starting training...\n")

        for step in range(n_steps):
            r, actor_loss, critic_loss = train_step(
                policy,
                critic,
                env,
                opt_p,
                opt_c,
                entropy_coeff=entropy_coeff,
            )

            reward_hist.append(r)

            if step % 50 == 0:
                avg_r = sum(reward_hist) / len(reward_hist) if reward_hist else 0.0
                print(
                    f"step {step:4d}: "
                    f"reward={r:6.3f}, "
                    f"train_avg={avg_r:6.3f}, "
                    f"actor_loss={actor_loss:7.4f}, "
                    f"critic_loss={critic_loss:7.4f}"
                )

            # Periodic greedy evaluation
            if step % 200 == 0 and step > 0:
                eval_avg = evaluate(policy, env, n_episodes=20)
                print(f"[eval] step {step:4d}: eval_avg_reward={eval_avg:6.3f}")

        # Save final models
        torch.save(policy.state_dict(), "policy.pt")
        torch.save(critic.state_dict(), "critic.pt")
        print("\nTraining finished. Models saved to policy.pt and critic.pt")

        eval_trained = evaluate(policy, env, n_episodes=50)
        eval_random  = evaluate_random(env, n_episodes=50)
        print(f"Trained eval avg: {eval_trained:.3f}")
        print(f"Random eval avg:  {eval_random:.3f}")
        return

    elif args.mode == "eval":
        print("\n=== Greedy rollout from trained policy ===")
        policy = PointerNetPolicy(feature_dim=feature_dim, hidden_dim=hidden_dim)
        policy.load_state_dict(torch.load(os.path.join(current_path, "packing/policy.pt"), map_location="cpu"))
        policy.eval()
        env = PackingEnv(n_objects, config)
        X = env.reset(False)
        indices, _, _ = policy(X, training=False)
        ordering = indices[0].tolist()
        print("Ordering:", ordering)

        b = Bin(*env.bin_dims)
        for idx in ordering:
            box = env.boxes[int(idx)]
            # placed = heuristics.place_box_with_rule(box, b)
            placed = place_box_with_rule(box, b)

            print(f"Placed box {idx}: {placed}, l: {box.length}, w: {box.width}, h: {box.height}")
            print(b.height_map)

        print("Final height_map:")
        print(b.height_map)

        # C = heuristics.compute_compactness(b)
        # P = heuristics.compute_pyramid(b)
        # A = heuristics.compute_access_cost(b)
        # F = heuristics.compute_fragility_penalty(
        #     b, 
        #     config.base_scaling, 
        #     config.heavy_factor, 
        #     config.fragile_quantile, 
        #     config.alpha_capacity
        # )
        C = compute_compactness(b)
        P = compute_pyramid(b)
        A = compute_access_cost(b)
        F = compute_fragility_penalty(
            b, 
            config.base_scaling, 
            config.heavy_factor, 
            config.fragile_quantile, 
            config.alpha_capacity
        )

        print(f"\nFinal metrics: C={C:.3f}, P={P:.3f}, A={A:.3f}, F={F:.3f}")

        print("\nPer-box placement (sorted by fragility):")
        # build list of (name, fragility, base_z, top_z, x, y)
        box_info = []
        for name, entry in b.boxes.items():
            box = entry["box"]
            z_base = entry["z"]
            z_top = z_base + box.height
            box_info.append((name, box.fragility, z_base, z_top, entry["x"], entry["y"]))

        # sort fragile → tough (fragility ascending)
        box_info.sort(key=lambda t: t[1])

        for name, frag, z_base, z_top, x, y in box_info:
            print(
                f"{name}: frag={frag:.3f}, base_z={z_base}, top_z={z_top}, "
                f"pos=({x}, {y})"
            )
        
        visualize_bin_pybullet(b, cell_size=0.05, gui=True)
        return


if __name__ == "__main__":
    main(config=PackingConfig())
