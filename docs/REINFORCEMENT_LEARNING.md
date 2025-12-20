```
Goal: Create a learning based packing system that would efficiently pack regular  
objects (rectangular shaped boxes) in a bin with respect to fragility, access  
priority, and stability.  

Process:
Our current flow for the reinforcement learning path (assuming policy.pt is already  
in place) is to use the packing_with_priors from main.py to return a set of coordinates  
that correspond to the "real world" offsets that the box should be moved to. The  
function calls env.reset which builds a feature matrix from the provided (or random  
box dims if not provided) box dimensions. This feature matrix is passed into the  
policy network which produces a permutation of indices. These indices are then passed  
to the place_box_with_rule in heuristics.py which places each box in the backing height  
map by minimizing z then y then x. After the relative coordinates are decided they are  
converted back to real world coordinates by reversing the conversion.  

Explanation of files:
    box.py:
        This stores the core data structures of the boxes and bins as well as bundles  
        which all inherit from BigBox. Bins also have a priority list that dictates  
        which boxes should be placed higher. In this file real world dimensions are  
        taken and converted to dimensions that can be used in the height map.  
        Unfortunately while coding the policy as well as heuristics we used integers  
        and was not really thinking about real-world coordinates so we had to use a  
        conversion once we started working with the actual arm.  

        The main thing to get from this file is the fact that we use a height map  
        backing which is basically a 2d-array where each "cell" stores the total  
        height from the base. This is an example of how the height map looks once we  
        place a 2x1 box, taken from heuristics.py:  
        Bin(2,4,x) height map is initially:  
        [[0, 0, 0, 0],  
        [0, 0, 0, 0]]  

        when we add a Box(2, 1, 3) (from left-top point 0,0) height map becomes:  
        [[3, 3, 0, 0],  
        [0, 0, 0, 0]]  

    config.py:
        We added this as a QOL improvement and source for various constants that are  
        needed in different parts of the program. Mainly this is useful in  
        packing_with_priors to store the bin dimensions and number of boxes rather  
        than passing them around files.  

    env.py:
        This is where we implement a gym style environment for packing. This handles  
        episode initialization with reset and action execution with step. From step  
        we return metrics that are used to score a packing outcome: C (compactness),  
        P (pyramid-like), A (accessibility), F (fragility constraints). The lambdas  
        below just control how strongly accessibility and fragility impact the  
        overall reward.  
            These are used to score according to this equation:
                R = C + P - λa*A - λf*F
        Step is mainly used to evaluate the episode, it produces the scalar reward  
        that reinforces actor loss.

    heuristics.py:
        This file contains our packing heuristic as well as penalty calculations.  
        There is a function to place boxes, a function to find all possible positions  
        for a given box, and reward components. Compactness (C) measures how tightly  
        boxes (total object volume) are packed within the bounding volume (max height  
        that boxes are stacked * bin footprint). Pyramid (P) builds a mask over  
        footprint cells, and returns sum of box volumes / sum of height map over  
        masked cells. The pyramid score rewards based on if there is not a lot of  
        empty vertical space. Access cost (A) uses the priority list as well as how  
        close the box is to the top of the bin, this penalizes based on priority boxes  
        being buried in the bin. Fragility (F) penalizes based on too much weight  
        being on a box relative to its fragility and its size, so if a box is fragile  
        yet its pretty big it might be placed further down because it results in a  
        more stable arrangement. There are a few helpers related to fragility  
        including footprint_overlap (how much 2 given boxes footprints overlap in  
        the height map), vertical_stacking (checks if a box is stacked on another  
        box), weight_on_box (this assumes that weight is proportional to volume,  
        calculates total weight on a particular box).  

    main.py:
        From this file you can run training and placement with random objects, as well  
        as query packing_with_priors. This is also where the pybullet visualization  
        code lives!

    model.py:
        This is where the pointer network policy lives which outputs a permutation of  
        boxes as well as the critic that predicts the expected reward from the input  
        feature matrix. The encoder turns each box feature into a hidden dimension  
        embedding. The decoder constructs a permutation of boxes one element at a  
        time. At each step, it uses a GRU hidden state and an attention mechanism over  
        the encoded box embeddings to produce a probability distribution over  
        remaining boxes, masks out boxes that have already been chosen, and then  
        either samples (during training) or takes the argmax (during evaluation).  
        The chosen box’s embedding is fed back into the GRU so the next choice depends  
        on all previous choices, and the decoder records log-probabilities and  
        entropies for policy-gradient training.

    train.py:
        The functions in this file perform a single actor-critic update by sampling a  
        full packing order from the pointer network, scoring it with heuristic  
        packing in the environment, and reinforcing the policy based on how much  
        better or worse that ordering performed compared to the critic’s prediction.  

    test_box.py and test_packing.py:  
        This was mainly used to test the initial box.py file as well as heuristics  
        and boxes being placed in the height map. Unfortunately I did not continue  
        updating this as the project progressed.

Further work and changes made from paper:
    Changes made:
        There was no code that was provided with the paper so we had to learn how to  
        implement a Pointer Net policy as well as the differences that were made in  
        this paper from a normal Pointer Net policy in the paper. Rather than using  
        supervised learning the paper used reinforcement learning where the reward  
        was based on packing success and space utilization. In our implementation  
        unlike the paper, the decoder state does not have a learned proxy for bin  
        occupancy. Our primary addition was adding fragility penalties and weight  
        aware penalties along with stability and support heuristics. Our primary goal  
        for these additions was to have lighter objects bubble up to the top and have  
        denser objects being placed near the bottom. This means more fragile boxes  
        are packed at the top and bigger/denser objects provide stability by being  
        placed at the bottom.  

    Further Work:
        We would like to implement a combination of offline and online packing that  
        would allow new boxes to be introduced in the middle of packing. This would  
        look like: Packing is started and boxes are being placed -> We introduce a  
        new box -> The arm stops its current plan and replans based on the current  
        height map and the set of boxes including the new box.  

Citation:
B. Wang, Z. Lin, W. Kong and H. Dong, "Bin Packing Optimization via Deep Reinforcement  
Learning," in IEEE Robotics and Automation Letters, vol. 10, no. 3, pp. 2542-2549,  
March 2025, doi: 10.1109/LRA.2025.3534070.  
keywords: {Three-dimensional displays;Genetic algorithms;Costs;Deep reinforcement  
learning;Decoding;Accuracy;Search problems;Logistics;Convolutional neural networks;  
Reinforcement learning;Robot sensing systems;Reinforcement learning;manipulation  
planning;bin packing problem (BPP);robot packing},
```

```
⠀⠀⠀⣀⣀⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⣀⡤⣄⡀⠀
⠶⡿⠟⠛⠛⠛⠛⠛⠿⠷⠶⡶⠦⠀⠀⠀⠻⢶⡶⠿⠿⠟⠛⠛⠛⠛⠷⢿⠲
⠀⠀⠀⢀⡠⢮⣭⣭⣼⣏⡓⢦⠀⠀⠀⠀⢀⡴⢛⣻⣿⣽⣿⡷⠤⣀⠀⠀⠈
⠀⠀⠐⠙⠤⠼⠿⠿⠇⠙⡄⠸⠀⠀⠀⠀⠈⠇⠠⠧⠼⠿⠿⠧⠴⠚⠁⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⠏⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡴⠀⢠⠏⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣀⣠⠴⠋⠀⠀⡞⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠿⠉⠉⠛⠛⠛⠋⠉⠉⠉⠉⠀⠀⠀⠀⠀⣰⡁⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠀⠀⠀⠀⠀
Artwork by Kevin Ying (2025)
```
