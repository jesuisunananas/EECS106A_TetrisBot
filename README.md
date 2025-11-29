# helo

## Install (with CUDA):
```
conda create -n TetrisBot python=3.9 numpy
conda activate TetrisBot
pip install tensorflow[and-cuda]
pip install pytest
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install pybullet
```

### To run tests (from inside packing dir):
```
pytest
```

### To retrain (generates new packing/critic.pt and packing/policy.pt):
```
python packing/main.py --mode train
```

### To run visualization (no training):
```
python packing/main.py --mode eval
```









