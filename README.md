helo

Steps to run (with CUDA):
```
conda create -n TetrisBot python=3.9 numpy
conda activate TetrisBot
pip install tensorflow[and-cuda]
pip install pytest
```

To run tests:
```
pytest
```