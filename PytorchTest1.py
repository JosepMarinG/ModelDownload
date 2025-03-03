import torch

if torch.cuda.is_available():
    print("CUDA está disponible. Se usará la GPU.")
else:
    print("CUDA NO está disponible. Se usará la CPU.")
