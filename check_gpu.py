import torch
import sys

print(f"Python        : {sys.version}")
print(f"PyTorch       : {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")

if torch.cuda.is_available():
    idx = torch.cuda.current_device()
    print(f"GPU index     : {idx}")
    print(f"GPU name      : {torch.cuda.get_device_name(idx)}")
    mem = torch.cuda.get_device_properties(idx).total_memory / 1024**3
    print(f"VRAM total    : {mem:.2f} GB")
    print(f"CUDA version  : {torch.version.cuda}")

    # Quick tensor op on GPU
    x = torch.randn(1000, 1000, device="cuda")
    y = x @ x.T
    print(f"\nTensor op on GPU: OK  (result shape {tuple(y.shape)})")
    print("\n==> RTX 3050 is ready for training.")
else:
    print("\n==> No CUDA GPU detected. Training will fall back to CPU.")
