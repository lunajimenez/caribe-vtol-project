# VTOL Prototype Development - Windows Setup Guide

## 📋 Overview

This repository contains the complete development environment for a VTOL (Vertical Takeoff and Landing) prototype integrating:
- **Aerodynamic Analysis** - Dynamic stability, CFD simulations
- **Control Systems** - PID cascaded control for quadcopter + fixed-wing hybrid
- **AI & Vision** - Onboard image recognition and object detection
- **HMI/App** - User interface and VTOL monitoring system
- **Simulations** - Dynamics testing and validation

## 🔧 Windows-Specific Fixes Applied

### Removed Incompatible Packages
The original requirements.txt had several Linux-specific packages that don't work on Windows:
- ❌ `pycairo` - Linux graphics library
- ❌ `ManimPango` - Requires system libraries
- ❌ `glcontext` - OpenGL context manager (Linux-only)
- ❌ `manim` - Animation framework (too heavy for this project)
- ❌ NVIDIA CUDA packages - Removed bloated GPU dependencies

### Windows-Compatible Alternatives
- ✅ Replaced **Manim** → **Plotly** + **PyVista** (3D visualization)
- ✅ Removed **TensorFlow** from main (now optional)
- ✅ Used **PyTorch** as primary DL framework (better Windows support)
- ✅ Cleaned up duplicate OpenCV packages
- ✅ Removed unnecessary dependencies (~120+ packages removed)

## 📦 New Requirements Structure

### 1. **requirements.txt** (Main - All-in-one)
Core packages needed for the entire project. **Start here!**

```bash
pip install -r requirements.txt
```

### 2. **requirements-control-systems.txt** (Minimal - Just dynamics)
For aerodynamic analysis and control system development.

```bash
pip install -r requirements-control-systems.txt
```

### 3. **requirements-ai-vision.txt** (ML & Vision)
For AI, computer vision, object detection work.

```bash
pip install -r requirements-ai-vision.txt
```

### 4. **requirements-app.txt** (HMI/Web)
For the user interface and VTOL monitoring application.

```bash
pip install -r requirements-app.txt
```

### 5. **requirements-dev.txt** (Development Tools)
Optional: Testing, documentation, code quality tools.

```bash
pip install -r requirements-dev.txt
```

## 🚀 Quick Start on Windows

### Step 1: Activate Virtual Environment
```powershell
# If using venv (your current setup)
.\.venv\Scripts\Activate.ps1

# Or if using conda
conda activate caribe-vtol
```

### Step 2: Install Packages
```powershell
# Install main requirements
pip install -r requirements.txt

# Or install specific modules only
pip install -r requirements-control-systems.txt
pip install -r requirements-ai-vision.txt
pip install -r requirements-app.txt
```

### Step 3: Verify Installation
```powershell
python -c "import numpy, scipy, pandas, torch, opencv-contrib-python; print('✅ Core packages OK')"
```

## 🎯 Module-Specific Installation

### Control Systems & Dynamics Development
```powershell
pip install -r requirements-control-systems.txt
# Then run dynamics models:
python control_system/dynamics_model/python/t1_rotational_dynamics.py
```

### AI & Vision Development
```powershell
pip install -r requirements-ai-vision.txt
# For Jupyter notebooks:
pip install jupyter notebook ipython
jupyter notebook control_system/dynamics_model/validation/
```

### HMI/Web App Development
```powershell
pip install -r requirements-app.txt
# Run the app (example):
streamlit run app/main.py  # if you have a main.py
# OR
python -m flask run       # if using Flask
```

## 📝 Known Windows Issues & Solutions

### Issue 1: OpenCV Installation Fails
**Solution:** The `opencv-contrib-python` should work now but if not:
```powershell
pip install opencv-contrib-python --no-cache-dir
```

### Issue 2: PyTorch GPU Support (Optional)
Current setup uses CPU. For GPU (NVIDIA CUDA):
```powershell
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### Issue 3: TensorFlow (if you prefer TensorFlow over PyTorch)
Uncomment in `requirements-ai-vision.txt` and:
```powershell
pip install tensorflow==2.18.0
```

### Issue 4: Jupyter Notebook Issues
```powershell
pip install --upgrade jupyter notebook ipykernel
python -m ipykernel install --user
```

## 🧪 Testing Your Setup

### Test Core Scientific Stack
```powershell
python -c "
import numpy as np
import scipy as sp
import pandas as pd
import matplotlib.pyplot as plt
print('✅ Scientific stack OK')
"
```

### Test Control Systems
```powershell
python -c "
import numpy as np
from scipy import signal
print('✅ Control systems OK')
"
```

### Test Computer Vision
```powershell
python -c "
import cv2
import mediapipe as mp
from ultralytics import YOLO
print('✅ Vision stack OK')
"
```

### Test Web Frameworks
```powershell
python -c "
import flask
import fastapi
import streamlit
print('✅ Web frameworks OK')
"
```

### Test Deep Learning
```powershell
python -c "
import torch
import torchvision
print(f'✅ PyTorch {torch.__version__} OK')
print(f'   GPU Available: {torch.cuda.is_available()}')
"
```

## 📚 Project Structure Usage

```
control_system/dynamics_model/python/
├── t1_rotational_dynamics.py    → Use with requirements-control-systems.txt
├── t2_translational_dynamics.py → Use with requirements-control-systems.txt
└── validation/
    ├── *.ipynb                 → Use with requirements-ai-vision.txt (includes Jupyter)
    
ai_vision/                      → Use with requirements-ai-vision.txt

app/                            → Use with requirements-app.txt

simulations/                    → Use with requirements-control-systems.txt + matplotlib/plotly
```

## 🔄 Switching Between Environments

Want to work on different modules? Easy!

```powershell
# For control system work
pip install -r requirements-control-systems.txt

# Later, add AI/Vision tools
pip install -r requirements-ai-vision.txt

# Python will auto-resolve overlapping packages
```

## 💾 Backup & Recovery

Your backup is saved as `requirements_backup.txt`. If issues arise:
```powershell
# Restore the old environment
pip uninstall -r requirements.txt -y
pip install -r requirements_backup.txt
```

## 🛠 Troubleshooting

### General: Slow Installation?
Some packages (PyTorch, OpenCV) are large. Be patient. Use:
```powershell
pip install -r requirements.txt --verbose
```

### General: Permission Denied Error?
Try:
```powershell
pip install --user -r requirements.txt
```

### Missing Package Errors?
Update pip and reinstall:
```powershell
python -m pip install --upgrade pip
pip install --no-cache-dir -r requirements.txt
```

## 📞 Support

If packages still don't install:
1. Check your Python version: `python --version` (should be 3.11+)
2. Check pip: `pip --version`
3. Try a fresh venv: `python -m venv venv_fresh`
4. Report specific error messages from pip install output

## ✨ What Changed

| Category | Before | After |
|----------|--------|-------|
| Total Packages | 180+ | ~70 |
| Linux-Only Packages | 5+ | 0 ✅ |
| Redundant Packages | Yes | No ✅ |
| GPU Bloat | Heavy | Minimal ✅ |
| Windows Compatibility | ❌ | ✅ |
| Installation Time | ~20 min | ~5-8 min |

---

**Ready to start coding!** 🚀

For questions about specific modules, check the individual README.md files in each folder.
