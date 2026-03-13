#!/usr/bin/env python
"""
VTOL Prototype - Environment Verification Script
Run this to verify all packages are installed correctly on Windows
"""

import sys

def test_imports():
    """Test all critical imports"""
    tests = {
        "Core Scientific": ["numpy", "scipy", "pandas", "sympy", "statsmodels"],
        "Control Systems": ["pyserial", "h5py"],
        "Visualization": ["matplotlib", "plotly", "pyvista", "pyqtgraph"],
        "Computer Vision": ["cv2", "PIL", "skimage"],
        "Deep Learning": ["torch", "torchvision", "torchaudio"],
        "Vision Models": ["mediapipe", "ultralytics"],
        "Web Frameworks": ["flask", "fastapi", "streamlit", "pydantic"],
        "Utilities": ["yaml", "requests", "BeautifulSoup4"],
    }
    
    results = {}
    for category, packages in tests.items():
        results[category] = {}
        for pkg in packages:
            try:
                __import__(pkg)
                results[category][pkg] = "✅"
            except ImportError:
                results[category][pkg] = "❌ Missing"
    
    return results

def print_results(results):
    """Pretty print test results"""
    print("\n" + "="*60)
    print("VTOL PROTOTYPE - ENVIRONMENT VERIFICATION")
    print("="*60 + "\n")
    
    all_good = True
    for category, packages in results.items():
        print(f"\n{category}:")
        print("-" * 40)
        for pkg, status in packages.items():
            print(f"  {pkg:.<30} {status}")
            if "❌" in status:
                all_good = False
    
    print("\n" + "="*60)
    if all_good:
        print("✅ ALL SYSTEMS GO! Your environment is ready.")
        print("\nYou can now:")
        print("  1. Run dynamics simulations: python control_system/dynamics_model/python/t1_*.py")
        print("  2. Test Jupyter notebooks: jupyter notebook control_system/dynamics_model/validation/")
        print("  3. Develop HMI: streamlit run app/<your_app>.py")
        print("  4. Work with vision: python ai_vision/<your_script>.py")
    else:
        print("⚠️  Some packages are missing.")
        print("\nTo fix, run:")
        print("  pip install -r requirements.txt")
        sys.exit(1)
    print("="*60 + "\n")

def print_package_info():
    """Print package versions for key libraries"""
    print("\n📦 INSTALLED PACKAGE VERSIONS:")
    print("-" * 40)
    
    packages = {
        "numpy": "numpy",
        "scipy": "scipy",
        "pandas": "pandas",
        "PyTorch": "torch",
        "OpenCV": "cv2",
        "matplotlib": "matplotlib",
        "plotly": "plotly",
    }
    
    for name, import_name in packages.items():
        try:
            mod = __import__(import_name)
            version = getattr(mod, "__version__", "Unknown")
            print(f"  {name:.<20} {version}")
        except ImportError:
            print(f"  {name:.<20} NOT INSTALLED")

if __name__ == "__main__":
    results = test_imports()
    print_results(results)
    print_package_info()
