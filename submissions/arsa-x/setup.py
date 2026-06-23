"""Setup script: verifies / downloads MuJoCo Menagerie model dependencies.

On Linux, you may need to install system packages first::

    sudo apt-get install libgl1-mesa-glx libglfw3 libgomp1
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


_HERE = Path(__file__).resolve().parent
_VENDOR = _HERE / "vendor"


def check_system_deps() -> None:
    """Print a reminder for Linux system dependencies if on Linux."""
    if sys.platform.startswith("linux"):
        print("NOTE: On Linux, MuJoCo requires system libraries.")
        print("      If you see 'error while loading shared libraries', run:")
        print("        sudo apt-get install libgl1-mesa-glx libglfw3 libgomp1")
        print()


def check_models() -> bool:
    """Return True if the Menagerie models are present."""
    panda = _VENDOR / "mujoco_menagerie" / "franka_emika_panda" / "panda.xml"
    allegro = _VENDOR / "mujoco_menagerie" / "wonik_allegro" / "right_hand.xml"
    return panda.exists() and allegro.exists()


def download_models() -> None:
    """Clone the MuJoCo Menagerie (sparse checkout for space efficiency)."""
    _VENDOR.mkdir(parents=True, exist_ok=True)

    if check_models():
        print("[OK] MuJoCo Menagerie models already present.")
        return

    print("Downloading MuJoCo Menagerie (sparse checkout)...")
    repo_url = "https://github.com/google-deepmind/mujoco_menagerie.git"
    target = _VENDOR / "mujoco_menagerie"

    try:
        subprocess.run(
            ["git", "clone", "--depth", "1", "--filter=blob:none", "--sparse", repo_url, str(target)],
            check=True, capture_output=True, text=True,
        )
        subprocess.run(
            ["git", "sparse-checkout", "set", "franka_emika_panda", "wonik_allegro"],
            cwd=target, check=True, capture_output=True, text=True,
        )
        print("[OK] Models downloaded successfully.")
    except subprocess.CalledProcessError as e:
        print(f"[FAIL] Failed to download models: {e}")
        print(f"  stderr: {e.stderr}")
        print("\nManually clone the repository:")
        print(f"  git clone --depth 1 {repo_url} {target}")
        sys.exit(1)


def install_deps() -> None:
    """Install Python dependencies."""
    print("Installing Python dependencies...")
    req = _HERE / "requirements.txt"
    if req.exists():
        subprocess.run(
            [sys.executable, "-m", "pip", "install", "-r", str(req)],
            check=True,
        )
    else:
        subprocess.run(
            [sys.executable, "-m", "pip", "install", "mujoco", "numpy", "imageio[ffmpeg]"],
            check=True,
        )
    print("[OK] Dependencies installed.")


def main() -> int:
    print("=" * 50)
    print("  ARSA-X — Setup")
    print("=" * 50)

    check_system_deps()
    install_deps()
    download_models()
    print("\n[OK] Setup complete. Run the demo with:")
    print("  python run.py                   # Interactive teleoperation")
    print("  python run.py --mode autonomous  # Autonomous suture demo")
    print("  python run.py --mode video       # Generate demo video")
    print("  python run.py --mode data-collection  # Record sensor data")
    return 0


if __name__ == "__main__":
    sys.exit(main())
