#!/usr/bin/env python3
"""Checks an installed MRPT wheel: every Python module imports, the C++ SDK is
complete, the per-module Python binding tests of the source tree pass, and a
C++ program builds against the SDK.

Usage: python test_wheel.py   (with the wheel installed in the environment)
"""

import importlib
import pkgutil
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]

# Modules not shipped in the wheel:
NOT_IN_WHEEL = {"hwdrivers"}


def main() -> int:
    import mrpt
    import mrpt.sdk

    mrpt_dirs = [Path(p) for p in mrpt.__path__]
    print("mrpt package dirs:", *mrpt_dirs, sep="\n  ")
    sdk = Path(mrpt.sdk.__file__).parent

    # 1) Every Python module in the wheel imports:
    names = sorted(m.name for m in pkgutil.iter_modules(mrpt.__path__) if m.name != "sdk")
    print(f"Found {len(names)} modules: {' '.join(names)}")
    failed = []
    for name in names:
        try:
            mod = importlib.import_module(f"mrpt.{name}")
            if Path(mod.__file__).parent.parent != sdk.parent:
                failed.append(f"mrpt.{name}: imported from {mod.__file__}, not from the wheel")
        except Exception as e:  # noqa: BLE001
            failed.append(f"mrpt.{name}: {e}")

    # 2) C++ SDK contents:
    print("SDK:", mrpt.sdk.get_cmake_prefix_path(), mrpt.sdk.MRPT_VERSION, mrpt.sdk.PYBIND11_VERSION)
    for f in ["include/mrpt/poses/CPose3D.h", "lib/mrpt_poses/cmake/mrpt_poses-config.cmake",
              "share/mrpt_common/cmake/mrpt_common-config.cmake", "include/nanoflann.hpp",
              "include/eigen3/Eigen/Core", "include/tbb/tbb.h"]:
        if not (sdk / f).exists():
            failed.append(f"SDK file missing: {f}")

    # 3) Per-module binding tests, when run from a source tree:
    tests = sorted(REPO_ROOT.glob("modules/mrpt_*/tests/python_*_test.py"))
    for t in tests:
        module = t.parent.parent.name.removeprefix("mrpt_")
        if module in NOT_IN_WHEEL:
            continue
        r = subprocess.run([sys.executable, str(t)], cwd=t.parent.parent,
                           capture_output=True, text=True)
        status = "OK" if r.returncode == 0 else "FAIL"
        print(f"[{status}] {t.relative_to(REPO_ROOT)}")
        if r.returncode != 0:
            failed.append(f"{t.name}:\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}")

    # 4) A C++ program builds and runs against the SDK (needs cmake):
    if shutil.which("cmake"):
        src = Path(__file__).resolve().parent / "sdk_consumer"
        with tempfile.TemporaryDirectory() as bld:
            steps = [
                ["cmake", "-S", str(src), "-B", bld, "-DCMAKE_BUILD_TYPE=Release",
                 f"-DCMAKE_PREFIX_PATH={mrpt.sdk.get_cmake_prefix_path()}",
                 # Only the SDK, as on a user system: not libraries registered
                 # by the wheel build in ~/.cmake/packages.
                 "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF"],
                ["cmake", "--build", bld],
                [str(Path(bld) / "sdk_consumer")],
            ]
            for cmd in steps:
                r = subprocess.run(cmd, capture_output=True, text=True)
                if r.returncode != 0:
                    failed.append(f"SDK consumer, {' '.join(cmd)}:\n{r.stdout[-3000:]}\n{r.stderr[-3000:]}")
                    break
            else:
                print(f"[OK] SDK consumer: {r.stdout.strip()}")
    else:
        print("[SKIP] SDK consumer: cmake not found")

    if failed:
        print("\nFAILURES:", *failed, sep="\n- ")
        return 1
    print("\nAll checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
