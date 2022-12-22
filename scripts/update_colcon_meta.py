"""
This script looks for all cmake based packages in a workspace and add configs to build
using clang + lld.
"""

import os
import subprocess
import sys

import yaml


def print_ignored(msg: str):
    # gray
    print(f"\033[90m{msg}\033[0m")


def print_ok(msg: str):
    # blue
    print(f"\033[94m{msg}\033[0m")


if len(sys.argv) != 2:
    print("Usage: python3 update_colcon_meta.py <path-to-rmf-workspace>")
    exit(1)

here = os.path.dirname(__file__)
with open(f"{here}/../colcon.meta", "r+") as f:
    colcon_meta = yaml.safe_load(f) or {}
    names: dict = colcon_meta.setdefault("names", {})

    result = subprocess.run(("colcon", "list"), stdout=subprocess.PIPE, check=True)
    for line in result.stdout.decode().splitlines():
        parts = line.split("\t")
        if parts[2] not in ("(ros.ament_cmake)", "(ros.cmake)"):
            print_ignored(f"{parts[0]}: not a cmake package, skipping")
            continue

        pkg = names.setdefault(parts[0], {})
        cmake_args = pkg.setdefault("cmake-args", [])
        if cmake_args:
            print_ignored(f"{parts[0]}: not overwriting existing cmake args")
            continue

        cmake_args = [
            "-DCMAKE_CXX_COMPILER=clang++",
            "-DCMAKE_C_COMPILER=clang",
            "-DCMAKE_EXE_LINKER_FLAGS=-fuse-ld=lld",
            "-DCMAKE_SHARED_LINKER_FLAGS=-fuse-ld=lld",
            "-DCMAKE_MODULE_LINKER_FLAGS=-fuse-ld=lld",
        ]
        pkg["cmake-args"] = cmake_args
        print_ok(f"{parts[0]}: added clang cmake arguments")

    f.seek(0)
    # FIXME(koonpeng): This will nuke all comments previously in the file. https://github.com/yaml/pyyaml/issues/90.
    yaml.safe_dump(colcon_meta, f)
