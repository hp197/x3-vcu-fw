#!/usr/bin/env python3
"""Run an on-target unit-test image through the x3-vcu-emu emulator.

``pio test -e emu-test`` calls this for each suite. By default it fetches the public
emulator (https://github.com/hp197/x3-vcu-emu) into ``.pio/`` and builds a venv with its
dependencies, both cached between runs, so a fresh checkout of this repository can run the
tests with no manual setup.

To use a local checkout / interpreter instead (for example while developing the emulator),
set ``custom_x3emu_dir`` and ``custom_x3emu_python`` in the untracked ``platformio.local.ini``;
``platformio.ini`` passes them here as ``--emu-dir`` / ``--python`` and they are used as is.

Environment: ``X3EMU_REPO`` and ``X3EMU_REF`` override the clone URL and git ref (default
``main``).
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO = os.environ.get("X3EMU_REPO", "https://github.com/hp197/x3-vcu-emu.git")
REF = os.environ.get("X3EMU_REF", "main")
DEPS = ["unicorn==2.1.2", "capstone>=6.0.0a4", "pyelftools>=0.32"]
MIN_PY = (3, 13)  # the emulator's requires-python

PROJECT = Path(__file__).resolve().parent.parent
CACHE = PROJECT / ".pio" / "x3emu"
EMU_DIR = CACHE / "checkout"
VENV_DIR = CACHE / "venv"


def _run(cmd: list[str], **kw: object) -> None:
    """Run a command, raising on failure."""
    subprocess.run(cmd, check=True, **kw)  # type: ignore[arg-type]


def _fetch_emu() -> Path:
    """Clone the emulator (or update an existing clone) and return its directory."""
    if (EMU_DIR / ".git").is_dir():
        _run(["git", "-C", str(EMU_DIR), "fetch", "--quiet", "origin", REF])
        _run(["git", "-C", str(EMU_DIR), "checkout", "--quiet", "FETCH_HEAD"])
    else:
        EMU_DIR.parent.mkdir(parents=True, exist_ok=True)
        _run(["git", "clone", "--quiet", "--depth", "1", "--branch", REF, REPO, str(EMU_DIR)])
    return EMU_DIR


def _find_python() -> str:
    """A ``python`` interpreter new enough for the emulator."""
    if sys.version_info >= MIN_PY:
        return sys.executable
    for name in (f"python{MIN_PY[0]}.{MIN_PY[1]}", "python3.14", "python3"):
        found = shutil.which(name)
        if found:
            out = subprocess.run([found, "-c", "import sys;print(sys.version_info[:2])"], capture_output=True, text=True)
            if out.returncode == 0 and tuple(int(n) for n in out.stdout.strip("()\n").split(",")) >= MIN_PY:
                return found
    raise SystemExit(f"x3emu needs Python >= {MIN_PY[0]}.{MIN_PY[1]}; none found on PATH")


def _venv_python() -> Path:
    """Build (once) a venv with the emulator's dependencies and return its interpreter."""
    py = VENV_DIR / "bin" / "python"
    marker = VENV_DIR / ".deps-ok"
    if not py.exists():
        _run([_find_python(), "-m", "venv", str(VENV_DIR)])
    if not marker.exists():
        _run([str(py), "-m", "pip", "install", "--quiet", "--upgrade", "pip"])
        _run([str(py), "-m", "pip", "install", "--quiet", *DEPS])
        marker.write_text("ok\n")
    return py


def main(argv: list[str] | None = None) -> int:
    """Resolve the emulator + interpreter (local override or cached download) and run the image."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--emu-dir", default="", help="local emulator checkout (else download)")
    parser.add_argument("--python", default="", help="interpreter with the emulator's deps (else a venv)")
    parser.add_argument("image", help="the .elf test image")
    parser.add_argument("emu_args", nargs=argparse.REMAINDER, help="extra args passed to x3emu")
    args = parser.parse_args(argv)

    emu_dir = Path(args.emu_dir).expanduser() if args.emu_dir else _fetch_emu()
    python = Path(args.python).expanduser() if args.python else _venv_python()
    cmd = [str(python), str(emu_dir), "test", "--pio", args.image, *args.emu_args]
    return subprocess.run(cmd).returncode


if __name__ == "__main__":
    sys.exit(main())
