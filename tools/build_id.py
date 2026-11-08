"""PlatformIO pre-script: stamp the git commit into the build id (CONFIG_BUILD_ID).

config_identity_sync copies CONFIG_BUILD_ID into IN2 +0x1C8..+0x1CD at every boot: the 6-byte
build id the app reads as BLE registers 0xE4..0xE6. It is the first 6 hex digits of the commit
the image is built from, as text (e.g. "99bc2c"). A tree with uncommitted changes gets the same
value as its HEAD.

The id stays unset (6 x 0xFF, config.h's fallback, the erased flash cell 0x08001430..35) when
git is missing, when the project is not a git checkout, or with X3VCU_BUILD_ID=none in the
environment. Use that for builds that must be byte-identical across commits: the build id is
part of the identity page the firmware writes to flash.
"""

import os
import subprocess

Import("env")  # noqa: F821  (SCons global)

BUILD_ID_LEN = 6
UNSET = None  # config.h's fallback: 6 x 0xFF


def build_id():
    """``(short hash or None, origin)``."""
    if os.environ.get("X3VCU_BUILD_ID", "").lower() == "none":
        return UNSET, "unset (X3VCU_BUILD_ID=none)"
    try:
        head = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=env.subst("$PROJECT_DIR"),  # noqa: F821
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return UNSET, "unset (no git commit)"
    return head[:BUILD_ID_LEN], "git " + head[:BUILD_ID_LEN]


SHORT, ORIGIN = build_id()
if SHORT is not None:
    env.Append(CPPDEFINES=[("CONFIG_BUILD_ID", '\\"%s\\"' % SHORT)])  # noqa: F821
print("x3-vcu-fw build id: %s (%s)" % (SHORT if SHORT is not None else "6 x 0xFF", ORIGIN))
