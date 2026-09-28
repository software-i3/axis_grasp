```bash
#!/usr/bin/env bash
# Build axis_grasp using the bundled CMake >= 3.14 from tools/
# without permanently modifying the calling shell's PATH.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# build.sh is in:
#   DWE_ws/src/axis_grasp/
# so workspace root is two levels above.
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

case "$(uname -m)" in
    x86_64)
        CMAKE_ROOT="$REPO_ROOT/tools/cmake-3.28.3-linux-x86_64"
        ;;
    aarch64)
        CMAKE_ROOT="$REPO_ROOT/tools/cmake-3.28.3-linux-aarch64"
        ;;
    *)
        echo "ERROR: Unsupported architecture: $(uname -m)" >&2
        exit 1
        ;;
esac

CMAKE_BIN="$CMAKE_ROOT/bin/cmake"

if [[ ! -x "$CMAKE_BIN" ]]; then
    echo "ERROR: Bundled CMake not found or not executable:"
    echo "  $CMAKE_BIN"
    exit 1
fi

export PATH="$CMAKE_ROOT/bin:$PATH"

echo "Workspace root:"
echo "  $REPO_ROOT"
echo

echo "Using CMake:"
echo "  $(command -v cmake)"
cmake --version
echo

cd "$REPO_ROOT"

exec catkin build axis_grasp "$@"
```
