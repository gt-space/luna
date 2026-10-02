#!/usr/bin/env bash
# Run the SW Onboarding Board SITL tests.
#
#   ./sitl/tests/sw_onboarding_board/run.sh
#
# Expects Renode in /Applications (macOS) and a virtualenv holding Renode's
# test dependencies. Override either with RENODE_ROOT / RENODE_VENV.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
RENODE_ROOT="${RENODE_ROOT:-/Applications/Renode.app/Contents/MacOS}"
RENODE_VENV="${RENODE_VENV:-$HOME/LocalDocuments/tools/renode-venv}"
STM32_BUNDLES="${STM32_BUNDLES:-$HOME/Library/Application Support/stm32cube/bundles}"

FIRMWARE="$REPO_ROOT/onboarding/imu_driver"
ELF="$FIRMWARE/build/Debug/imu_driver.elf"

# 1. Build the firmware under test.
"$FIRMWARE/fetch-hal.sh"

export PATH="$STM32_BUNDLES/gnu-tools-for-stm32/14.3.1+st.2/bin:$STM32_BUNDLES/ninja/1.13.2+st.1/bin:$PATH"
cmake --preset Debug -S "$FIRMWARE" >/dev/null
cmake --build --preset Debug --preset-dir "$FIRMWARE" 2>/dev/null \
  || (cd "$FIRMWARE" && cmake --build --preset Debug)
echo "built $(basename "$ELF")"

# 2. Run the tests. Renode writes its report files into the working directory.
export PATH="$RENODE_VENV/bin:$PATH"
cd "$REPO_ROOT"
"$RENODE_ROOT/renode-test" sitl/tests/sw_onboarding_board/imu_driver.robot
