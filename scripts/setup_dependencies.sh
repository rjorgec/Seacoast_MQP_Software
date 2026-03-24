#!/usr/bin/env bash
# =============================================================================
# setup_dependencies.sh
# =============================================================================
# Sets up everything needed to build, flash, and serial-monitor firmware for
# the Raspberry Pi Pico 2 (RP2350) and ESP32-C6 on a fresh Debian/Ubuntu/Mint
# installation of VS Code.
#
# Designed for: Debian 12 (Bookworm), Ubuntu 22.04/24.04 LTS, or
#               Linux Mint 21.x / 22.x (which is based on Ubuntu LTS)
# Must NOT be run as root — the script will call sudo where required and
# prompt you for your password as needed.
#
# Usage (run from the repository root):
#   bash scripts/setup_dependencies.sh [IDF_PATH]
#
#   IDF_PATH  — optional: override the ESP-IDF installation directory.
#               If omitted, defaults to ~/.espressif/5.5.2/esp-idf
#
# What this script installs / configures:
#   1. System packages   — build tools, ARM cross-compiler, USB libraries,
#                          serial terminal (picocom), Python, Git, CMake, etc.
#   2. Git submodules    — initialises extern/pico-scale inside this repo
#   3. Pico SDK 2.2.0    — cloned to ~/.pico-sdk/sdk/2.2.0 with submodules
#   4. picotool 2.2.0-a4 — built from source; installed to the path that the
#                          VS Code tasks expect for one-click flashing
#   5. ESP-IDF v5.5.2    — cloned to ~/.espressif/5.5.2/esp-idf (only the
#                          esp32c6 toolchain is downloaded to save space)
#   6. udev rules        — lets you flash/monitor Pico and ESP32-C6 without
#                          sudo once you reconnect your USB cable
#   7. Group membership  — adds you to 'dialout' and 'plugdev' so serial ports
#                          and USB devices are accessible without sudo
#   8. VS Code settings  — creates .vscode/settings.json files with correct
#                          SDK paths (safe to customise; they are git-ignored)
#   9. Pico cmake fix    — writes ~/.pico-sdk/cmake/pico-vscode.cmake so the
#                          build always uses the SDK installed here, even when
#                          a different Pico SDK is already present on the system
# =============================================================================

set -euo pipefail

# -----------------------------------------------------------------------------
# Script constants — change these if you need a different version
# -----------------------------------------------------------------------------

PICO_SDK_VERSION="2.2.0"
PICO_SDK_ROOT="${HOME}/.pico-sdk/sdk/${PICO_SDK_VERSION}"

PICOTOOL_VERSION="2.2.0-a4"
# This exact path is what the VS Code tasks expect; do not change without also
# updating .vscode/tasks.json PICO: Flash task.
PICOTOOL_INSTALL_DIR="${HOME}/.pico-sdk/picotool/${PICOTOOL_VERSION}/picotool"

IDF_VERSION="v5.5.2"
# Default ESP-IDF location — can be overridden by the first script argument or
# by 'idf.currentSetup' in .vscode/settings.json.
# ${IDF_VERSION#v} strips the leading "v" (e.g. "v5.5.2" → "5.5.2") so the
# directory name matches the Espressif convention of plain version numbers.
IDF_PATH="${HOME}/.espressif/${IDF_VERSION#v}/esp-idf"

# -----------------------------------------------------------------------------
# Resolve optional IDF_PATH override
# -----------------------------------------------------------------------------

# Allow the caller to pass a custom IDF path as the first argument.
if [ -n "${1:-}" ]; then
  IDF_PATH="$1"
fi

# If there is already a .vscode/settings.json with idf.currentSetup set, use
# that path so re-running the script respects an existing install.
if [ -f ".vscode/settings.json" ] && command -v python3 >/dev/null 2>&1; then
  _idf=$(python3 -c \
    'import json,sys; d=json.load(open(".vscode/settings.json")); print(d.get("idf.currentSetup",""))' \
    2>/dev/null || true)
  if [ -n "$_idf" ]; then
    IDF_PATH="$_idf"
  fi
fi

# -----------------------------------------------------------------------------
# Guard: must be run on a Debian/Ubuntu/Mint system with apt-get
# -----------------------------------------------------------------------------

if ! command -v apt-get >/dev/null 2>&1; then
  echo "ERROR: This script targets Debian/Ubuntu/Mint Linux and requires apt-get." >&2
  echo "       Please install the required tools manually on your distribution." >&2
  exit 1
fi

# Guard: must NOT be run as root (we will call sudo ourselves)
if [ "$EUID" -eq 0 ]; then
  echo "ERROR: Do not run this script as root. Run it as your normal user." >&2
  echo "       The script will call sudo when elevated privileges are needed." >&2
  exit 1
fi

# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------

# Print a clearly visible section header so progress is easy to follow.
section() {
  echo ""
  echo "================================================================"
  echo "  $*"
  echo "================================================================"
}

# Install one or more Debian/Ubuntu/Mint packages, skipping any that are already present.
apt_install() {
  local to_install=()
  for pkg in "$@"; do
    # dpkg-query exits non-zero when the package is not installed via apt/dpkg.
    # Note: packages installed by other means (e.g. snap, pip, manually compiled
    # binaries) will NOT appear in dpkg-query — they will be queued for apt
    # install, which is harmless since apt-get install is idempotent and will
    # simply report "already up to date" if the package is already satisfied.
    if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
      to_install+=("$pkg")
    fi
  done

  if [ ${#to_install[@]} -eq 0 ]; then
    echo "  (all packages already installed via apt — skipping apt-get)"
    return
  fi

  echo "  Installing: ${to_install[*]}"

  # Run apt-get update to refresh the package index.
  # We use '|| true' here so that a broken third-party repository (e.g. a
  # stale AMD GPU, CUDA, or other vendor/Mint repo that no longer has a Release
  # file) does not abort the entire setup script.  If the update itself fails,
  # apt-get install will still work from the cached package lists for all
  # standard Debian/Ubuntu/Mint repositories.
  sudo apt-get update -y -qq 2>&1 || {
    echo "  WARNING: 'apt-get update' reported errors (possibly a broken third-party"
    echo "  repository in /etc/apt/sources.list.d/).  Proceeding with cached package"
    echo "  lists — run 'sudo apt-get update' manually to investigate the error."
  }

  sudo apt-get install -y "${to_install[@]}"
}

# Clone a git repository only if the destination directory does not yet exist.
# Usage: git_clone_once <tag/branch> <url> <destination>
git_clone_once() {
  local ref="$1" url="$2" dest="$3"
  if [ -d "$dest" ]; then
    echo "  Already present: $dest — skipping clone."
    return
  fi
  echo "  Cloning $url @ $ref → $dest"
  mkdir -p "$(dirname "$dest")"
  git clone -b "$ref" --depth 1 "$url" "$dest"
}

# =============================================================================
# STEP 1 — System packages
# =============================================================================
# Install everything a fresh Debian/Ubuntu/Mint system needs to compile Pico and ESP32
# firmware, flash devices, and open a serial monitor.
# =============================================================================

section "STEP 1 — Installing system packages (sudo required)"

# Each package is listed with a comment explaining why it is needed.
# All packages are passed to apt_install as an array to avoid word-splitting.
SYS_PACKAGES=(
  build-essential      # gcc, g++, make  — C/C++ host compiler and linker
  cmake                # build system used by both the Pico SDK and ESP-IDF
  ninja-build          # fast build backend preferred by Pico SDK and ESP-IDF
  git                  # version control and submodule management
  python3              # required by ESP-IDF install scripts and tooling
  python3-pip          # pip — Python package installer
  python3-venv         # virtual-environment support needed by ESP-IDF
  wget                 # used by ESP-IDF install.sh to download toolchain archives
  curl                 # fallback download tool
  pkg-config           # locates libusb headers when building picotool
  libusb-1.0-0-dev     # USB library needed to build and run picotool
  gcc-arm-none-eabi    # ARM bare-metal cross-compiler for Pico firmware
  libnewlib-arm-none-eabi         # newlib C runtime for ARM bare-metal targets
  libstdc++-arm-none-eabi-newlib  # C++ standard library for ARM bare-metal
  dfu-util             # USB DFU flashing (ESP32-C6 DFU mode fallback)
  picocom              # lightweight serial terminal for Pico USB CDC stdio
  usbutils             # provides lsusb for diagnosing USB device enumeration
)

apt_install "${SYS_PACKAGES[@]}"

echo "  System packages OK."

# =============================================================================
# STEP 2 — Git submodules (pico-scale HX711 library)
# =============================================================================

section "STEP 2 — Initialising git submodules"

# The pico-scale submodule lives at extern/pico-scale and provides the HX711
# load-cell driver used by the Pico firmware.  Running this after cloning
# ensures the extern directory is populated before cmake tries to add it.
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
echo "  Repository root: $REPO_ROOT"

if [ -f "$REPO_ROOT/.gitmodules" ]; then
  git -C "$REPO_ROOT" submodule update --init --recursive
  echo "  Submodules initialised."
else
  echo "  No .gitmodules file found — skipping."
fi

# =============================================================================
# STEP 3 — Raspberry Pi Pico SDK 2.2.0
# =============================================================================

section "STEP 3 — Pico SDK ${PICO_SDK_VERSION}"

git_clone_once "${PICO_SDK_VERSION}" \
  https://github.com/raspberrypi/pico-sdk.git \
  "$PICO_SDK_ROOT"

# The Pico SDK has its own submodules (tinyusb, mbedtls, etc.); without these
# the USB stack and TLS libraries will not compile.
if [ -d "$PICO_SDK_ROOT" ]; then
  echo "  Updating Pico SDK submodules..."
  git -C "$PICO_SDK_ROOT" submodule update --init --recursive
fi

echo "  Pico SDK OK at: $PICO_SDK_ROOT"

# =============================================================================
# STEP 4 — picotool ${PICOTOOL_VERSION}
# =============================================================================
# picotool is used by the VS Code 'PICO: Flash' task to load a .uf2 file onto
# the Pico 2 via USB without manually entering BOOTSEL mode.
# We build it from source because the binary in the Pico VS Code extension
# may not be present on a fresh system.
# =============================================================================

section "STEP 4 — picotool ${PICOTOOL_VERSION}"

PICOTOOL_BIN="${PICOTOOL_INSTALL_DIR}/picotool"

if [ -x "$PICOTOOL_BIN" ]; then
  echo "  picotool already installed at: $PICOTOOL_BIN — skipping build."
else
  echo "  Building picotool from source (requires Pico SDK at $PICO_SDK_ROOT)..."

  PICOTOOL_SRC_DIR="/tmp/picotool-src-${PICOTOOL_VERSION}"
  PICOTOOL_BUILD_DIR="/tmp/picotool-build-${PICOTOOL_VERSION}"

  git_clone_once "${PICOTOOL_VERSION}" \
    https://github.com/raspberrypi/picotool.git \
    "$PICOTOOL_SRC_DIR"

  cmake -S "$PICOTOOL_SRC_DIR" -B "$PICOTOOL_BUILD_DIR" \
    -DPICO_SDK_PATH="$PICO_SDK_ROOT" \
    -DCMAKE_BUILD_TYPE=Release \
    -G Ninja

  cmake --build "$PICOTOOL_BUILD_DIR" --parallel

  mkdir -p "$PICOTOOL_INSTALL_DIR"
  cp "$PICOTOOL_BUILD_DIR/picotool" "$PICOTOOL_BIN"
  echo "  picotool installed at: $PICOTOOL_BIN"
fi

# =============================================================================
# STEP 5 — ESP-IDF ${IDF_VERSION} (ESP32-C6 target only)
# =============================================================================
# Only the esp32c6 toolchain is downloaded.  Downloading all targets would
# consume several extra gigabytes and is not needed for this project.
# =============================================================================

section "STEP 5 — ESP-IDF ${IDF_VERSION} for esp32c6"

git_clone_once "${IDF_VERSION}" \
  https://github.com/espressif/esp-idf.git \
  "$IDF_PATH"

# install.sh downloads the Xtensa/RISC-V toolchains and OpenOCD into
# ~/.espressif/tools/.  Passing 'esp32c6' downloads only what is needed.
#
# Installation check: we look for two stable markers —
#   1. The IDF Python virtual-environment directory created by install.sh.
#      Pattern: ~/.espressif/python_env/idf<IDF_VER>_py<PY_VER>_env/
#      (Espressif has kept this naming scheme since IDF v4.x.)
#   2. The RISC-V GCC toolchain binary for esp32c6 targets.
#      If either is missing we re-run install.sh to be safe.
_idf_py_env_ok=false
_riscv_gcc_ok=false
compgen -G "${HOME}/.espressif/python_env/idf*_py*_env/bin/python" > /dev/null 2>&1 \
  && _idf_py_env_ok=true
ls "${HOME}/.espressif/tools/riscv32-esp-elf/"*/bin/riscv32-esp-elf-gcc > /dev/null 2>&1 \
  && _riscv_gcc_ok=true

if ! $_idf_py_env_ok || ! $_riscv_gcc_ok; then
  echo "  ESP-IDF tools not fully installed — running install.sh esp32c6..."
  # Pass the target name so the install script fetches only esp32c6 binaries.
  "$IDF_PATH/install.sh" esp32c6
else
  echo "  ESP-IDF Python environment and RISC-V toolchain already present — skipping install.sh."
fi

echo "  ESP-IDF OK at: $IDF_PATH"

# =============================================================================
# STEP 5b — Export IDF_PATH to the system environment
# =============================================================================
# ESP-IDF tools (idf.py, esptool.py, etc.) require the IDF_PATH environment
# variable to locate the framework.  Without a persistent export the variable
# is only set inside this script's process and disappears when the script exits.
#
# Two complementary locations are written:
#   /etc/profile.d/99-esp-idf.sh  — picked up by login shells for ALL users
#                                    (covers VS Code integrated terminal on most
#                                    desktop environments, SSH sessions, etc.)
#   ~/.bashrc                      — picked up by interactive non-login shells
#                                    (the typical case when you open a terminal
#                                    emulator that does not invoke a login shell)
# =============================================================================

section "STEP 5b — Exporting IDF_PATH to the system environment (sudo required)"

# --- /etc/profile.d/ (system-wide, login shells) ----------------------------

PROFILE_D_FILE="/etc/profile.d/99-esp-idf.sh"

# Build the file content as a variable so we can compare it to what is already
# on disk and skip the sudo write when nothing has changed.
PROFILE_D_CONTENT="# ESP-IDF environment — written by setup_dependencies.sh
# Re-run setup_dependencies.sh to update after an IDF version change.
export IDF_PATH=\"${IDF_PATH}\""

if [ -f "$PROFILE_D_FILE" ] && [ "$(cat "$PROFILE_D_FILE")" = "$PROFILE_D_CONTENT" ]; then
  echo "  $PROFILE_D_FILE already up-to-date — skipping."
else
  echo "  Writing $PROFILE_D_FILE (sudo required)..."
  printf '%s\n' "$PROFILE_D_CONTENT" | sudo tee "$PROFILE_D_FILE" > /dev/null
  sudo chmod 644 "$PROFILE_D_FILE"
  echo "  System-wide IDF_PATH → $IDF_PATH"
fi

# --- ~/.bashrc (current user, interactive non-login shells) ------------------

BASHRC="${HOME}/.bashrc"
BASHRC_MARKER="# ESP-IDF — added by setup_dependencies.sh"

if grep -qF "$BASHRC_MARKER" "$BASHRC" 2>/dev/null; then
  echo "  $BASHRC already contains IDF_PATH export — skipping."
else
  {
    echo ""
    echo "$BASHRC_MARKER"
    echo "export IDF_PATH=\"${IDF_PATH}\""
    # Sourcing export.sh adds idf.py and the Xtensa/RISC-V toolchain binaries
    # to PATH.  The '2>/dev/null || true' guard prevents errors from breaking
    # a shell session if the IDF directory is temporarily unavailable (e.g. a
    # network mount that is not yet mounted, or before a first-time install).
    echo ". \"\$IDF_PATH/export.sh\" 2>/dev/null || true"
  } >> "$BASHRC"
  echo "  Added IDF_PATH export + export.sh source to $BASHRC"
fi

echo "  IDF_PATH environment export OK."

# =============================================================================
# STEP 6 — udev rules for USB device access (sudo required)
# =============================================================================
# Without these rules you would need to run picotool / idf.py flash as root.
# After installing the rules and reconnecting the USB cable, flashing and
# monitoring will work as a normal user.
# =============================================================================

section "STEP 6 — Installing udev rules (sudo required)"

UDEV_RULES_FILE="/etc/udev/rules.d/99-seacoast-dev.rules"

# Write the file only if it does not already exist or its content has changed.
UDEV_CONTENT=$(cat <<'UDEV_EOF'
# =============================================================================
# Seacoast MQP — USB udev rules
# Allows normal users in the 'dialout' and 'plugdev' groups to access target
# devices for flashing and serial monitoring without running sudo.
#
# After installing these rules run:
#   sudo udevadm control --reload-rules && sudo udevadm trigger
# and reconnect your USB cable.
# =============================================================================

# --- Raspberry Pi Pico / RP2350 in BOOTSEL (USB mass-storage) mode ----------
# VID 2E8A is Raspberry Pi; product IDs vary by board revision.
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0003", \
  MODE="0660", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000f", \
  MODE="0660", GROUP="plugdev", TAG+="uaccess"

# --- Raspberry Pi Pico USB CDC serial (stdio / debug output) -----------------
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", \
  MODE="0660", GROUP="dialout", SYMLINK+="ttyPico"

# --- ESP32-C6 built-in USB Serial/JTAG interface -----------------------------
# VID 303A is Espressif; PID 1001 is the USB CDC+JTAG combo interface.
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", \
  MODE="0660", GROUP="dialout", SYMLINK+="ttyESP32"
SUBSYSTEM=="usb", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", \
  MODE="0660", GROUP="plugdev", TAG+="uaccess"

# --- CP2102 / CP2102N USB-to-UART bridge (common devkit adapter) -------------
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
  MODE="0660", GROUP="dialout"

# --- CH340 / CH341 USB-to-UART bridge (common devkit adapter) ----------------
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  MODE="0660", GROUP="dialout"
UDEV_EOF
)

if [ -f "$UDEV_RULES_FILE" ] && [ "$(cat "$UDEV_RULES_FILE")" = "$UDEV_CONTENT" ]; then
  echo "  udev rules already up-to-date: $UDEV_RULES_FILE"
else
  echo "  Writing udev rules to $UDEV_RULES_FILE"
  echo "$UDEV_CONTENT" | sudo tee "$UDEV_RULES_FILE" > /dev/null
  echo "  Reloading udev rules..."
  sudo udevadm control --reload-rules
  sudo udevadm trigger
  echo "  udev rules installed."
fi

# =============================================================================
# STEP 7 — Add current user to 'dialout' and 'plugdev' groups
# =============================================================================
# 'dialout' provides access to /dev/ttyACM* and /dev/ttyUSB* serial ports.
# 'plugdev' provides access to USB mass-storage and JTAG devices.
# The new membership only takes effect after you log out and log back in.
# =============================================================================

section "STEP 7 — Group membership (dialout, plugdev)"

GROUPS_CHANGED=false

for grp in dialout plugdev; do
  if id -nG "$USER" | grep -qw "$grp"; then
    echo "  Already a member of: $grp"
  else
    echo "  Adding $USER to group: $grp (sudo required)"
    sudo usermod -aG "$grp" "$USER"
    GROUPS_CHANGED=true
  fi
done

if [ "$GROUPS_CHANGED" = true ]; then
  echo ""
  echo "  *** IMPORTANT: You were added to one or more groups. ***"
  echo "  *** You MUST log out and log back in for the change  ***"
  echo "  *** to take effect before flashing or monitoring.    ***"
fi

# =============================================================================
# STEP 8 — VS Code settings.json files
# =============================================================================
# These files are git-ignored (.vscode/settings.json is not tracked) so it is
# safe to create them here.  If a file already exists it is left untouched so
# that any local customisations are preserved.
#
# The root settings cover the Pico/CMake workspace.
# The lcd_uext_ili9341 sub-folder settings cover the ESP32 IDF workspace.
# =============================================================================

section "STEP 8 — Creating VS Code settings (if not already present)"

# Helper: write $2 to $1 only if $1 does not already exist.
create_vscode_settings() {
  local path="$1"
  local content="$2"

  if [ -f "$path" ]; then
    echo "  Already exists — skipping: $path"
    return
  fi

  mkdir -p "$(dirname "$path")"
  printf '%s\n' "$content" > "$path"
  echo "  Created: $path"
}

# Root workspace settings.
# cmake.configureSettings passes PICO_SDK_PATH to cmake so the Pico firmware
# can be configured without the VS Code Pico extension having downloaded the
# SDK separately.
create_vscode_settings "${REPO_ROOT}/.vscode/settings.json" \
"$(cat <<SETTINGS_EOF
{
  // Path to the ESP-IDF installation used by the ESP-IDF VS Code extension
  // and by the ESP: Build / Flash / Monitor tasks.
  "idf.currentSetup": "${IDF_PATH}",

  // Default serial port for idf.py flash / monitor.
  // Change this to match the /dev/tty* device shown by: ls /dev/ttyACM* /dev/ttyUSB*
  "idf.port": "/dev/ttyACM0",

  // Tell the CMake Tools extension where the Pico firmware CMakeLists.txt is.
  "cmake.sourceDirectory": "\${workspaceFolder}/pico_fw/src",

  // Pass the Pico SDK location to cmake so builds work without the Pico
  // VS Code extension having to pre-install the SDK.
  "cmake.configureSettings": {
    "PICO_SDK_PATH": "${PICO_SDK_ROOT}",
    "PICO_BOARD": "pico2"
  }
}
SETTINGS_EOF
)"

# ESP32 sub-project settings (lcd_uext_ili9341 folder).
# These settings are read when you open the lcd_uext_ili9341 folder as a
# workspace in VS Code, or by the Espressif ESP-IDF extension.
create_vscode_settings "${REPO_ROOT}/lcd_uext_ili9341/.vscode/settings.json" \
"$(cat <<SETTINGS_EOF
{
  // Path to the ESP-IDF installation.
  "idf.currentSetup": "${IDF_PATH}",

  // OpenOCD configuration for the ESP32-C6 built-in USB JTAG interface.
  "idf.openOcdConfigs": [
    "board/esp32c6-builtin.cfg"
  ],

  // Lock the IDF target to esp32c6 so 'idf.py set-target' is not needed.
  "idf.customExtraVars": {
    "IDF_TARGET": "esp32c6"
  },

  // Serial port for idf.py flash / monitor.
  // Change to match your device: ls /dev/ttyACM* /dev/ttyUSB*
  // After udev rules are installed, the ESP32-C6 should also appear as /dev/ttyESP32
  "idf.port": "/dev/ttyACM0",

  // Use UART flashing (no OpenOCD / JTAG needed for basic flash+monitor).
  "idf.flashType": "UART"
}
SETTINGS_EOF
)"

# =============================================================================
# STEP 9 — Pico VS Code cmake integration (pico-vscode.cmake)
# =============================================================================
# The project CMakeLists.txt conditionally includes:
#   ~/.pico-sdk/cmake/pico-vscode.cmake
# before pico_sdk_import.cmake.  The Pico VS Code extension writes this file
# and typically sets PICO_SDK_PATH with CACHE … FORCE, which can override the
# -DPICO_SDK_PATH argument we pass from the VS Code task.
#
# If a user has a pre-existing Pico SDK at a different location (e.g.
# ~/pico/pico-sdk from pico-setup or a manually installed copy) the stale
# pico-vscode.cmake will point cmake at the wrong SDK, producing the error:
#   "source directory …/pico-sdk/src/rp2350/pico_platform is not a
#    subdirectory of ~/.pico-sdk/sdk/2.2.0/src"
#
# We always overwrite the file here so it references the SDK version we just
# installed, regardless of any previous installation.
#
# We also delete any stale cmake cache in pico_fw/src/build/.  A cache that
# was created with a different PICO_SDK_PATH can cause the same collision even
# after pico-vscode.cmake is corrected.
# =============================================================================

section "STEP 9 — Fixing Pico cmake integration (pico-vscode.cmake)"

PICO_VSCODE_CMAKE="${HOME}/.pico-sdk/cmake/pico-vscode.cmake"
mkdir -p "$(dirname "$PICO_VSCODE_CMAKE")"

cat > "$PICO_VSCODE_CMAKE" <<VSCODE_CMAKE_EOF
# Generated by setup_dependencies.sh — do not edit manually.
# Re-run setup_dependencies.sh to regenerate after an SDK version change.
#
# Forces the Pico cmake build to use the SDK installed by setup_dependencies.sh,
# overriding any pre-existing PICO_SDK_PATH (e.g. from ~/pico/pico-sdk set up
# by pico-setup or the Pico VS Code extension configured with a different SDK).
set(PICO_SDK_PATH "${PICO_SDK_ROOT}" CACHE PATH "" FORCE)
VSCODE_CMAKE_EOF

echo "  Written: $PICO_VSCODE_CMAKE"
echo "    PICO_SDK_PATH → ${PICO_SDK_ROOT}"

# Clear any stale Pico cmake cache.  A cache created with a different
# PICO_SDK_PATH will keep the wrong path even after pico-vscode.cmake is fixed.
# Deleting it forces a clean configure on the next PICO: Build / Configure run.
PICO_BUILD_DIR="${REPO_ROOT}/pico_fw/src/build"
if [ -f "${PICO_BUILD_DIR}/CMakeCache.txt" ]; then
  echo "  Removing stale cmake cache: ${PICO_BUILD_DIR}/CMakeCache.txt"
  rm -f "${PICO_BUILD_DIR}/CMakeCache.txt"
fi

echo "  Pico cmake integration OK."

# =============================================================================
# Done — print a summary
# =============================================================================

section "Setup complete"

echo "  Pico SDK   : $PICO_SDK_ROOT"
echo "  picotool   : $PICOTOOL_BIN"
echo "  ESP-IDF    : $IDF_PATH"
echo ""
echo "  Next steps:"
echo "   1. If you were added to a new group, LOG OUT and LOG BACK IN."
echo "   2. Reconnect USB cables so the new udev rules take effect."
echo "   3. Open VS Code in this repository folder."
echo "   4. Accept the recommended extensions when prompted."
echo "   5. Use the VS Code Tasks menu (Terminal → Run Task) to:"
echo "        PICO: Build             — compile Pico 2 firmware"
echo "        PICO: Flash (picotool)  — flash firmware via USB"
echo "        PICO: Serial Monitor    — open serial console (USB CDC)"
echo "        ESP: Build              — compile ESP32-C6 firmware"
echo "        ESP: Flash + Monitor    — flash and open serial console"
echo ""
if ! command -v idf.py >/dev/null 2>&1; then
  echo "  NOTE: 'idf.py' is not on your PATH in this shell session."
  echo "  IDF_PATH has been exported to /etc/profile.d/99-esp-idf.sh and"
  echo "  ~/.bashrc so it will be available in all new terminal sessions."
  echo "  To activate it in the CURRENT shell without opening a new one, run:"
  echo "    source \"${IDF_PATH}/export.sh\""
  echo ""
fi
