#!/usr/bin/env bash

set -euo pipefail

IDF_PATH="${1:-}"

# Try to fall back to the workspace setting if no argument was provided.
if [ -z "$IDF_PATH" ] && [ -f ".vscode/settings.json" ]; then
  IDF_PATH=$(python3 -c 'import json,sys; d=json.load(open(".vscode/settings.json")); print(d.get("idf.currentSetup",""))')
fi

# Final fallback: use the common default location for ESP-IDF.
if [ -z "$IDF_PATH" ]; then
  IDF_PATH="${HOME}/.espressif/v5.5.2/esp-idf"
  echo "INFO: idf.currentSetup not set; defaulting to $IDF_PATH"
fi

# Validate we have a non-empty path at this point.
if [ -z "$IDF_PATH" ]; then
  echo "ERROR: Unable to determine ESP-IDF path. Set 'idf.currentSetup' in .vscode/settings.json or pass it as an argument to this script."
  exit 1
fi

# Helper: install packages via distro package manager (if sudo available)
install_packages() {
  local pkgs=("$@")

  if [ "$EUID" -ne 0 ] && command -v sudo >/dev/null; then
    SUDO="sudo"
  else
    SUDO=""
  fi

  if command -v apt-get >/dev/null; then
    $SUDO apt-get update -y
    $SUDO apt-get install -y "${pkgs[@]}"
    return
  fi

  if command -v dnf >/dev/null; then
    $SUDO dnf install -y "${pkgs[@]}"
    return
  fi

  if command -v pacman >/dev/null; then
    $SUDO pacman -Sy --noconfirm "${pkgs[@]}"
    return
  fi

  if command -v zypper >/dev/null; then
    $SUDO zypper install -y "${pkgs[@]}"
    return
  fi

  echo "WARNING: Could not detect a supported package manager (apt/dnf/pacman/zypper)."
  echo "Please install the following packages manually: ${pkgs[*]}"
}

# Ensure python3 + pip exist
if ! command -v python3 >/dev/null; then
  echo "python3 not found; attempting to install via package manager..."
  install_packages python3 python3-pip
fi

if ! python3 -m pip --version >/dev/null 2>&1; then
  echo "pip not found; attempting to bootstrap via ensurepip..."
  python3 -m ensurepip --upgrade || true
fi

# Ensure cmake is available
if ! command -v cmake >/dev/null; then
  echo "cmake not found; attempting to install via package manager..."
  install_packages cmake
fi

# Ensure git is available
if ! command -v git >/dev/null; then
  echo "git not found; attempting to install via package manager..."
  install_packages git
fi

# Ensure required build tools exist for esp-idf / pico
if ! command -v cmake >/dev/null || ! command -v git >/dev/null || ! command -v python3 >/dev/null; then
  echo "ERROR: Required tools are still missing. Please install git, cmake, and python3." >&2
  exit 1
fi

# Setup Pico SDK (used by Pico firmware)
PICO_SDK_ROOT="${HOME}/.pico-sdk/sdk/2.2.0"
if [ ! -d "$PICO_SDK_ROOT" ]; then
  echo "Pico SDK not found at '$PICO_SDK_ROOT'; cloning pico-sdk v2.2.0..."
  mkdir -p "$(dirname "$PICO_SDK_ROOT")"
  git clone -b 2.2.0 --depth 1 https://github.com/raspberrypi/pico-sdk.git "$PICO_SDK_ROOT"
  # Ensure pico-sdk submodules are initialized
  git -C "$PICO_SDK_ROOT" submodule update --init --recursive
fi

# Ensure ESP-IDF is cloned
if [ ! -d "$IDF_PATH" ]; then
  echo "ESP-IDF not found at '$IDF_PATH'; cloning esp-idf v5.5.2..."
  mkdir -p "$(dirname "$IDF_PATH")"
  git clone -b v5.5.2 --depth 1 https://github.com/espressif/esp-idf.git "$IDF_PATH"
fi

# Install ESP-IDF python tools (and build tools like ninja, cmake, etc.)
if ! compgen -G "${HOME}/.espressif/python_env/idf*_py*_env/bin/python" > /dev/null; then
  echo "ESP-IDF tools not installed; running '$IDF_PATH/install.sh'..."
  "$IDF_PATH/install.sh"
fi

# Generate workspace VS Code settings if missing (helps new installs)
create_vscode_settings() {
  local settings_path="$1"
  local content="$2"

  if [ -f "$settings_path" ]; then
    return
  fi

  mkdir -p "$(dirname "$settings_path")"
  echo "$content" > "$settings_path"
  echo "Created VS Code settings at $settings_path"
}

root_settings_content=$(cat <<'EOF'
{
  "idf.currentSetup": "${IDF_PATH}",
  "cmake.sourceDirectory": "${workspaceFolder}/pico_fw/src"
}
EOF
)

esp_settings_content=$(cat <<'EOF'
{
  "idf.currentSetup": "${IDF_PATH}",
  "idf.openOcdConfigs": [
    "board/esp32c6-builtin.cfg"
  ],
  "idf.customExtraVars": {
    "IDF_TARGET": "esp32c6"
  },
  "idf.port": "/dev/ttyACM0",
  "idf.flashType": "UART",
  "cmake.sourceDirectory": "${workspaceFolder}/lcd_uext_ili9341"
}
EOF
)

# Attempt to resolve workspaceFolder for VS Code settings (best-effort)
workspaceFolder="$(pwd)"
create_vscode_settings "$workspaceFolder/.vscode/settings.json" "$root_settings_content"
create_vscode_settings "$workspaceFolder/lcd_uext_ili9341/.vscode/settings.json" "$esp_settings_content"

# Provide guidance if the install script didn't install key tools
if ! command -v idf.py >/dev/null; then
  echo "WARNING: 'idf.py' not found on PATH. You may need to source '$IDF_PATH/export.sh' or add it to your shell init." >&2
fi

echo "Setup complete."
