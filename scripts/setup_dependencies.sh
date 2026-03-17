#!/usr/bin/env bash

set -euo pipefail

IDF_PATH="${1:-}"

if [ -z "$IDF_PATH" ]; then
  echo "ERROR: Missing ESP-IDF path argument (from idf.currentSetup workspace setting)."
  echo "Set 'idf.currentSetup' in .vscode/settings.json and re-run this task."
  exit 1
fi

# Ensure cmake is available (try pip install fallback)
if ! command -v cmake >/dev/null; then
  echo "cmake not found; attempting python -m pip install --user cmake..."
  python3 -m pip install --user cmake
fi

# Ensure git is available
if ! command -v git >/dev/null; then
  echo "git not found; please install git via your package manager."
  exit 1
fi

# Clone ESP-IDF if missing
if [ ! -d "$IDF_PATH" ]; then
  echo "ESP-IDF not found at '$IDF_PATH'; cloning esp-idf v5.5.2..."
  mkdir -p "$(dirname "$IDF_PATH")"
  git clone -b v5.5.2 --depth 1 https://github.com/espressif/esp-idf.git "$IDF_PATH"
fi

# Ensure ESP-IDF python tools installed
if ! compgen -G "${HOME}/.espressif/python_env/idf*_py*_env/bin/python" > /dev/null; then
  echo "ESP-IDF tools not installed; running '$IDF_PATH/install.sh'..."
  "$IDF_PATH/install.sh"
fi

echo "Setup complete."
