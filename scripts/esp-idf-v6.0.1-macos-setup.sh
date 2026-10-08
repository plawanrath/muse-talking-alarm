#!/usr/bin/env bash
#
# ESP-IDF v6.0.1 toolchain setup for macOS
# Prepared for: talking alarm clock + voice assistant speaker build
# Board: Waveshare ESP32-S3-AUDIO-Board (ESP32-S3, Path A: custom Gadget SDK board profile)
#
# The Muse Gadget SDK's official docs pin ESP-IDF to v6.0.1 exactly on macOS.
# Run:  bash esp-idf-v6.0.1-macos-setup.sh
# Takes ~20-40 min and ~2-3 GB of disk on first run. Safe to re-run: every
# step checks first and skips work that is already done.
#
set -euo pipefail

ESP_IDF_VERSION="v6.0.1"
IDF_PATH="${IDF_PATH:-$HOME/esp/esp-idf}"

step() { echo ""; echo "== $1 =="; }

# 1. Homebrew ----------------------------------------------------------------
step "Homebrew"
if ! command -v brew >/dev/null 2>&1; then
  echo "Homebrew not found. Install it first:"
  echo "  /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\""
  exit 1
fi
echo "Homebrew OK: $(brew --version | head -1)"

# 2. Documented macOS prerequisites -------------------------------------------
step "Prerequisites (cmake, ninja, dfu-util, python3)"
for pkg in cmake ninja dfu-util python3; do
  if brew list "$pkg" >/dev/null 2>&1; then
    echo "  $pkg already installed, skipping"
  else
    echo "  installing $pkg ..."
    brew install "$pkg"
  fi
done

# 3. ESP-IDF v6.0.1 -------------------------------------------------------------
step "ESP-IDF ${ESP_IDF_VERSION} at ${IDF_PATH}"
if [ -d "${IDF_PATH}/.git" ]; then
  echo "ESP-IDF repo already present, syncing to ${ESP_IDF_VERSION} ..."
  git -C "${IDF_PATH}" fetch --tags
  git -C "${IDF_PATH}" checkout "${ESP_IDF_VERSION}"
  git -C "${IDF_PATH}" submodule update --init --recursive
else
  mkdir -p "$(dirname "${IDF_PATH}")"
  echo "Cloning ESP-IDF (recursive, this takes a while) ..."
  git clone -b "${ESP_IDF_VERSION}" --recursive \
    https://github.com/espressif/esp-idf.git "${IDF_PATH}"
fi

# 4. Toolchain install (ESP32-S3 target) ----------------------------------------
step "Installing ESP-IDF tools for esp32s3"
cd "${IDF_PATH}"
./install.sh esp32s3

# 5. Verify --------------------------------------------------------------------
step "Verification"
# shellcheck disable=SC1091
. ./export.sh >/dev/null 2>&1
idf.py --version
echo ""
echo "Toolchain ready."

# 6. Next steps ------------------------------------------------------------------
cat <<'EOF'

Next steps (manual, one time):
  1. Add this to your shell profile (~/.zshrc) so idf.py is always available:
       alias get_idf=". $HOME/esp/esp-idf/export.sh"
     then run:  get_idf
  2. Get a Muse Gadget SDK token at https://gadgets.muse.ai (the SDK requires it).
  3. When the board arrives (due ~Oct 6), plug it in and confirm the serial port:
       ls /dev/cu.usb*
     The SDK's flashing instructions use that /dev/cu.usb* port.
  4. Path A work: add the custom board profile (ES8311 codec, ES7210 mics,
     buttons, 7x RGB LEDs, PCF85063 RTC) per the talking-alarm-clock tracker.
EOF
