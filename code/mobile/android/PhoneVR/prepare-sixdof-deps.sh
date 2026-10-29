#!/usr/bin/env bash
# PhoneVR 6DoF fork: reproducible version of prepare-alvr-deps.sh (noGvr only).
#
# Upstream runs `cargo update` and installs the newest cargo-ndk / cbindgen. In 2026 that breaks the
# ALVR v20.8.0 client build (cargo-ndk panics). Here ALVR keeps its own Cargo.lock (May 2024) and the
# tools are pinned to versions from that time. Rust itself is pinned in the workflow (1.78).
set -euo pipefail
cd "$(dirname "$0")"

rustup target add aarch64-linux-android armv7-linux-androideabi x86_64-linux-android i686-linux-android

cargo install --locked cargo-ndk --version 3.5.4
cargo install --locked cbindgen --version 0.26.0

# Cardboard SDK, built exactly like upstream (PhoneVR maintainer's fork of googlevr/cardboard)
rm -rf cardboard cardboard-master
curl -sLS "https://github.com/nift4/cardboard/archive/refs/heads/master.zip" > download.zip
unzip -q download.zip
rm download.zip
pushd cardboard-master
./gradlew sdk:assembleRelease -Parm64-v8a
popd
mkdir cardboard
mv cardboard-master/sdk/build/outputs/aar/sdk-release.aar cardboard/cardboard-sdk.aar
cp cardboard-master/sdk/include/cardboard.h cardboard/cardboard.h
rm -rf cardboard-master
