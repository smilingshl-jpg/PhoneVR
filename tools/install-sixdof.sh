#!/usr/bin/env bash
# Installs a PhoneVR 6DoF build from GitHub Actions as an UPDATE (keeps the app's permissions and viewer setup).
#
# Each CI run signs with a brand-new throwaway debug key, so Android refuses to install one build over another.
# This re-signs the APK with one fixed key that stays on this PC (never in git or on GitHub), then installs it.
#
#   tools/install-sixdof.sh <artifact .zip or .apk> [adb serial]
#
# Key: %USERPROFILE%\.phonevr6dof\sixdof-signing.jks, password in keystore.properties next to it.
# Losing the key means one more uninstall + reinstall, nothing worse.
set -euo pipefail

src="${1:?usage: install-sixdof.sh <artifact .zip or .apk> [adb serial]}"
serial="${2:-}"
keydir="${PHONEVR6DOF_KEYDIR:-$HOME/.phonevr6dof}"
props="$keydir/keystore.properties"
sdk="${ANDROID_HOME:-/c/Android/Sdk}"
jdk="${JAVA_HOME:-/c/Program Files/Eclipse Adoptium/jdk-21.0.12.101-hotspot}"

[ -f "$props" ] || { echo "no signing key: $props missing" >&2; exit 1; }
prop() { sed -n "s/^$1=//p" "$props" | tr -d '\r'; }
ks="$keydir/sixdof-signing.jks"
pw="$(prop storePassword)"
alias="$(prop keyAlias)"

# newest build-tools that has apksigner
apksigner="$(ls -d "$sdk"/build-tools/*/ | sort -V | tail -1)lib/apksigner.jar"
[ -f "$apksigner" ] || { echo "apksigner not found under $sdk/build-tools" >&2; exit 1; }

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
case "$src" in
    *.zip) unzip -q -o "$src" '*.apk' -d "$work/in"; apk="$(ls "$work"/in/*.apk | head -1)" ;;
    *.apk) apk="$src" ;;
    *) echo "expected a .zip or .apk: $src" >&2; exit 1 ;;
esac
out="$work/PhoneVR-6DoF-signed.apk"

# the password goes through the environment, not the command line
PHONEVR6DOF_PW="$pw" "$jdk/bin/java" -jar "$apksigner" sign --ks "$ks" --ks-key-alias "$alias" \
    --ks-pass env:PHONEVR6DOF_PW --key-pass env:PHONEVR6DOF_PW --out "$out" "$apk"
"$jdk/bin/java" -jar "$apksigner" verify "$out"

adb ${serial:+-s "$serial"} install -r "$out"
