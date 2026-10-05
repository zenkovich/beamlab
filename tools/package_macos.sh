#!/bin/bash
# Builds a distributable BeamLab.app (universal arm64 + x86_64, macOS 11+) and zips it into dist/.
# Only the vehicle files that the app actually reads are packaged (sounds, sources, unused meshes are left out):
# every vehicle variant and scene is loaded once with BL_TRACE_FILES and the traced files are copied.
#
# Usage: tools/package_macos.sh            -> dist/BeamLab-macOS.zip
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT="$(pwd)"
BUILD="$ROOT/build_dist"
OUT="$ROOT/dist"
STAGE="$OUT/BeamLab"
APP="$STAGE/BeamLab.app"
ZIP="$OUT/BeamLab-macOS.zip"

echo "== build (universal, portable)"
cmake -S . -B "$BUILD" -G Ninja -DCMAKE_BUILD_TYPE=Release -DBEAMLAB_PORTABLE=ON \
    "-DCMAKE_OSX_ARCHITECTURES=arm64;x86_64" -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 > /dev/null
cmake --build "$BUILD" --target beamlab

echo "== trace the asset files in use"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
TRACE="$TMP/trace.log"
python3 - "$ROOT/assets/vehicles" > "$TMP/ids.txt" << 'EOF'
import os, sys
root = sys.argv[1]
for d in sorted(os.listdir(root)):
    p = os.path.join(root, d)
    if os.path.isdir(p):
        for f in sorted(os.listdir(p)):
            s, e = os.path.splitext(f)
            if e.lower() in ('.truck', '.car', '.trailer', '.load'):
                print(f"{d}/{s}")
EOF
n=0
while IFS= read -r id; do
    BL_TRACE_FILES="$TRACE" "$BUILD/beamlab" --scene proving --vehicle "$id" --frames 4 --size 320x180 --hidden \
        --screenshot "$TMP/t.png" > /dev/null 2>&1 || echo "   warning: '$id' failed to run"
    n=$((n + 1))
done < "$TMP/ids.txt"
for sc in forest canyon offroad crash vehicle_crash lab test_site stress_vehicles stress_derby stress_bridge; do
    BL_TRACE_FILES="$TRACE" "$BUILD/beamlab" --scene "$sc" --frames 4 --size 320x180 --hidden \
        --screenshot "$TMP/t.png" > /dev/null 2>&1 || echo "   warning: scene '$sc' failed to run"
done
echo "   $n vehicle definitions, $(sort -u "$TRACE" | wc -l | tr -d ' ') files read"

echo "== stage BeamLab.app"
rm -rf "$STAGE" "$ZIP"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources/assets"
cp "$BUILD/beamlab" "$APP/Contents/MacOS/beamlab"
strip -S "$APP/Contents/MacOS/beamlab"
cp -R "$ROOT/assets/shaders" "$ROOT/assets/fonts" "$APP/Contents/Resources/assets/"
# the open (CC0) textures, sky and models of tools/fetch_assets.py: the scenes' and the cars' look (the RBR stages are not
# redistributable and stay out)
for d in textures models; do
    [ -d "$ROOT/assets/$d" ] && cp -R "$ROOT/assets/$d" "$APP/Contents/Resources/assets/"
done
python3 - "$ROOT/assets" "$APP/Contents/Resources/assets" "$TRACE" << 'EOF'
import os, shutil, sys
src, dst, trace = sys.argv[1:4]
src = os.path.realpath(src)
files = set()
for line in open(trace):
    p = os.path.realpath(line.strip())
    if p.startswith(src + os.sep) and os.sep + 'vehicles' + os.sep in p:
        files.add(os.path.relpath(p, src))
# licences / credits of every mod
for d in os.listdir(os.path.join(src, 'vehicles')):
    s = os.path.join('vehicles', d, 'SOURCE.txt')
    if os.path.isfile(os.path.join(src, s)):
        files.add(s)
total = 0
for rel in sorted(files):
    os.makedirs(os.path.dirname(os.path.join(dst, rel)), exist_ok=True)
    shutil.copy2(os.path.join(src, rel), os.path.join(dst, rel))
    total += os.path.getsize(os.path.join(src, rel))
print(f"   {len(files)} vehicle files, {total / 1e6:.0f} MB")
EOF
cat > "$APP/Contents/Info.plist" << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleExecutable</key><string>beamlab</string>
    <key>CFBundleIdentifier</key><string>local.beamlab.prototype</string>
    <key>CFBundleName</key><string>BeamLab</string>
    <key>CFBundleDisplayName</key><string>BeamLab</string>
    <key>CFBundlePackageType</key><string>APPL</string>
    <key>CFBundleShortVersionString</key><string>1.0</string>
    <key>CFBundleVersion</key><string>1</string>
    <key>LSMinimumSystemVersion</key><string>11.0</string>
    <key>NSHighResolutionCapable</key><true/>
</dict>
</plist>
EOF
# ad-hoc signature over the whole bundle (an unsigned/partially signed bundle is reported as "damaged")
codesign --force --deep --sign - "$APP"
cp "$ROOT/tools/package_readme.txt" "$STAGE/README.txt"

echo "== zip"
(cd "$OUT" && ditto -c -k --keepParent BeamLab "$(basename "$ZIP")")
du -sh "$APP" "$ZIP"
