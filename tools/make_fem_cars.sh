#!/bin/bash
# Regenerates the FEM cars made from Rigs of Rods mods (assets/vehicles/shell_car/fem_<car>.truck): each mod's meshes
# are exported by the game (BL_EXPORT_FLEX) into build/fem_export/<folder>, then tools/make_part_car.py builds the car.
#   tools/make_fem_cars.sh [car ...]      (default: all; the cars: CARS in tools/make_part_car.py)
# Needs build/beamlab and the mods (tools/fetch_vehicles.py). One beamlab at a time: do not run the game meanwhile.
cd "$(dirname "$0")/.." || exit 1
OUT=build/fem_export
mkdir -p "$OUT"
python3 - "$@" <<'P' | while read -r car src; do
import ast, re, sys
src = open("tools/make_part_car.py").read()
names = re.findall(r'^    "(\w+)": dict\(\s*\n?\s*title="[^"]*", src="([^"]+)"', src, re.M)
want = sys.argv[1:]
for car, s in names:
    if not want or car in want:
        print(car, s)
P
    folder=${src%%/*}
    if [ ! -d "assets/vehicles/$folder" ]; then echo "[skip] $car: assets/vehicles/$folder is not there (tools/fetch_vehicles.py)"; continue; fi
    mkdir -p "$OUT/$folder"
    BL_EXPORT_FLEX="$OUT/$folder" ./build/beamlab --scene proving --vehicle "$src" --frames 2 --hidden --screenshot "$OUT/$folder/export.png" > "$OUT/$car.export.log" 2>&1
    if python3 tools/make_part_car.py "$car" "$OUT/$folder" > "$OUT/$car.log" 2>&1; then
        echo "[ok  ] $car: $(tail -1 "$OUT/$car.log" | sed 's/.*fem_/fem_/' | cut -c1-150)"
    else
        echo "[FAIL] $car: see $OUT/$car.log"; tail -3 "$OUT/$car.log"
    fi
done
