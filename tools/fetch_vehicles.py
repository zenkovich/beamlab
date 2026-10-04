#!/usr/bin/env python3
"""Download Rigs of Rods vehicle mods from the official RoR repository
(forum.rigsofrods.org resource manager, same endpoint the RoR game client uses)
and unpack them into assets/vehicles/<name>/.

Usage: python3 tools/fetch_vehicles.py [--force]
"""
import io, os, sys, zipfile, urllib.request

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets", "vehicles")

# The vehicle menu's section of each folder (written to SOURCE.txt as "kind: ..."; a folder not named here: Cars)
KINDS = {"mercedes_vito": "SUVs, vans, pickups", "mercedes_w460": "SUVs, vans, pickups", "mitsubishi_pajero": "SUVs, vans, pickups",
         "ford_f_1999": "SUVs, vans, pickups", "ford_f250_2014": "SUVs, vans, pickups", "nissan_d21": "SUVs, vans, pickups", "trophy_truck_v2": "Off-road",
         "lcf_trucks": "Trucks", "autocar_xpeditor": "Trucks", "kme_predator": "Trucks", "freightliner_fla": "Trucks", "kenworth_wrecker": "Trucks",
         "tatra_815_6x6": "Trucks", "thomas_hdx_bus": "Buses", "man_caetano_enigma": "Buses"}

# (folder, resource_id, file_id, description)
VEHICLES = [
    # cars
    ("bmw_e36",            286, 15797, "BMW E36 sedan (Nadeox1, CC BY-NC-SA 3.0)"),
    ("bmw_e39_m5",         336,  8957, "BMW E39 M5 sedan (PrTAudiman, Gabester)"),
    ("audi_quattro",        85, 14930, "1988 Audi Quattro coupe (TatangJose, Flystyle, CuriousMike)"),
    ("audi_80",            474, 24339, "Audi 80 (B2) sedan (OnyxPL, Mythbuster, PrT_Audiman, kheridr)"),
    ("mercedes_clk",       387, 21241, "Mercedes-Benz CLK (C208) coupe (Gabester, Mitchieboy, CuriousMike)"),
    ("seat_ibiza",         335, 13481, "SEAT Ibiza (6K) hatchback (Mythbuster, Gavas_Bean, Xploder98, Gavas__Bean)"),
    ("toyota_ae86",        612,  6979, "Toyota AE86 Trueno / Levin (Stoat_Muldoon, Prt_Audiman)"),
    ("dodge_viper",         88,  7191, "1996 Dodge Viper GTS"),
    # (single-file resources: the file id is not needed, 0 serves the one file)
    ("subaru_impreza",     333,     0, "Subaru Impreza WRX 1997 sedan, coupe, wagon, 22B (Stoat_Muldoon, CuriousMike)"),
    ("ford_crown_victoria", 147,    0, "1999 Ford Crown Victoria (Creak, silvermanblue, Jesse, Nadeox1, Lt. Smell My, josh99, Offorader23, CuriousMike)"),
    ("camaro_iroc_z",      285,     0, "Chevrolet Camaro IROC-Z and Pontiac Firebird Trans Am (Stoat_Muldoon, PrT_Audiman, CuriousMike)"),
    ("chevelle",          1263,     0, "1970/1972 Chevrolet Chevelle and El Camino (Stoat_Muldoon, CuriousMike, Mark)"),
    ("citroen_zx",        1274,     0, "Citroen ZX 3-door, hatchback, Break (masfilip, GamerN3x, Gouranga, CuriousMike)"),
    ("mazda_626_gf",      1082,     0, "Mazda 626 GF 1.8i sedan (masfilip)"),
    ("audi_a4",            332,     0, "Audi A4 / S4 / RS4 (Jalkku)"),
    # vans, SUVs, pickups, off-road
    ("mercedes_vito",      386, 21448, "Mercedes-Benz Vito (W639) van (Mitchieboy, Gabester, Mythbuster, Stoat_Muldoon)"),
    ("mercedes_w460",      466, 15916, "Mercedes-Benz G-Class (W460) (Deadlyquasar)"),
    ("mitsubishi_pajero",  478, 24427, "Mitsubishi Pajero SUV (Masa, CuriousMike)"),
    ("ford_f_1999",         17, 24946, "1999 Ford F-Series pickup"),
    ("ford_f250_2014",      97,  7136, "2014 Ford F-250 Super Duty pickup (Stoat_Muldoon, Negativeice, VeyronEB, CuriousMike)"),
    ("nissan_d21",         465,     0, "Nissan King Cab Hardbody D21 pickup (dmtactical)"),
    ("trophy_truck_v2",     89,  6601, "Unlimited class trophy truck (HemiBoy, CuriousMike, ShawnVallance, Zikmester96)"),
    # trucks and buses
    ("lcf_trucks",         423, 14877, "Isuzu-style LCF medium trucks (NEG)"),
    ("autocar_xpeditor",  1229, 30384, "Autocar Xpeditor heavy truck (NEGICE)"),
    ("kme_predator",       254,  9070, "2008 KME Predator fire engine (Voulk/Pascal, ThatsNice, Renault_Bird, Eminox)"),
    ("freightliner_fla",   601,  6825, "Freightliner FLA cabover semi truck"),
    ("kenworth_wrecker",   628,  7289, "Kenworth T800 50-ton wrecker"),
    ("tatra_815_6x6",      709,  9085, "TATRA 815 6x6 heavy truck"),
    ("thomas_hdx_bus",      28,  1629, "Thomas Saf-T-Liner HDX school bus"),
    ("man_caetano_enigma", 356, 19686, "MAN Caetano Enigma coach (Pedro)"),
]

def main():
    force = "--force" in sys.argv
    os.makedirs(ROOT, exist_ok=True)
    for folder, rid, fid, desc in VEHICLES:
        dst = os.path.join(ROOT, folder)
        if os.path.isdir(dst) and os.listdir(dst) and not force:
            print(f"[skip] {folder}")
            continue
        url = f"https://forum.rigsofrods.org/resources/{rid}/download?file={fid}"
        print(f"[get ] {folder:20s} <- {url}")
        try:
            data = urllib.request.urlopen(url, timeout=300).read()
            zf = zipfile.ZipFile(io.BytesIO(data))
        except Exception as e:
            print(f"   FAILED: {e}")
            continue
        os.makedirs(dst, exist_ok=True)
        zf.extractall(dst)
        with open(os.path.join(dst, "SOURCE.txt"), "w") as f:
            f.write(f"{desc}\nhttps://forum.rigsofrods.org/resources/{rid}/\nkind: {KINDS.get(folder, 'Cars')}\n")
        print(f"       {len(zf.namelist())} files, {len(data)/1e6:.1f} MB")

if __name__ == "__main__":
    main()
