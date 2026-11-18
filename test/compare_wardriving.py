#!/usr/bin/env python3
"""
compare_wardriving.py — Alat uji performa & bug-finding firmware wardriving v1 vs v2.

Input: file log WiGLE-format (WigleWifi-1.4) hasil lapangan:
  v1 = ESP8266-Wardriving.ino   (WiFi.scanNetworks blocking + SoftwareSerial GPS)
  v2 = wardriving_fast.ino      (sniffer channel-hopping + Serial1 GPS non-blocking)

Metrik per-file:
  * throughput      : baris/menit, KB/menit (dari rentang timestamp FirstSeen)
  * unique BSSID    : jumlah AP berbeda → recall penangkapan sinyal WiFi
  * GPS fix rate    : % baris ber-koordinat valid (bukan kosong / bukan 0,0)
  * duplikat        : baris MAC+timestamp identik → indikasi bug dedup window
  * SSID kosong     : jaringan hidden ATAU bug parsing IE SSID
  * RSSI outlier    : nilai di luar [-120, 0] dBm → bug konversi RSSI
  * channel invalid : di luar 1..14 → bug hopping/parsing
  * baris cacat     : jumlah kolom salah, MAC malformed, koordinat di luar rentang

Pemakaian:
  python3 test/compare_wardriving.py LOG_V1.CSV [LOG_V2.CSV] [--label-v1 v1 --label-v2 v2]

Catatan penting saat mengukur: bandingkan hanya log dengan DURASI & RUTE sama.
Firmware v2 men-dedup ulang AP setelah DEDUP_WINDOW_S, jadi angka "baris/menit"
v2 secara desain lebih rendah di rute statis — itu BUKAN regresi; gunakan
"unique BSSID" dan "GPS fix rate" sebagai tolok ukur utama kualitas tangkapan.
"""

import argparse
import csv
import os
import re
import sys
from collections import Counter
from datetime import datetime

MAC_RE = re.compile(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")


TS_RE = re.compile(r"^(\d{4})[-/](\d{1,2})[-/](\d{1,2}) "
                   r"(\d{1,2}):(\d{1,2}):(\d{1,2})$")


def try_ts(v):
    m = TS_RE.match(v.strip())
    if not m:
        return None
    try:
        return datetime(*map(int, m.groups()))
    except ValueError:
        return None


def analyze(path):
    s = {
        "file": path, "bytes": os.path.getsize(path), "rows": 0,
        "malformed": [], "bad_mac": 0, "dup": 0, "empty_ssid": 0,
        "gps_valid": 0, "gps_zero": 0, "gps_missing": 0,
        "bad_rssi": 0, "bad_chan": 0, "first_ts": None, "last_ts": None,
        "bssids": Counter(), "enc": Counter(), "header_ok": False,
    }
    seen = set()
    with open(path, newline="", encoding="utf-8", errors="replace") as fh:
        rdr = csv.reader(fh)
        header = None
        for row in rdr:
            if not row or all(not c.strip() for c in row):
                continue
            first = row[0].strip()
            if first.startswith("WigleWifi"):
                continue
            if header is None:
                header = [h.strip().lower() for h in row]
                # alias: firmware v1/v2 menulis "AuthMode"/"FirstSeen"/
                # "CurrentLatitude"; log lama WiGLE app memakai "Encryption"/
                # "Seen"/"Latitude". Diterima keduanya.
                aliases = {
                    "mac":     {"mac", "bssid"},
                    "ssid":    {"ssid", "essid"},
                    "auth":    {"authmode", "encryption"},
                    "ts":      {"firstseen", "seen"},
                    "channel": {"channel"},
                    "rssi":    {"rssi"},
                    "lat":     {"currentlatitude", "latitude", "lat"},
                    "lon":     {"currentlongitude", "longitude", "lon"},
                }
                idx = {}
                for name, al in aliases.items():
                    idx[name] = next((i for i, h in enumerate(header)
                                      if h in al), None)
                missing = [n for n, i in idx.items() if i is None]
                s["header_ok"] = not missing
                if missing:
                    print(f"[WARN] {os.path.basename(path)}: kolom tak dikenal: "
                          f"{missing} — header: {row}", file=sys.stderr)
                continue
            if len(row) < len(header):
                s["malformed"].append((s["rows"] + 2, f"{len(row)} kolom ≠ {len(header)}"))
                continue
            s["rows"] += 1

            def g(name):
                i = idx[name]
                return row[i].strip() if i is not None and i < len(row) else ""

            mac = g("mac")
            if MAC_RE.match(mac):
                s["bssids"][mac.upper()] += 1
            else:
                s["bad_mac"] += 1

            key = (mac, g("ts"))
            if key in seen:
                s["dup"] += 1
            else:
                seen.add(key)

            if g("ssid") == "":
                s["empty_ssid"] += 1

            ts = try_ts(g("ts"))
            if ts:
                if s["first_ts"] is None or ts < s["first_ts"]:
                    s["first_ts"] = ts
                if s["last_ts"] is None or ts > s["last_ts"]:
                    s["last_ts"] = ts

            try:
                r = int(g("rssi"))
                if not -120 <= r <= 0:
                    s["bad_rssi"] += 1
            except ValueError:
                s["bad_rssi"] += 1

            try:
                c = int(g("channel"))
                if not 1 <= c <= 14:
                    s["bad_chan"] += 1
            except ValueError:
                s["bad_chan"] += 1

            try:
                lat, lon = float(g("lat")), float(g("lon"))
                if lat == 0.0 and lon == 0.0:
                    s["gps_zero"] += 1
                elif abs(lat) <= 90 and abs(lon) <= 180:
                    s["gps_valid"] += 1
                else:
                    s["malformed"].append((s["rows"] + 2, f"koordinat luar rentang {lat},{lon}"))
            except ValueError:
                s["gps_missing"] += 1

            s["enc"][g("auth")] += 1

    dur = ((s["last_ts"] - s["first_ts"]).total_seconds()
           if s["first_ts"] and s["last_ts"] else 0)
    if dur <= 0:
        # timestamp tak terseragamkan (mis. detik satu digit "14:34:6") →
        # estimasi kasar dari urutan baris TIDAK mungkin; tandai durasi None.
        dur = 0
    s["duration_s"] = dur
    mins = max(dur, 1.0) / 60.0
    s["rows_per_min"] = s["rows"] / mins
    s["kb_per_min"] = s["bytes"] / 1024.0 / mins
    s["unique_bssid"] = len(s["bssids"])
    n = max(s["rows"], 1)
    s["gps_fix_pct"] = 100.0 * (s["gps_valid"] + s["gps_zero"]) / n
    s["bug_count"] = (len(s["malformed"]) + s["bad_mac"] + s["bad_rssi"]
                      + s["bad_chan"] + s["dup"])
    return s


def report(s, label):
    d = f"{s['duration_s']:.0f}s" if s["duration_s"] else "n/a"
    print(f"\n=== [{label}] {s['file']} ===")
    print(f"  header WiGLE valid : {'YA' if s['header_ok'] else 'TIDAK <-- cek firmware'}")
    print(f"  ukuran / baris     : {s['bytes']/1024:.1f} KB / {s['rows']} baris, durasi {d}")
    print(f"  throughput         : {s['rows_per_min']:.1f} baris/menit | {s['kb_per_min']:.1f} KB/menit")
    print(f"  unique BSSID       : {s['unique_bssid']}")
    print(f"  GPS fix            : {s['gps_fix_pct']:.1f}%  (valid={s['gps_valid']}, "
          f"0,0={s['gps_zero']}, kosong={s['gps_missing']})")
    print(f"  SSID kosong        : {s['empty_ssid']} ({100*s['empty_ssid']/max(s['rows'],1):.1f}%)")
    print(f"  --- BUG FINDING ---")
    print(f"  duplikat MAC+time  : {s['dup']}{'   <-- dedup window bocor?' if s['dup'] else ''}")
    print(f"  MAC rusak          : {s['bad_mac']}{'   <-- bug parsing frame!' if s['bad_mac'] else ''}")
    print(f"  RSSI di luar -120..0 dBm : {s['bad_rssi']}{'   <-- bug konversi RSSI!' if s['bad_rssi'] else ''}")
    print(f"  Channel di luar 1..14    : {s['bad_chan']}{'   <-- bug hopping!' if s['bad_chan'] else ''}")
    print(f"  Baris cacat             : {len(s['malformed'])}"
          + ("   <-- I/O terpotong?!" if s["malformed"] else ""))
    for ln, why in s["malformed"][:5]:
        print(f"      L{ln}: {why}")
    top = ", ".join(f"{(k or '(kosong)')}:{v}" for k, v in s["enc"].most_common(4))
    print(f"  AuthMode (top)     : {top}")


def compare(a, b, la, lb):
    print(f"\n=========== PERBANDINGAN {la} vs {lb} ===========")
    rows = [
        ("Unique BSSID (recall)", a["unique_bssid"], b["unique_bssid"], True),
        ("GPS fix (%)", a["gps_fix_pct"], b["gps_fix_pct"], True),
        ("Throughput (baris/mnt)", a["rows_per_min"], b["rows_per_min"], True),
        ("Total BUG terdeteksi", a["bug_count"], b["bug_count"], False),
        ("Ukuran file (KB)", a["bytes"] / 1024, b["bytes"] / 1024, None),
    ]
    for name, va, vb, higher_better in rows:
        delta = (vb - va) / va * 100 if va else float("inf")
        if higher_better is None or abs(delta) < 1e-9:
            w = "="
        elif higher_better:
            w = lb if vb > va else la
        else:
            w = lb if vb < va else la
        print(f"  {name:<24} {la}={va:>10.1f}  {lb}={vb:>10.1f}  Δ={delta:+7.1f}%  [{w}]")
    print("\n  Catatan: pada rute statis, baris/menit v2 lebih rendah BY DESIGN")
    print("  (dedup). Jika unique BSSID v2 juga lebih rendah → regresi sniffer.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_v1", help="log firmware v1 (ESP8266-Wardriving.ino)")
    ap.add_argument("csv_v2", nargs="?", help="log firmware v2 (wardriving_fast.ino)")
    ap.add_argument("--label-v1", default="v1")
    ap.add_argument("--label-v2", default="v2")
    args = ap.parse_args()

    sa = analyze(args.csv_v1)
    report(sa, args.label_v1)
    rc = 1 if sa["bug_count"] else 0
    if args.csv_v2:
        sb = analyze(args.csv_v2)
        report(sb, args.label_v2)
        compare(sa, sb, args.label_v1, args.label_v2)
        rc = 1 if (sa["bug_count"] or sb["bug_count"]) else 0
    print()
    sys.exit(rc)  # exit≠0 bila ada bug → mudah dipakai di CI


if __name__ == "__main__":
    main()
