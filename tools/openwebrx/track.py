"""Track the strongest peak in a window through OpenWebRX over time: select a
profile, and for every FFT frame record (UTC time, peak frequency); print one
line per second, then restore the previously active profile."""
import json, math, sys, time
from datetime import datetime, timezone
import websocket
# IMA ADPCM, as in openwebrx's ImaAdpcmCodec
IDX = [-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8]
STEP = [7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97,
        107, 118, 130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
        876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871,
        5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623,
        27086, 29794, 32767]

def adpcm_decode(data):
    out, pred, idx = [], 0, 0
    for byte in data:
        for nib in (byte & 0x0F, byte >> 4):
            step = STEP[idx]
            diff = step >> 3
            if nib & 1: diff += step >> 2
            if nib & 2: diff += step >> 1
            if nib & 4: diff += step
            if nib & 8: diff = -diff
            pred = max(-32768, min(32767, pred + diff))
            idx = max(0, min(88, idx + IDX[nib]))
            out.append(pred)
    return out


URL = "ws://192.168.13.5:8073/ws/"
PROFILE, F_LO, F_HI, SECONDS = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])

ws = websocket.create_connection(URL, timeout=10)
ws.send("SERVER DE CLIENT client=claude-track type=receiver")
ws.send(json.dumps({"type": "connectionproperties", "params": {"output_rate": 12000, "hd_output_rate": 48000}}))
cfg, orig, t0, rows = {}, None, None, []
while True:
    m = ws.recv()
    if isinstance(m, str):
        if m.startswith("CLIENT DE SERVER"): continue
        j = json.loads(m)
        if j.get("type") == "config":
            cfg.update(j["value"])
            if orig is None and "sdr_id" in cfg and "profile_id" in cfg:
                orig = f'{cfg["sdr_id"]}|{cfg["profile_id"]}'
                if orig != PROFILE: ws.send(json.dumps({"type": "selectprofile", "params": {"profile": PROFILE}}))
            if f'{cfg.get("sdr_id")}|{cfg.get("profile_id")}' == PROFILE and t0 is None:
                t0 = time.time() + 3
        continue
    if not m or m[0] != 1 or t0 is None or time.time() < t0: continue
    if time.time() > t0 + SECONDS: break
    v = [x / 100 for x in adpcm_decode(m[1:])][10:]
    cf, sr, N = cfg["center_freq"], cfg["samp_rate"], len(v)
    bw = sr / N
    lo, hi = int((F_LO - cf + sr / 2) / bw), int((F_HI - cf + sr / 2) / bw)
    i = max(range(lo, hi), key=lambda k: v[k])
    a, b, c = v[i - 1], v[i], v[i + 1]
    off = 0.5 * (a - c) / (a - 2 * b + c) if (a - 2 * b + c) else 0
    snr = b - sorted(v[lo:hi])[(hi - lo) // 2]
    rows.append((time.time(), cf - sr / 2 + (i + 0.5 + off) * bw, snr))
if orig and orig != PROFILE:
    ws.send(json.dumps({"type": "selectprofile", "params": {"profile": orig}}))
    time.sleep(1)
ws.close()
# one line per whole second: median frequency of frames with a clear peak
by = {}
for t, f, s in rows:
    if s > 10: by.setdefault(int(t), []).append(f)
for sec in sorted(by):
    fs = sorted(by[sec])
    print(datetime.fromtimestamp(sec, timezone.utc).strftime("%H:%M:%S"), f"{fs[len(fs)//2]/1e6:.6f}", len(fs))
print("restored", orig, file=sys.stderr)
