"""Measure a carrier through OpenWebRX as an ordinary web client: select a
profile, average the FFT (waterfall) frames, report the strongest peaks in a
window, then restore the profile that was active before."""
import json, sys, time
import websocket  # websocket-client

URL = "ws://192.168.13.5:8073/ws/"
PROFILE = sys.argv[1] if len(sys.argv) > 1 else "rtlsdr|23cm"
F_LO, F_HI = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (1296.70e6, 1296.90e6)
SECONDS = float(sys.argv[4]) if len(sys.argv) > 4 else 20

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

ws = websocket.create_connection(URL, timeout=10)
ws.send("SERVER DE CLIENT client=claude-measure type=receiver")
ws.send(json.dumps({"type": "connectionproperties", "params": {"output_rate": 12000, "hd_output_rate": 48000}}))
cfg, orig_profile, acc, n, t_end, switched = {}, None, None, 0, None, False
while True:
    m = ws.recv()
    if isinstance(m, str):
        if m.startswith("CLIENT DE SERVER"): continue
        j = json.loads(m)
        if j.get("type") == "config":
            cfg.update(j["value"])
            if orig_profile is None and "sdr_id" in cfg and "profile_id" in cfg:
                orig_profile = f'{cfg["sdr_id"]}|{cfg["profile_id"]}'
                print("active profile before:", orig_profile, flush=True)
                if orig_profile != PROFILE:
                    ws.send(json.dumps({"type": "selectprofile", "params": {"profile": PROFILE}}))
                    switched = True
            if f'{cfg.get("sdr_id")}|{cfg.get("profile_id")}' == PROFILE and t_end is None:
                t_end = time.time() + 3 + SECONDS  # let the SDR settle 3 s
                t_start = time.time() + 3
        continue
    if not m or m[0] != 1 or t_end is None or time.time() < t_start: 
        if t_end and time.time() > t_end: break
        continue
    if cfg.get("fft_compression") == "adpcm":
        v = [x / 100 for x in adpcm_decode(m[1:])][10:]
    else:
        import struct; v = list(struct.unpack(f"<{(len(m)-1)//4}f", m[1:]))
    v = [10 ** (x / 10) for x in v]  # average in power
    acc = v if acc is None else [a + b for a, b in zip(acc, v)]
    n += 1
    if time.time() > t_end: break

cf, sr = cfg["center_freq"], cfg["samp_rate"]
N = len(acc); bw = sr / N
import math
db = [10 * math.log10(a / n) for a in acc]
freq = lambda i: cf - sr / 2 + (i + 0.5) * bw
win = [i for i in range(N) if F_LO <= freq(i) <= F_HI]
noise = sorted(db[i] for i in win)[len(win) // 2]
peaks = sorted((db[i], i) for i in win if db[i] == max(db[max(0, i - 3):i + 4]))[-5:][::-1]
print(f"profile {PROFILE}: centre {cf/1e6:.6f} MHz, samp {sr/1e6:.3f} MHz, fft {N} ({bw:.0f} Hz/bin), {n} frames averaged")
print(f"noise floor in window ~{noise:.1f} dB")
for d, i in peaks:
    # parabolic interpolation around the peak bin
    a, b, c = db[i - 1], db[i], db[i + 1]
    off = 0.5 * (a - c) / (a - 2 * b + c) if (a - 2 * b + c) else 0
    print(f"  peak {(freq(i) + off * bw)/1e6:.6f} MHz  {d - noise:+.1f} dB above noise")
if switched and orig_profile:
    ws.send(json.dumps({"type": "selectprofile", "params": {"profile": orig_profile}}))
    time.sleep(1)
    print("restored profile:", orig_profile)
ws.close()
