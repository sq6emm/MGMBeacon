# Measuring the beacon through OpenWebRX

Both scripts connect to an OpenWebRX receiver as an ordinary web client
(websocket-client), switch it to the given profile, read the waterfall FFT
(ADPCM-compressed, decoded as in openwebrx.js), and switch back to the
profile that was active before. Anyone listening at that moment hears a
short interruption. `URL` at the top points at the SP6PLH receiver.

    pip install websocket-client
    python measure.py 'rtlsdr|23cm' 1296.70e6 1296.90e6 30   # strongest peaks, 30 s average
    python track.py   'rtlsdr|23cm' 1296.74e6 1296.88e6 70   # peak frequency, one line per second

Resolution is the FFT bin (2.4 MHz / 4096 = 586 Hz) with parabolic
interpolation. The RTL-SDR has no ppm correction configured, so absolute
readings are only good to a few ppm; use them to compare, or calibrate
against a signal of known frequency first.
