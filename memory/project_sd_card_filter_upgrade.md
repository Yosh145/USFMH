---
name: SD Card Arrival & Filter Upgrade Plan
description: 32 GB SD card on order; plan to move to higher-order FIR filters and MATLAB-generated coefficients once it arrives
type: project
---

32 GB SD card has been ordered to replace the current 8 MB SPIFFS partition.

**Why:** The SPIFFS partition (~2.2 MB usable) limits audio file duration to ~23 seconds at 48 kHz 16-bit mono. The SD card will also allow storing larger filter coefficient tables (5001-tap × 32 bands ≈ 625 KB).

**How to apply:** When the SD card arrives, two upgrades become possible:
1. Longer/higher-quality audio files with no size constraint
2. Use the MATLAB-generated coefficients in MATLAB/matrix.txt (N=5000, Kaiser β=15) instead of the current 201-tap Hamming-window coefficients — requires updating generate_filter_header.py (currently hardcoded to 201 taps and 32 filters; matrix.txt has ~5001 taps and 30 filters)

**Current filter quality gap:** Firmware uses 201-tap Hamming window (~40 dB stopband). MATLAB reference uses 5001-tap Kaiser β=15 (~115 dB stopband). The broadband noise in the filtered spectrum is mainly from this gap, NOT from audio compression (SPIFFS stores uncompressed PCM).

**Immediate improvement available without SD card:** Increase FILTER_BANK_NUM_TAPS to 501 and switch Python coefficient generator (demo.py design_bandpass_fir) from Hamming to Kaiser β=15 window — regenerate filter_bank_coeffs.h. This gives ~3x better transition bandwidth and 75 dB more stopband rejection.
