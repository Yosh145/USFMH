# MATLAB Reference Filter Bank

This directory holds the **ground-truth** 32-band FIR design used to validate
the firmware's runtime filter bank. The MATLAB script generates a high-order
(N=5000-tap) Kaiser-windowed bandpass per band; the firmware's 501-tap
runtime filters are derived from the same band-edge layout but with a
shorter impulse response that fits in PSRAM.

## Files

| File | Purpose |
|------|---------|
| `Senior_Project_test_script6.m` | Designs the 32-band bandpass FIR bank, plots each filter's frequency response, and writes the coefficients to `matrix.txt`. |
| `wave4.wav` | Test signal the script reads on startup (resampled to 48 kHz). |
| `matrix.txt` | Output coefficient matrix — one block per band, each block prefixed with `Filter <i>: fmin, fmax, fcenter` and followed by 5001 `%.16f` taps. |

## Inputs / Outputs

```
wave4.wav  ──►  Senior_Project_test_script6.m  ──►  matrix.txt
                                                ──►  interactive plots (paused per band)
```

The script also plays the original signal and each band-filtered output
through `sound()` — press a key to advance through bands.

## How this feeds the firmware

```
matrix.txt  ──►  pythondemo/gens/generate_filter_bin.py  ──►  filters.bin
                                                          ──►  /sdcard/filters.bin (read at boot)
```

The host-side Python script in `pythondemo/demo.py --export-filter-matrix`
can also generate a (different, 501-tap) coefficient text file using the same
band edges as the MATLAB script — the two outputs are *not* interchangeable
because the tap counts differ. `pythondemo/gens/generate_filter_bin.py`
reads the 5001-tap MATLAB output and downsamples it for the firmware.

## Band-edge definition

The frequency layout is set by `Ff3` on line 18:

```
[1 30 69 85 105 130 160 197 243 300 370 456 562 693 854 1052 1296
 1596 1967 2424 2986 3680 4534 5586 6883 8479 9000 10445 11273
 15000 15000 20000 20000 25000]
```

Each band spans `[Ff3(k), Ff3(k+1)]`, with the band center at the midpoint.
The same edges (with minor rounding) are mirrored in
[`pythondemo/demo.py`](../pythondemo/demo.py) as `FILTER_BANK_MIN_FREQS` /
`FILTER_BANK_MAX_FREQS`, and in
[`USFMH/main/filter_bank_coeffs.h`](../USFMH/main/filter_bank_coeffs.h).
If you change the layout, all three places need updating before rebuilding
`filters.bin`.

## Running the script

```matlab
cd MATLAB
Senior_Project_test_script6
```

You'll need MATLAB with the **Signal Processing Toolbox** for `fir1`,
`kaiser`, `freqz`, `awgn`, and `resample`.
