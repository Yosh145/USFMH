# Archived debug dumps

These are serial-capture diagnostic files produced by `pythondemo/demo.py --dump`
during development. Each `dump_sample_<label>.txt` contains, for one run:

- Original / filtered audio statistics
- Top FFT peaks for both signals
- Per-band filter analysis (peak dB, gain at center, energy ratio)
- Decimated FFT data over the audible range
- The first 200 time-domain samples of each signal

## Filename convention

| `<label>` | Meaning                                                |
|-----------|--------------------------------------------------------|
| `<N>`     | `--sample N` debug mode (one band active, index `N`)   |
| `a`       | `--sample all` debug mode (all DSP bands active)       |
| `hearing` | Real interactive hearing test                          |

## How to regenerate

```bash
./run.sh --port /dev/ttyACM0 --sample 11 --dump
# or, all bands active:
./run.sh --port /dev/ttyACM0 --sample all --dump
```

`demo.py` writes new dumps to `dumps/` at the repo root; copy the output here
if you want to keep it as part of the archive.

## Why these are kept

Reference data for the next contributor — useful for verifying that a rebuilt
toolchain still produces the same per-band response shape, and for
sanity-checking filter changes before flashing.
