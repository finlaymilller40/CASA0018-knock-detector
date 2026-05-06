
# Data Collection Protocol

## Equipment
- Arduino Nano 33 BLE Sense (PDM microphone, 16 kHz)
- Edge Impulse data acquisition (web-based, USB-connected)

## Knock samples (General Knock class)

A 36-cell systematic grid was used:

- 3 surfaces: internal door (wood), front door (mixed wood and glass), back door (glass window)
- 4 background conditions: quiet, close noise (0.3m), mid noise (2m), far noise (5m)
- 3 force levels: soft, medium, hard

Each cell was populated with 11 samples, randomly split 8 train / 3 test.

Total: 36 cells × 11 samples = 396 knock samples (~288 seconds of audio).

## Background noise samples

Confusers deliberately included:
- Footsteps (trainers, slippers; concrete, wood, carpet, stairs)
- Door closures and cupboard closures
- Keys being placed down
- Items being dropped
- TV and music in background
- Conversational speech

Total: ~468 seconds of audio across the negative class.

## Sampling parameters
- Sample rate: 16 kHz
- Window length: 1 second
- Train/test split: 73/27

## Reproducing this dataset
1. Set up Edge Impulse project with two labels: General Knock, Background Noise
2. Connect Arduino Nano 33 BLE Sense via USB
3. For each cell, record knocks at the specified surface, background condition, and force level
4. Use Edge Impulse's automatic segmentation at 1-second windows
5. Manually verify each sample's label after collection
