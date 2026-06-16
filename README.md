# Clapping Detector Using FFT and Envelope Techniques

A clap detection system based on an **ESP32-C3** and an inexpensive **I2S microphone**.

## Features

- Uses a combination of **FFT (Fast Fourier Transform)** and **envelope analysis** for clap detection.
- Runs on low-cost hardware.
- Performs reasonably well in typical environments.

## Notes

The current implementation already achieves good results. Reliability can be increased significantly by using it as a building block for more complex patterns, such as:

- Detecting **two claps in quick succession**
- Recognizing predefined clap sequences

Requiring multiple consecutive clap events can greatly reduce false positives and improve overall detection accuracy.
