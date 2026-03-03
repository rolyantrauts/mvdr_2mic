```
# MVDR 2-Mic DSP Engine (Raspberry Pi Zero 2 W)

This repository contains a highly optimized, ALSA-native C++ Minimum Variance Distortionless Response (MVDR) beamformer designed specifically for the 2-microphone ReSpeaker HAT on a Raspberry Pi Zero 2 W. 

It is designed to be the first stage in a modular Voice Activity Detection (VAD) and Wakeword pipeline, using UNIX sockets (IPC) to dynamically steer acoustic nulls and track targets in real-time.

---

## 🎧 The Physics & Limitations of a 2-Mic Array

While this engine is capable of aggressive noise suppression, it is strictly bound by the acoustic physics of using exactly two microphones.



* **Degrees of Freedom (DOF):** An array with *N* microphones has *N-1* degrees of freedom. This 2-mic array has exactly **1 DOF**.
This means the covariance matrix can only deploy **one** spatial null at a time. It will hunt and mathematically crush the single loudest point-source noise in the room (e.g., a TV), but it cannot cancel a TV *and* a washing machine simultaneously.
* **The 58mm Spacing:** The ReSpeaker HAT has a 58mm distance between microphones. This is highly usable for the human voice band, but it has hard physical limits.
At low frequencies (<300 Hz), the wavelengths are too massive for the array to resolve accurate phase differences.
At high frequencies (~2957 Hz), the array hits its "Spatial Aliasing" ceiling, where wavelengths become shorter than the mic gap, causing directional confusion.
* **Front/Back Ambiguity:** A linear 2-mic array cannot tell if a sound is in front of it or behind it.
If you generate a 360-degree polar map (using the tools below), the resulting graph will mirror perfectly from front to back.

## 🎛️ White Noise Gain (WNG) & Diagonal Loading

One of the biggest challenges with MVDR on cheap MEMS microphones is White Noise Gain.
When the MVDR math attempts to cancel out highly correlated noise (like room reverberation) or resolve low frequencies, the matrix inversions become unstable and severely amplify the electrical hiss of the microphones.

To counter this, this engine utilizes **Dynamic Trace Loading (Diagonal Loading)**.
By injecting artificial mathematical noise into the covariance matrix, we force the algorithm to remain stable. 
* **The Compromise:** Diagonal loading acts as a governor on the math.
We intentionally trade away a few decibels of maximum theoretical point-source attenuation to guarantee the audio output remains smooth, natural, and free of harsh static clicks.
You can tune this live using the `--diag-load` parameter.

## 🧠 The VAD Architecture & The Covariance Matrix

This engine is designed to be controlled by a downstream Neural Network (like a TFLite Wakeword runner) via IPC commands.
**An external VAD (Voice Activity Detection) signal is mandatory for adaptive MVDR to function in the real world.**



If an MVDR array continues to adapt its covariance matrix while you are speaking, it will see your voice as a massive energy spike, classify it as "noise," and mathematically attempt to delete your voice.
To prevent this **Target Self-Cancellation**, the engine relies on a strict state machine:

1. **VAD 0 (Silence/Background):** The covariance matrix is actively learning.
It acts like a live camera, tracking the room, finding the humming fridge or background TV, and deploying deep nulls to silence them.
2. **VAD 1 (Speech Detected):** The moment the IPC socket receives `VAD 1`, the covariance matrix **freezes**. It stops learning.
It holds the nulls perfectly on the TV and fridge, but becomes blind to your voice. This clears an acoustic tunnel, allowing your speech to pass through the array completely undistorted while the background noise remains crushed.

## 🎯 GCC-PHAT Target Tracking & Steering Mismatch

While the covariance matrix handles the nulls, the **GCC-PHAT** algorithm tracks the target angle (DOA). 

Because 2-mic arrays have incredibly wide, "fat" listening lobes, they are highly forgiving of Steering Mismatch.
Even if the GCC-PHAT calculation is a few degrees off due to the 1 DOF limitations, your voice will still easily fall within the protected "Distortionless" passband.
GCC-PHAT provides sub-sample phase accuracy, ensuring the target angle is tracked reliably enough to protect your voice while the matrix attacks the background noise.

---

## 🛠️ Diagnostic Tools (Polar Mapping)

To visualize the spatial rejection profile of the array, this repository includes Python scripts to simulate acoustic fractional delays and measure the engine's attenuation.

The scripts have been configured to sweep the engine's beam in **361 steps (5-degree increments)** against a fixed 0-degree test source to generate high-resolution polar plots.



### 1. Generate the Test Audio
First, generate the simulated voice-band chirp that acts as our fixed 0-degree noise source:

python3 generate_test_wav.py --angle 0 --duration 3.0

2. Run the MVDR Engine in Debug Mode
We use the ALSA snd-aloop virtual cables to route audio entirely inside the Pi without needing physical speakers. Start the engine in a terminal:

./mvdr_engine --mic plughw:0,1,0 --out plughw:0,0,1 --debug --diag-load 0.001
3. Generate the Polar Plot
In a second terminal, run the automation script. It will send IPC SET <angle> commands to the engine, stream the audio, measure the RMS decibel drop, and output a polar_plot_beam_sweep.png graph:

python3 run_polar_test.py
⚠️ Current Status & Disclaimer
The downstream Wakeword dataset is currently being trained.
While the core C++ MVDR engine is complete, phase-accurate, and ALSA clock-drift proof, the accompanying C++ TFLite Wakeword runner (which will supply the required VAD 1/VAD 0 signals) has not yet been published.

Until the Wakeword runner is integrated, this repository functions primarily as an acoustic laboratory testing tool.
Unless you can provide an external IPC VAD signal to /tmp/mvdr_ipc.sock, the engine will default to either a fixed Delay-and-Sum beamformer or will attempt to adaptively null all audio, including target speech.
```
