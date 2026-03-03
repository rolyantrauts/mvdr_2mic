import argparse
import numpy as np
import scipy.io.wavfile as wav
from scipy.signal import chirp

def fractional_delay_phase_shift(signal, delay_sec, sample_rate):
    """Applies a mathematically perfect sub-sample delay using FFT phase shifting."""
    N = len(signal)
    
    # Transform to frequency domain
    X = np.fft.rfft(signal)
    freqs = np.fft.rfftfreq(N, d=1.0/sample_rate)
    
    # Apply phase shift: e^(-j * 2 * pi * f * delay)
    phase_shift = np.exp(-1j * 2 * np.pi * freqs * delay_sec)
    X_shifted = X * phase_shift
    
    # Transform back to time domain
    return np.fft.irfft(X_shifted, n=N)

def main():
    parser = argparse.ArgumentParser(description="Generate 2-channel MVDR test files (Voice Band Sweet Spot).")
    parser.add_argument("--angle", type=float, default=0.0, help="Simulated source angle in degrees (0-180). 90 is broadside.")
    parser.add_argument("--mic-dist", type=float, default=0.058, help="Distance between mics in meters.")
    parser.add_argument("--duration", type=float, default=3.0, help="Duration of the test tone in seconds.")
    args = parser.parse_args()

    sample_rate = 16000
    speed_of_sound = 343.0

    print(f"=== MVDR Voice-Band Sweet Spot Generator ===")
    print(f"Simulating target at: {args.angle}°")
    
    # Calculate the hardware aliasing limit
    alias_freq = speed_of_sound / (2.0 * args.mic_dist)
    print(f"Hardware Aliasing Limit: {alias_freq:.1f} Hz")
    
    # 1. Generate bounded Voice Chirp (300 Hz to just below Aliasing Limit)
    f_start = 300.0
    f_end = 2950.0
    print(f"Generating chirp from {f_start} Hz to {f_end} Hz...")
    
    t = np.linspace(0, args.duration, int(sample_rate * args.duration), endpoint=False)
    base_signal = chirp(t, f0=f_start, f1=f_end, t1=args.duration, method='logarithmic')
    
    # Soft fade-in and fade-out to prevent speaker pops
    fade_len = int(sample_rate * 0.05) # 50ms fade
    window = np.hanning(fade_len * 2)
    base_signal[:fade_len] *= window[:fade_len]
    base_signal[-fade_len:] *= window[fade_len:]

    # 2. Calculate the acoustic delay (Tau) based on the C++ geometry
    theta_rad = np.radians(args.angle)
    tau_sec = (args.mic_dist * np.cos(theta_rad)) / speed_of_sound
    
    delay_samples = tau_sec * sample_rate
    print(f"Calculated acoustic delay: {tau_sec*1000:.3f} ms ({delay_samples:.2f} samples)")

    # 3. Apply the delay
    ch1 = base_signal
    ch2 = fractional_delay_phase_shift(base_signal, tau_sec, sample_rate)

    # 4. Format and Export to 16-bit PCM WAV
    ch1_int16 = np.int16(ch1 * 32767 * 0.5)
    ch2_int16 = np.int16(ch2 * 32767 * 0.5)
    
    stereo_interleaved = np.vstack((ch1_int16, ch2_int16)).T
    
    filename = f"test_tone_{int(args.angle)}deg.wav"
    wav.write(filename, sample_rate, stereo_interleaved)
    print(f"Successfully saved: {filename}\n")

if __name__ == "__main__":
    main()
