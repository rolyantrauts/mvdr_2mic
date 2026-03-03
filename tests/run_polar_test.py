import os
import time
import socket
import subprocess
import numpy as np
import scipy.io.wavfile as wav
import matplotlib.pyplot as plt

SOCKET_PATH = "/tmp/mvdr_ipc.sock"
TEST_FILE = "test_tone_0deg.wav"  # The fixed sound source at 0 degrees

def send_ipc_command(cmd):
    """Sends a string command to the C++ MVDR engine."""
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        sock.sendto(cmd.encode('utf-8'), SOCKET_PATH)
        sock.close()
        print(f"[IPC] Sent: {cmd}")
    except Exception as e:
        print(f"[IPC Error] Could not send to {SOCKET_PATH}. Is MVDR running? ({e})")

def calculate_rms(wav_path):
    """Reads a WAV file and calculates the RMS energy in dBFS."""
    try:
        rate, data = wav.read(wav_path)
        # Always measure only the raw left microphone for the baseline input
        if len(data.shape) > 1:
            data = data[:, 0] 
            
        data = data.astype(np.float32) / 32768.0 
        rms = np.sqrt(np.mean(data**2))
        return 20.0 * np.log10(rms + 1e-9)
    except Exception as e:
        print(f"Error reading {wav_path}: {e}")
        return -100.0

def main():
    print("=== MVDR Beam Sweeping Polar Plot Generator ===")
    
    if not os.path.exists(TEST_FILE):
        print(f"[!] Error: {TEST_FILE} not found. Please generate it first.")
        return

    # Measure the baseline input volume of the 0-degree file once
    in_db = calculate_rms(TEST_FILE)
    print(f"Baseline Input Energy (0° Source): {in_db:.2f} dBFS\n")

    angles = range(0, 181, 5)
    attenuations_db = []

    for beam_angle in angles:
        result_file = f"result_beam_at_{beam_angle}deg.wav"
        
        print(f"--- Sweeping Beam to: {beam_angle}° ---")
        
        # 1. Steer the MVDR beam to the new angle
        send_ipc_command(f"SET {beam_angle}")
        time.sleep(0.5) # Give the C++ engine time to recalculate the steering vector
        
        # 2. Record the MVDR output in the background
        record_cmd = ["arecord", "-D", "plughw:0,1,1", "-f", "S16_LE", "-r", "16000", "-c", "1", result_file]
        recorder = subprocess.Popen(record_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        # 3. Play the fixed 0-degree test tone
        play_cmd = ["aplay", "-D", "plughw:0,0,0", TEST_FILE]
        subprocess.run(play_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        # 4. Stop Recording
        recorder.terminate()
        recorder.wait()
        
        # 5. Calculate attenuation for this specific beam angle
        out_db = calculate_rms(result_file)
        
        drop_db = in_db - out_db
        if drop_db < 0: drop_db = 0 
        
        print(f"Output: {out_db:.2f} dBFS | 0° Source Attenuated by: {drop_db:.2f} dB\n")
        attenuations_db.append(drop_db)

    # --- Generate the Polar Plot ---
    print("Generating Beam-Sweep Polar Plot...")
    
    theta = np.radians(list(angles))
    r = list(attenuations_db)
    
    # Mirror data to create a full 360 circle
    theta_full = np.concatenate((theta, theta + np.pi))
    r_full = np.concatenate((r, r))

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1) 
    
    ax.plot(theta_full, r_full, color='purple', linewidth=2)
    ax.fill(theta_full, r_full, color='purple', alpha=0.2)
    
    max_att = max(attenuations_db) if attenuations_db and max(attenuations_db) > 10 else 10
    ax.set_rmax(max_att + 2)
    
    ax.set_title(f"MVDR Spatial Rejection Profile\n(Fixed 0° Source, Sweeping Beam Angle)", va='bottom')
    plt.savefig("polar_plot_beam_sweep.png", dpi=300)
    print("Saved graph to: polar_plot_beam_sweep.png")

    # Return the beam to a neutral state when finished
    send_ipc_command("RESET")

if __name__ == "__main__":
    main()
