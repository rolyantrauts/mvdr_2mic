# mvdr_2mic
wakeword driven MVDR beamformer


```
# 1. Install dependencies
sudo apt-get update
sudo apt-get install -y build-essential libasound2-dev git python3

# 2. Enable the virtual audio loopback
sudo modprobe snd-aloop
echo "snd-aloop" | sudo tee -a /etc/modules

# 3. Clone and build
git clone https://github.com/rolyantrauts/mvdr_2mic.git
mkdir mvdr_2mic/build
cd mvdr_2mic/build
cmake ..
make -j 3

# 4. Run the engine (Wait for Python IPC commands)
./mvdr_engine --mic hw:1,0 --out plughw:Loopback,0,0 --debug
```

```
:~/mvdr_2mic/build $ ./mvdr_engine --help
=== MVDR PFFFT Standalone Engine (IPC Controlled) ===
Usage: ./mvdr_engine [OPTIONS]

Options:
  -h, --help           Show this help message and exit.
  --mic <hw:X,Y>       Set ALSA capture device (Default: hw:1,0).
  --out <hw:X,Y>       Set ALSA playback device for processed audio (e.g., plughw:Loopback,0,0).
  --in <file.wav>      Bypass ALSA mic and process a raw 16kHz 2-channel WAV file directly.
  --mic-spacing <m>    Distance between microphones in meters (Default: 0.058).
                       [!] MIN: ~0.03m (30mm) to overcome hardware phase variance (WNG limit).
                       [!] MAX: ~0.042m (42mm) for pure voice band without spatial aliasing.
                       (Note: The default 0.058m leverages an internal 2.9kHz anti-aliasing guard).
  --doa-history <n>    Number of 16ms frames to keep for the LOCK average (Default: 100 = 1.6s).
  --measure-clock      Run a 10-second hardware clock drift test and exit.
  --debug              Enable live terminal metering, CSV telemetry, and raw WAV dumping.

IPC Control:
  This engine listens on UNIX socket: /tmp/mvdr_ipc.sock
  Commands:
    'VAD 1' : Start steering beam (Voice detected)
    'VAD 0' : Stop steering, learn background noise
    'LOCK'  : Analyze history buffer, lock beam direction
    'RESET' : Unlock beam, return to background noise mode
    'SET X' : Manually force beam to X degrees (e.g., 'SET 45')
```
