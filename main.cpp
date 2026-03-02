#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <csignal>
#include <iomanip>
#include <algorithm> 
#include <unistd.h> 
#include <sys/socket.h> 
#include <sys/un.h>     
#include <fcntl.h>
#include <chrono>
#include <alsa/asoundlib.h>
#include "pffft.h"
#include "mvdr_neon.h"

#define SAMPLE_RATE 16000
#define FRAME_SIZE 512
#define HOP_SIZE 256
#define MIC_DIST 0.058f
#define SOCKET_PATH "/tmp/mvdr_ipc.sock"

volatile bool keep_running = true;
void intHandler(int) { keep_running = false; }

struct AppConfig {
    std::string device_in = "hw:1,0";
    std::string device_out = "";
    std::string file_in = "";
    bool debug_mode = false;
    bool measure_clock = false;
    float static_doa = 90.0f;
    float mic_dist = 0.058f; 
    int doa_history_frames = 100; // Default: 100 frames * 16ms = 1.6 seconds
};

void print_help(const char* prog_name) {
    std::cout << "=== MVDR PFFFT Standalone Engine (IPC Controlled) ===\n"
              << "Usage: " << prog_name << " [OPTIONS]\n\n"
              << "Options:\n"
              << "  -h, --help           Show this help message and exit.\n"
              << "  --mic <hw:X,Y>       Set ALSA capture device (Default: hw:1,0).\n"
              << "  --out <hw:X,Y>       Set ALSA playback device for processed audio (e.g., plughw:Loopback,0,0).\n"
              << "  --in <file.wav>      Bypass ALSA mic and process a raw 16kHz 2-channel WAV file directly.\n"
              << "  --mic-spacing <m>    Distance between microphones in meters (Default: 0.058).\n"
              << "                       [!] MIN: ~0.03m (30mm) to overcome hardware phase variance (WNG limit).\n"
              << "                       [!] MAX: ~0.042m (42mm) for pure voice band without spatial aliasing.\n"
              << "                       (Note: The default 0.058m leverages an internal 2.9kHz anti-aliasing guard).\n"
              << "  --doa-history <n>    Number of 16ms frames to keep for the LOCK average (Default: 100 = 1.6s).\n"
              << "  --measure-clock      Run a 10-second hardware clock drift test and exit.\n"
              << "  --debug              Enable live terminal metering, CSV telemetry, and raw WAV dumping.\n\n"
              << "IPC Control:\n"
              << "  This engine listens on UNIX socket: " << SOCKET_PATH << "\n"
              << "  Commands:\n"
              << "    'VAD 1' : Start steering beam (Voice detected)\n"
              << "    'VAD 0' : Stop steering, learn background noise\n"
              << "    'LOCK'  : Analyze history buffer, lock beam direction\n"
              << "    'RESET' : Unlock beam, return to background noise mode\n"
              << "    'SET X' : Manually force beam to X degrees (e.g., 'SET 45')\n\n";
}

struct FrameStat { float energy; float angle; };

class DoaHistory {
    std::vector<FrameStat> buffer;
    size_t head = 0;
public:
    DoaHistory(size_t size = 100) { buffer.resize(size, {0.0f, 90.0f}); }
    void push(float energy, float angle) {
        buffer[head] = {energy, angle};
        head = (head + 1) % buffer.size();
    }
    float get_best_angle() {
        float sum_sin = 0.0f, sum_cos = 0.0f, total_weight = 0.0f;
        for (const auto& f : buffer) {
            if (f.energy < 1e-8f) continue;
            float rad = f.angle * PI / 180.0f;
            sum_sin += sinf(rad) * f.energy;
            sum_cos += cosf(rad) * f.energy;
            total_weight += f.energy;
        }
        if (total_weight < 1e-8f) return 90.0f;
        float avg_deg = atan2f(sum_sin, sum_cos) * 180.0f / PI;
        if (avg_deg < 0.0f) avg_deg = 0.0f;
        if (avg_deg > 180.0f) avg_deg = 180.0f;
        return avg_deg;
    }
    void dump_to_csv(const std::string& filename, int event_id, float final_angle) {
        std::ofstream csv(filename, std::ios::app);
        if (!csv.is_open()) return;
        csv << "--- LOCK EVENT " << event_id << " | Calculated Optimal Angle: " << final_angle << " ---\n";
        csv << "Chronological_Frame,Voice_Energy,Raw_Angle_Deg\n";
        for (size_t i = 0; i < buffer.size(); ++i) {
            size_t idx = (head + i) % buffer.size();
            csv << i << "," << buffer[idx].energy << "," << buffer[idx].angle << "\n";
        }
        csv << "\n";
        csv.close();
    }
};

snd_pcm_t* setup_alsa(const std::string& dev, snd_pcm_stream_t stream, int channels, AppConfig& cfg) {
    snd_pcm_t* handle;
    if (snd_pcm_open(&handle, dev.c_str(), stream, 0) < 0) return nullptr;
    snd_pcm_hw_params_t* params;
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(handle, params);
    snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(handle, params, channels);
    unsigned int rate = SAMPLE_RATE;
    snd_pcm_hw_params_set_rate_near(handle, params, &rate, 0);
    snd_pcm_uframes_t period_size = HOP_SIZE;
    snd_pcm_hw_params_set_period_size_near(handle, params, &period_size, 0);
    snd_pcm_uframes_t buffer_size = (stream == SND_PCM_STREAM_PLAYBACK) ? (HOP_SIZE * 32) : (HOP_SIZE * 4);
    snd_pcm_hw_params_set_buffer_size_near(handle, params, &buffer_size);
    if (snd_pcm_hw_params(handle, params) < 0) return nullptr;
    return handle;
}

void run_clock_drift_test(const std::string& capture_dev) {
    std::cout << "\n=== HARDWARE CLOCK DRIFT TEST ===\n";
    std::cout << "Target: " << capture_dev << " @ 16000Hz\n";
    AppConfig dummy_cfg;
    snd_pcm_t* pcm_in = setup_alsa(capture_dev, SND_PCM_STREAM_CAPTURE, 2, dummy_cfg);
    if (!pcm_in) { std::cerr << "Failed to open device for testing.\n"; return; }
    std::vector<int16_t> buffer(HOP_SIZE * 2);
    int total_frames_to_read = 160000;
    int frames_read = 0;
    std::cout << "Reading exactly 160,000 frames (Should take 10.000 seconds)...\n";
    auto start_time = std::chrono::high_resolution_clock::now();
    while (frames_read < total_frames_to_read) {
        int err = snd_pcm_readi(pcm_in, buffer.data(), HOP_SIZE);
        if (err > 0) frames_read += err;
        else if (err < 0) snd_pcm_recover(pcm_in, err, 1);
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    double actual_duration = elapsed.count();
    double actual_sample_rate = total_frames_to_read / actual_duration;
    std::cout << "\n--- TEST RESULTS ---\n";
    std::cout << "Time elapsed: " << std::fixed << std::setprecision(4) << actual_duration << " seconds\n";
    std::cout << "Actual hardware sample rate: " << actual_sample_rate << " Hz\n";
    std::cout << "Drift: " << (actual_sample_rate - 16000.0) << " samples per second.\n";
    snd_pcm_close(pcm_in);
}

void write_wav_header(std::ofstream& file, uint32_t data_size, int channels, int sample_rate) {
    uint32_t file_size = data_size + 36;
    uint32_t byte_rate = sample_rate * channels * 2;
    uint16_t block_align = channels * 2;
    uint16_t bits_per_sample = 16;
    uint16_t audio_format = 1;
    file.seekp(0);
    file.write("RIFF", 4); file.write((char*)&file_size, 4); file.write("WAVE", 4);
    file.write("fmt ", 4); 
    uint32_t fmt_size = 16; file.write((char*)&fmt_size, 4);
    file.write((char*)&audio_format, 2); file.write((char*)&channels, 2);
    file.write((char*)&sample_rate, 4); file.write((char*)&byte_rate, 4);
    file.write((char*)&block_align, 2); file.write((char*)&bits_per_sample, 2);
    file.write("data", 4); file.write((char*)&data_size, 4);
}

void calculate_levels(const std::vector<float>& buf, float& peak_db, float& rms_db) {
    float peak = 0.0f, sum_sq = 0.0f;
    for (float val : buf) {
        float abs_val = std::abs(val);
        if (abs_val > peak) peak = abs_val;
        sum_sq += val * val;
    }
    float rms = std::sqrt(sum_sq / buf.size());
    peak_db = 20.0f * std::log10(peak + 1e-9f);
    rms_db = 20.0f * std::log10(rms + 1e-9f);
}

void build_steering_vector(float* sv1, float* sv2, float angle_deg, float mic_dist) {
    float theta = angle_deg * PI / 180.0f;
    float tau = (mic_dist * cos(theta)) / 343.0f; 
    for (int k = 0; k < NUM_BINS; ++k) {
        float freq = (float)k * SAMPLE_RATE / FRAME_SIZE;
        float omega_tau = 2.0f * PI * freq * tau;
        sv1[2*k] = 1.0f; sv1[2*k+1] = 0.0f;
        sv2[2*k] = cosf(-omega_tau); sv2[2*k+1] = sinf(-omega_tau);
    }
}

int setup_ipc_socket() {
    int sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    unlink(SOCKET_PATH);
    struct sockaddr_un servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sun_family = AF_UNIX;
    strncpy(servaddr.sun_path, SOCKET_PATH, sizeof(servaddr.sun_path) - 1);
    bind(sock, (const struct sockaddr *)&servaddr, sizeof(servaddr));
    fcntl(sock, F_SETFL, fcntl(sock, F_GETFL, 0) | O_NONBLOCK);
    return sock;
}

int main(int argc, char** argv) {
    signal(SIGINT, intHandler);
    AppConfig cfg;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") { print_help(argv[0]); return 0; }
        else if (arg == "--debug") cfg.debug_mode = true;
        else if (arg == "--measure-clock") cfg.measure_clock = true;
        else if (arg == "--in" && i + 1 < argc) cfg.file_in = argv[++i];
        else if (arg == "--out" && i + 1 < argc) cfg.device_out = argv[++i];
        else if (arg == "--mic" && i + 1 < argc) cfg.device_in = argv[++i];
        else if (arg == "--mic-spacing" && i + 1 < argc) cfg.mic_dist = std::stof(argv[++i]);
        else if (arg == "--doa-history" && i + 1 < argc) cfg.doa_history_frames = std::stoi(argv[++i]);
    }

    if (cfg.measure_clock) { run_clock_drift_test(cfg.device_in); return 0; }

    std::cout << "=== MVDR PFFFT Standalone Engine (IPC Controlled) ===\n";
    std::cout << "Mic Spacing: " << cfg.mic_dist << "m | History Buffer: " << cfg.doa_history_frames << " frames\n";

    snd_pcm_t *pcm_in = nullptr, *pcm_out = nullptr;
    std::ifstream wav_in;

    if (!cfg.file_in.empty()) {
        wav_in.open(cfg.file_in, std::ios::binary);
        if (!wav_in.is_open()) { std::cerr << "[ERROR] Cannot open " << cfg.file_in << "\n"; return 1; }
        wav_in.seekg(44); 
    } else {
        pcm_in = setup_alsa(cfg.device_in, SND_PCM_STREAM_CAPTURE, 2, cfg);
        if (!pcm_in) return 1;
    }

    if (!cfg.device_out.empty()) {
        pcm_out = setup_alsa(cfg.device_out, SND_PCM_STREAM_PLAYBACK, 1, cfg);
        if (!pcm_out) return 1;
    }

    int ipc_sock = setup_ipc_socket();
    DoaHistory history(cfg.doa_history_frames); 

    std::ofstream debug_csv, debug_wav;
    uint32_t debug_wav_bytes = 0;
    
    if (cfg.debug_mode) {
        std::ofstream clear_locks("debug_doa_lock_history.csv", std::ios::trunc); clear_locks.close();
        debug_csv.open("debug_levels.csv");
        debug_csv << "Frame,RMS_L_dB,Ext_VAD,Raw_DOA,Steered_DOA,Mode\n";
        debug_wav.open("debug_raw_mics.wav", std::ios::binary);
        write_wav_header(debug_wav, 0, 2, SAMPLE_RATE); 
        std::cout << "[DEBUG MODE] Logging active.\n";
    }

    PFFFT_Setup *setup = pffft_new_setup(FRAME_SIZE, PFFFT_REAL);
    float *t_ch1 = (float*)pffft_aligned_malloc(FRAME_SIZE * sizeof(float));
    float *t_ch2 = (float*)pffft_aligned_malloc(FRAME_SIZE * sizeof(float));
    float *t_out = (float*)pffft_aligned_malloc(FRAME_SIZE * sizeof(float));
    float *f_ch1 = (float*)pffft_aligned_malloc(FRAME_SIZE * sizeof(float));
    float *f_ch2 = (float*)pffft_aligned_malloc(FRAME_SIZE * sizeof(float));
    float *f_out = (float*)pffft_aligned_malloc(FRAME_SIZE * sizeof(float));
    float *doa_f = (float*)pffft_aligned_malloc(FRAME_SIZE * sizeof(float));
    float *doa_t = (float*)pffft_aligned_malloc(FRAME_SIZE * sizeof(float));
    float *work  = (float*)pffft_aligned_malloc(FRAME_SIZE * sizeof(float));
    float *sv1   = (float*)pffft_aligned_malloc(NUM_BINS * 2 * sizeof(float));
    float *sv2   = (float*)pffft_aligned_malloc(NUM_BINS * 2 * sizeof(float));

    std::vector<cpx> X1(NUM_BINS), X2(NUM_BINS), Y(NUM_BINS), phat(NUM_BINS);
    std::vector<float> ring1(FRAME_SIZE, 0.0f), ring2(FRAME_SIZE, 0.0f), overlap(FRAME_SIZE, 0.0f);
    std::vector<float> window(FRAME_SIZE);
    for (int i = 0; i < FRAME_SIZE; ++i) window[i] = sinf(PI * i / FRAME_SIZE);

    MvdrState st; mvdr_init(&st);
    build_steering_vector(sv1, sv2, cfg.static_doa, cfg.mic_dist);

    std::vector<int16_t> in_buf(HOP_SIZE * 2);
    std::vector<int16_t> out_buf(HOP_SIZE);
    std::vector<float> block_L(HOP_SIZE), block_R(HOP_SIZE);

    int frame_count = 0, lock_counter = 0;
    float steered_angle = 90.0f;
    bool ext_vad_active = false, is_locked = false;
    std::string mode_str = "AUTO";
    float period_max_peak = -100.0f;

    float dc_state_l = 0.0f;
    float dc_state_r = 0.0f;

    std::cout << "\nRunning... Waiting for IPC commands. Press Ctrl+C to stop.\n";

    while (keep_running) {
        char ipc_buf[128]; int n;
        while ((n = recvfrom(ipc_sock, ipc_buf, sizeof(ipc_buf)-1, 0, NULL, NULL)) > 0) {
            ipc_buf[n] = '\0'; std::string cmd(ipc_buf);
            if (cmd.find("VAD 1") == 0) ext_vad_active = true;
            else if (cmd.find("VAD 0") == 0) ext_vad_active = false;
            else if (cmd.find("LOCK") == 0 && !is_locked) {
                is_locked = true; mode_str = "LOCK";
                steered_angle = history.get_best_angle();
                build_steering_vector(sv1, sv2, steered_angle, cfg.mic_dist);
                std::cout << "\n\n>>> [LOCK] Optimal Acoustic Center: " << std::fixed << std::setprecision(1) << steered_angle << "°\n\n";
                if (cfg.debug_mode) {
                    lock_counter++;
                    history.dump_to_csv("debug_doa_lock_history.csv", lock_counter, steered_angle);
                }
            }
            else if (cmd.find("RESET") == 0) { is_locked = false; ext_vad_active = false; mode_str = "AUTO"; }
            else if (cmd.find("SET") == 0) {
                try { is_locked = true; mode_str = "MANUAL"; steered_angle = std::stof(cmd.substr(4)); build_steering_vector(sv1, sv2, steered_angle, cfg.mic_dist); } catch (...) {}
            }
        }

        if (!cfg.file_in.empty()) {
            wav_in.read((char*)in_buf.data(), HOP_SIZE * 4);
            if (wav_in.gcount() == 0) break; 
        } else {
            int err = snd_pcm_readi(pcm_in, in_buf.data(), HOP_SIZE);
            if (err < 0) { snd_pcm_recover(pcm_in, err, 1); continue; }
        }

        if (cfg.debug_mode && debug_wav.is_open()) {
            debug_wav.write((char*)in_buf.data(), HOP_SIZE * 4);
            debug_wav_bytes += (HOP_SIZE * 4);
        }

        for (int i = 0; i < HOP_SIZE; ++i) {
            float raw_l = in_buf[i*2] / 32768.0f;
            float raw_r = in_buf[i*2+1] / 32768.0f;
            
            dc_state_l = dc_state_l + 0.001f * (raw_l - dc_state_l);
            dc_state_r = dc_state_r + 0.001f * (raw_r - dc_state_r);
            
            block_L[i] = raw_l - dc_state_l;
            block_R[i] = raw_r - dc_state_r;
        }

        std::copy(ring1.begin() + HOP_SIZE, ring1.end(), ring1.begin());
        std::copy(ring2.begin() + HOP_SIZE, ring2.end(), ring2.begin());
        for (int i = 0; i < HOP_SIZE; ++i) {
            ring1[FRAME_SIZE - HOP_SIZE + i] = block_L[i];
            ring2[FRAME_SIZE - HOP_SIZE + i] = block_R[i];
        }

        apply_window_neon(ring1.data(), window.data(), t_ch1, FRAME_SIZE);
        apply_window_neon(ring2.data(), window.data(), t_ch2, FRAME_SIZE);

        pffft_transform_ordered(setup, t_ch1, f_ch1, work, PFFFT_FORWARD);
        pffft_transform_ordered(setup, t_ch2, f_ch2, work, PFFFT_FORWARD);

        unpack_pffft_to_cpx(f_ch1, X1.data(), FRAME_SIZE);
        unpack_pffft_to_cpx(f_ch2, X2.data(), FRAME_SIZE);

        float frame_energy = 0.0f;
        for(int k=1; k<NUM_BINS; ++k) frame_energy += std::norm(X1[k]) + std::norm(X2[k]);
        float raw_angle = steered_angle; 

        if (ext_vad_active && !is_locked) {
            calculate_doa_gcc_phat(X1.data(), X2.data(), phat.data(), cfg.mic_dist, SAMPLE_RATE);
            pack_cpx_to_pffft(phat.data(), doa_f, FRAME_SIZE);
            pffft_transform_ordered(setup, doa_f, doa_t, work, PFFFT_BACKWARD);
            raw_angle = extract_angle_from_gcc(doa_t, FRAME_SIZE, cfg.mic_dist, SAMPLE_RATE);
            
            history.push(frame_energy, raw_angle); 
            steered_angle = 0.8f * steered_angle + 0.2f * raw_angle;
            build_steering_vector(sv1, sv2, steered_angle, cfg.mic_dist);
        } else {
            history.push(0.0f, raw_angle); 
        }

        if (cfg.debug_mode) {
            float pk_l, rms_l, pk_r, rms_r;
            calculate_levels(block_L, pk_l, rms_l); calculate_levels(block_R, pk_r, rms_r);
            float current_peak = std::max(pk_l, pk_r);
            if (current_peak > period_max_peak) period_max_peak = current_peak;
            if (debug_csv.is_open()) debug_csv << frame_count << "," << rms_l << "," << (ext_vad_active ? 1 : 0) << "," << raw_angle << "," << steered_angle << "," << mode_str << "\n";

            if (frame_count % 31 == 0) {
                std::string warning_str = "";
                if (period_max_peak > -1.0f) warning_str = " [! CLIP !]";
                else if (period_max_peak < -45.0f) warning_str = " [! LOW !]";

                std::cout << "\r[Mode] " << std::setw(6) << mode_str << " | VAD: " << (ext_vad_active ? "ON " : "OFF") 
                          << " | DOA: " << std::setw(5) << std::setprecision(1) << steered_angle << "°" << warning_str << " \033[K";
                std::cout << std::flush;
                period_max_peak = -100.0f;
            }
        }

        bool update_covariance = (!ext_vad_active && !is_locked);
        process_mvdr_neon(&st, X1.data(), X2.data(), sv1, sv2, Y.data(), NUM_BINS, update_covariance);

        pack_cpx_to_pffft(Y.data(), f_out, FRAME_SIZE);
        pffft_transform_ordered(setup, f_out, t_out, work, PFFFT_BACKWARD);

        apply_window_neon(t_out, window.data(), t_out, FRAME_SIZE);
        overlap_add_neon(overlap.data(), t_out, 1.0f / FRAME_SIZE, FRAME_SIZE);

        for (int i = 0; i < HOP_SIZE; ++i) {
            float val = overlap[i] * 32767.0f;
            out_buf[i] = (int16_t)(std::clamp(val, -32768.0f, 32767.0f));
        }

        std::copy(overlap.begin() + HOP_SIZE, overlap.end(), overlap.begin());
        std::fill(overlap.begin() + FRAME_SIZE - HOP_SIZE, overlap.end(), 0.0f);

        if (pcm_out) {
            int err = snd_pcm_writei(pcm_out, out_buf.data(), HOP_SIZE);
            if (err < 0) {
                err = snd_pcm_recover(pcm_out, err, 1); 
                if (err == 0) snd_pcm_writei(pcm_out, out_buf.data(), HOP_SIZE); 
            }
        }
        frame_count++;
    }

    std::cout << "\nShutting down...\n";
    close(ipc_sock); unlink(SOCKET_PATH);
    if (cfg.debug_mode) {
        if (debug_csv.is_open()) debug_csv.close();
        if (debug_wav.is_open()) { write_wav_header(debug_wav, debug_wav_bytes, 2, SAMPLE_RATE); debug_wav.close(); }
    }
    if (pcm_in) snd_pcm_close(pcm_in);
    if (pcm_out) snd_pcm_close(pcm_out);
    pffft_aligned_free(t_ch1); pffft_aligned_free(t_ch2); pffft_aligned_free(t_out);
    pffft_aligned_free(f_ch1); pffft_aligned_free(f_ch2); pffft_aligned_free(f_out);
    pffft_aligned_free(doa_f); pffft_aligned_free(doa_t); pffft_aligned_free(work);
    pffft_aligned_free(sv1); pffft_aligned_free(sv2); pffft_destroy_setup(setup);
    return 0;
}
