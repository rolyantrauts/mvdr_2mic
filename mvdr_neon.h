#ifndef MVDR_NEON_H
#define MVDR_NEON_H

#include <complex>
#include <cmath>
#include <arm_neon.h>

#define NUM_BINS 257         
#define ALPHA 0.95f          
#define PI 3.14159265358979323846f

typedef std::complex<float> cpx;

struct MvdrState {
    cpx R[NUM_BINS][2][2]; 
};

inline void mvdr_init(MvdrState* st) {
    for (int k = 0; k < NUM_BINS; ++k) {
        st->R[k][0][0] = cpx(1e-4f, 0.0f); 
        st->R[k][0][1] = cpx(0.0f, 0.0f);
        st->R[k][1][0] = cpx(0.0f, 0.0f);
        st->R[k][1][1] = cpx(1e-4f, 0.0f);
    }
}

inline void unpack_pffft_to_cpx(const float* __restrict pffft_out, cpx* __restrict cpx_out, int frame_size) {
    cpx_out[0] = cpx(pffft_out[0], 0.0f);                   
    cpx_out[frame_size / 2] = cpx(pffft_out[1], 0.0f);      
    for (int k = 1; k < frame_size / 2; ++k) {
        cpx_out[k] = cpx(pffft_out[2*k], pffft_out[2*k+1]);
    }
}

inline void pack_cpx_to_pffft(const cpx* __restrict cpx_in, float* __restrict pffft_in, int frame_size) {
    pffft_in[0] = cpx_in[0].real();
    pffft_in[1] = cpx_in[frame_size / 2].real();
    for (int k = 1; k < frame_size / 2; ++k) {
        pffft_in[2*k] = cpx_in[k].real();
        pffft_in[2*k+1] = cpx_in[k].imag();
    }
}

inline void apply_window_neon(const float* __restrict input, const float* __restrict window, float* __restrict output, int size) {
    int i = 0;
    for (; i <= size - 4; i += 4) {
        float32x4_t v_in = vld1q_f32(&input[i]);
        float32x4_t v_win = vld1q_f32(&window[i]);
        vst1q_f32(&output[i], vmulq_f32(v_in, v_win));
    }
    for (; i < size; ++i) output[i] = input[i] * window[i];
}

inline void overlap_add_neon(float* __restrict overlap_buf, const float* __restrict new_time_buf, float scale, int size) {
    int i = 0;
    float32x4_t v_scale = vdupq_n_f32(scale);
    for (; i <= size - 4; i += 4) {
        float32x4_t v_lap = vld1q_f32(&overlap_buf[i]);
        float32x4_t v_new = vld1q_f32(&new_time_buf[i]);
        v_lap = vmlaq_f32(v_lap, v_new, v_scale); 
        vst1q_f32(&overlap_buf[i], v_lap);
    }
    for (; i < size; ++i) overlap_buf[i] += new_time_buf[i] * scale;
}

inline void invert_2x2(cpx R[2][2], cpx invR[2][2]) {
    cpx det = R[0][0] * R[1][1] - R[0][1] * R[1][0];
    float mag = std::abs(det);
    if (mag < 1e-9f) {
        invR[0][0] = 1.0f; invR[0][1] = 0.0f; invR[1][0] = 0.0f; invR[1][1] = 1.0f;
        return;
    }
    cpx invDet = 1.0f / det;
    invR[0][0] =  R[1][1] * invDet;
    invR[0][1] = -R[0][1] * invDet;
    invR[1][0] = -R[1][0] * invDet;
    invR[1][1] =  R[0][0] * invDet;
}

inline float calculate_doa_gcc_phat(const cpx* __restrict X1, 
                                    const cpx* __restrict X2, 
                                    cpx* __restrict phat_out, 
                                    float mic_dist,
                                    int sample_rate) {
    const int MIN_BIN = 10;  
    const int MAX_BIN = 93;  

    for(int k=0; k<NUM_BINS; ++k) {
        if (k >= MIN_BIN && k <= MAX_BIN) {
            cpx cross = X1[k] * std::conj(X2[k]);
            float mag = std::abs(cross);
            phat_out[k] = cross / (mag + 1e-9f); 
        } else {
            phat_out[k] = cpx(0.0f, 0.0f);
        }
    }
    return 0.0f; 
}

inline float extract_angle_from_gcc(const float* __restrict temp_time_out, int frame_size, float mic_dist, int sample_rate) {
    int best_idx = 0;
    float max_val = -1e9f;
    for(int i=0; i<frame_size; ++i) {
        if(temp_time_out[i] > max_val) {
            max_val = temp_time_out[i];
            best_idx = i;
        }
    }
    
    float fractional_delay = (float)best_idx;
    int left_idx = (best_idx - 1 + frame_size) % frame_size;
    int right_idx = (best_idx + 1) % frame_size;
    
    float alpha = temp_time_out[left_idx];
    float beta = temp_time_out[best_idx];
    float gamma = temp_time_out[right_idx];
    
    float denom = alpha - 2.0f * beta + gamma;
    if (denom < -1e-6f) { 
        float p = 0.5f * (alpha - gamma) / denom;
        fractional_delay += p; 
    }

    if (fractional_delay > (float)frame_size / 2.0f) fractional_delay -= (float)frame_size;
    
    float speed_sound = 343.0f;
    float ratio = (fractional_delay / (float)sample_rate) * speed_sound / mic_dist;
    
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < -1.0f) ratio = -1.0f;
    
    return acosf(ratio) * 180.0f / PI;
}

inline void process_mvdr_neon(MvdrState* __restrict st, const cpx* __restrict X1, const cpx* __restrict X2, 
                              const float* __restrict sv1, const float* __restrict sv2, 
                              cpx* __restrict Y, int num_bins, bool update_covariance) { 
    for (int k = 0; k < num_bins; ++k) {
        cpx x1 = X1[k]; cpx x2 = X2[k];
        if (update_covariance) {
            cpx xx00 = x1 * std::conj(x1);
            cpx xx01 = x1 * std::conj(x2);
            st->R[k][0][0] = ALPHA * st->R[k][0][0] + (1.0f - ALPHA) * xx00;
            st->R[k][0][1] = ALPHA * st->R[k][0][1] + (1.0f - ALPHA) * xx01;
            st->R[k][1][0] = ALPHA * st->R[k][1][0] + (1.0f - ALPHA) * std::conj(xx01);
            st->R[k][1][1] = ALPHA * st->R[k][1][1] + (1.0f - ALPHA) * (x2 * std::conj(x2));
        }

        // --- The Fix: Dynamic Trace Loading (5%) to prevent WNG Hiss ---
        float trace = st->R[k][0][0].real() + st->R[k][1][1].real();
        float dynamic_load = 0.05f * trace + 1e-4f; 

        cpx R_loaded[2][2];
        R_loaded[0][0] = st->R[k][0][0] + dynamic_load; R_loaded[0][1] = st->R[k][0][1];
        R_loaded[1][0] = st->R[k][1][0];             R_loaded[1][1] = st->R[k][1][1] + dynamic_load;

        cpx invR[2][2]; invert_2x2(R_loaded, invR);
        cpx d1(sv1[2*k], sv1[2*k+1]); cpx d2(sv2[2*k], sv2[2*k+1]);
        
        cpx num1 = invR[0][0] * d1 + invR[0][1] * d2;
        cpx num2 = invR[1][0] * d1 + invR[1][1] * d2;
        cpx den = std::conj(d1) * num1 + std::conj(d2) * num2;
        if (std::abs(den) < 1e-9f) den = 1.0f;
        
        Y[k] = std::conj(num1 / den) * x1 + std::conj(num2 / den) * x2;
    }
}
#endif
