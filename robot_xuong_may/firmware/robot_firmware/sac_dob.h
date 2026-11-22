/*
 * =====================================================================
 *  sac_dob.h – BỘ ĐIỀU KHIỂN TỐC ĐỘ ĐỘNG CƠ SAC + DOB (THÍCH NGHI)
 * =====================================================================
 *  SAC : Simple Adaptive Control (Kaufman – Barkana – Sobel)
 *        u_sac = K(t) * rho,   rho = [e_y, x_m, u_m]
 *        K     = K_P + K_I
 *        K_P   = e_y * rho * Gamma_P
 *        dK_I/dt = e_y * rho * Gamma_I - sigma * (K_I - K_0)
 *        Mô hình mẫu bậc 1:  tau_m * dx_m/dt = -x_m + u_m   (y_m = x_m)
 *        e_y = y_m - y
 *
 *  DOB : Disturbance Observer với mô hình danh định Pn(s) = Kn / (tau_n s + 1)
 *        d_hat = Q(s) * Pn^-1(s) * y  -  Q(s) * u ,   Q(s) = 1 / (tau_q s + 1)
 *        u     = u_sac - d_hat
 *
 *  Tín hiệu chuẩn hóa: y = RPM / RPM_DANH_DINH, u = duty trong [-1, 1].
 *
 *  File này là C++ thuần (không phụ thuộc Arduino) để có thể biên dịch
 *  trên máy tính và so sánh 1-1 với bản Python robot/sac_dob.py
 *  (xem tests/test_sac_dob_parity.py).
 * =====================================================================
 */
#ifndef SAC_DOB_H
#define SAC_DOB_H

#include <math.h>
#include <stdint.h>

enum {
  CTRL_PI = 0,       // PI kinh điển (để so sánh)
  CTRL_SAC = 1,      // chỉ SAC
  CTRL_SAC_DOB = 2   // SAC + DOB (mặc định)
};

struct SacDobParams {
  float Ts;          // chu kỳ lấy mẫu (s)
  // ---- SAC ----
  float tau_m;       // hằng số thời gian mô hình mẫu
  float gi[3];       // Gamma_I  [Ke, Kx, Ku]
  float gp[3];       // Gamma_P  [Ke, Kx, Ku]
  float sigma;       // hệ số rò sigma-modification
  float k0[3];       // độ lợi ban đầu
  float kmin[3];     // chặn dưới
  float kmax[3];     // chặn trên
  float e_db;        // vùng chết sai số cho luật thích nghi
  // ---- DOB ----
  float kn;          // Kn mô hình danh định
  float tau_n;       // tau_n mô hình danh định
  float tau_q;       // tau_q bộ lọc Q
  float d_lim;       // giới hạn |d_hat|
  // ---- PI ----
  float pi_kp;
  float pi_ki;
  int16_t mode;      // CTRL_PI / CTRL_SAC / CTRL_SAC_DOB
  // ---- hằng số dẫn xuất (tính bởi sacdob_prepare) ----
  float am;          // exp(-Ts/tau_m)
  float aq;          // Ts/(tau_q+Ts)
  float inv_tau_q;
  float inv_kn;
};

static inline float sd_clamp(float x, float lo, float hi) {
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}

/* Tính các hằng số dẫn xuất – gọi lại mỗi khi đổi tham số */
static inline void sacdob_prepare(SacDobParams* p) {
  if (p->tau_m < p->Ts) p->tau_m = p->Ts;
  if (p->tau_q < p->Ts) p->tau_q = p->Ts;
  if (p->tau_n < 0.0f) p->tau_n = 0.0f;
  if (p->kn < 0.05f) p->kn = 0.05f;
  if (p->mode < CTRL_PI || p->mode > CTRL_SAC_DOB) p->mode = CTRL_SAC_DOB;
  p->am = expf(-p->Ts / p->tau_m);
  p->aq = p->Ts / (p->tau_q + p->Ts);
  p->inv_tau_q = 1.0f / p->tau_q;
  p->inv_kn = 1.0f / p->kn;
}

class SacDobCtrl {
 public:
  // Biến công khai để gửi telemetry / debug
  float ym;      // đầu ra mô hình mẫu
  float e;       // sai số e_y = ym - y
  float u;       // tín hiệu điều khiển đã giới hạn [-1, 1]
  float dhat;    // nhiễu ước lượng (luôn tính, chỉ bù khi mode = SAC_DOB)
  float KI[3];   // phần tích phân của độ lợi thích nghi
  float K[3];    // độ lợi tổng K = K_I + K_P

  void init(const SacDobParams* params) {
    p = params;
    resetGains();
    hold(0.0f);
  }

  /* Đưa độ lợi thích nghi về giá trị ban đầu K0 */
  void resetGains() {
    for (uint8_t i = 0; i < 3; i++) {
      KI[i] = p->k0[i];
      K[i] = p->k0[i];
    }
  }

  /* Giữ bộ điều khiển ở trạng thái nghỉ, đồng bộ với tốc độ đo y
     (dùng khi đứng yên / chạy vòng hở / dừng khẩn) – giữ nguyên độ lợi đã học */
  void hold(float y) {
    ym = y;
    yf = y;
    uf = 0.0f;
    e = 0.0f;
    u = 0.0f;
    dhat = 0.0f;
    pi_int = 0.0f;
    sat = 0;
  }

  /* Một bước điều khiển. r = tốc độ đặt chuẩn hóa, y = tốc độ đo chuẩn hóa */
  float update(float r, float y) {
    const float Ts = p->Ts;
    float uc = 0.0f;
    e = ym - y;

    if (p->mode == CTRL_PI) {
      // ---------- PI kinh điển, chống bão hòa tích phân ----------
      float err = r - y;
      bool windup = (sat > 0 && err > 0.0f) || (sat < 0 && err < 0.0f);
      if (!windup) pi_int += p->pi_ki * Ts * err;
      pi_int = sd_clamp(pi_int, -1.0f, 1.0f);
      uc = p->pi_kp * err + pi_int;
    } else {
      // ---------- SAC: luật thích nghi đơn giản ----------
      float rho[3];
      rho[0] = e;      // sai số đầu ra
      rho[1] = ym;     // trạng thái mô hình mẫu
      rho[2] = r;      // đầu vào mô hình mẫu
      bool adapt = fabsf(e) > p->e_db;
      // bão hòa cùng chiều sai số -> ngừng thích nghi (chống trôi do bão hòa)
      if ((sat > 0 && e > 0.0f) || (sat < 0 && e < 0.0f)) adapt = false;
      for (uint8_t i = 0; i < 3; i++) {
        float g = e * rho[i];
        float dk = -p->sigma * (KI[i] - p->k0[i]);
        if (adapt) dk += p->gi[i] * g;
        KI[i] = sd_clamp(KI[i] + Ts * dk, p->kmin[i], p->kmax[i]);
        K[i] = KI[i] + p->gp[i] * g;
        uc += K[i] * rho[i];
      }
    }

    // ---------- DOB: luôn cập nhật bộ lọc để chuyển chế độ không giật ----------
    yf += p->aq * (y - yf);                                   // Q*y
    float qpinv = (p->tau_n * (y - yf) * p->inv_tau_q + yf)   // Q*Pn^-1*y
                  * p->inv_kn;
    dhat = sd_clamp(qpinv - uf, -p->d_lim, p->d_lim);         // uf = Q*u (bước trước)
    float comp = (p->mode == CTRL_SAC_DOB) ? dhat : 0.0f;

    float ur = uc - comp;
    u = sd_clamp(ur, -1.0f, 1.0f);
    sat = (ur > 1.0f) ? 1 : ((ur < -1.0f) ? -1 : 0);

    uf += p->aq * (u - uf);                                   // cập nhật Q*u
    ym = p->am * ym + (1.0f - p->am) * r;                     // mô hình mẫu bước kế
    return u;
  }

 private:
  const SacDobParams* p;
  float yf;      // Q*y
  float uf;      // Q*u
  float pi_int;  // tích phân PI
  int8_t sat;    // +1 bão hòa trên, -1 bão hòa dưới, 0 không
};

#endif  // SAC_DOB_H
