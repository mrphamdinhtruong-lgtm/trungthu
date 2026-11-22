// Chương trình chạy trên máy tính để kiểm tra sac_dob.h (firmware) cho kết quả
// giống hệt robot/sac_dob.py. Được tests/test_sac_dob_parity.py tự biên dịch bằng g++.
//
// stdin : dòng 1 = vector tham số (thứ tự SacDobParams.as_vector() trong Python)
//         các dòng sau: "U r y" = update(r, y) ; "H y" = hold(y) ; "Z" = resetGains()
// stdout: mỗi bước update in "u ym e dhat K0 K1 K2"
#include <cstdio>
#include "../../firmware/robot_firmware/sac_dob.h"

int main() {
  SacDobParams p;
  const int NP = 26;
  float v[NP];
  for (int i = 0; i < NP; i++) {
    if (scanf("%f", &v[i]) != 1) return 1;
  }
  int k = 0;
  p.Ts = v[k++];
  p.tau_m = v[k++];
  for (int i = 0; i < 3; i++) p.gi[i] = v[k++];
  for (int i = 0; i < 3; i++) p.gp[i] = v[k++];
  p.sigma = v[k++];
  for (int i = 0; i < 3; i++) p.k0[i] = v[k++];
  for (int i = 0; i < 3; i++) p.kmin[i] = v[k++];
  for (int i = 0; i < 3; i++) p.kmax[i] = v[k++];
  p.e_db = v[k++];
  p.kn = v[k++];
  p.tau_n = v[k++];
  p.tau_q = v[k++];
  p.d_lim = v[k++];
  p.pi_kp = v[k++];
  p.pi_ki = v[k++];
  p.mode = (int16_t)(v[k++] + 0.5f);
  sacdob_prepare(&p);

  SacDobCtrl c;
  c.init(&p);
  char tag;
  while (scanf(" %c", &tag) == 1) {
    if (tag == 'H') {
      float y;
      if (scanf("%f", &y) != 1) return 2;
      c.hold(y);
    } else if (tag == 'Z') {
      c.resetGains();
    } else if (tag == 'U') {
      float r, y;
      if (scanf("%f %f", &r, &y) != 2) return 3;
      float u = c.update(r, y);
      printf("%.7g %.7g %.7g %.7g %.7g %.7g %.7g\n", u, c.ym, c.e, c.dhat, c.K[0], c.K[1], c.K[2]);
    } else {
      return 4;
    }
  }
  return 0;
}
