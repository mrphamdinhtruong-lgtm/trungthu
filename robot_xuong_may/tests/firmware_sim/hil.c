/* Chạy robot_firmware.elf (biên dịch với Arduino AVR core thật) trên ATmega328P mô phỏng (simavr)
 * + mô hình 4 động cơ / encoder đọc PWM & chân chiều quay thật. Dùng bởi tests/test_firmware_sim.py.
 * In: "t -> lệnh" (Pi gửi), "t <- dòng" (Arduino trả), "[động cơ vật lý RPM ...]". */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <simavr/sim_avr.h>
#include <simavr/sim_elf.h>
#include <simavr/avr_uart.h>
#include <simavr/avr_ioport.h>

static avr_t *avr;
static char line[256]; static int ln = 0;
static double now(void) { return (double)avr->cycle / 16e6; }
static void uart_out(struct avr_irq_t *irq, uint32_t v, void *p) {
  (void)irq; (void)p;
  if (v == '\n') { line[ln] = 0; printf("%.3f <- %s\n", now(), line); ln = 0; }
  else if (v != '\r' && ln < 255) line[ln++] = (char)v;
}
/* lịch gửi lệnh: (thời điểm, dòng) */
typedef struct { double t; const char *s; } cmd_t;
static cmd_t cmds[400]; static int ncmd = 0;
static void add(double t, const char *s) { cmds[ncmd].t = t; cmds[ncmd].s = s; ncmd++; }

int main(int argc, char **argv) {
  elf_firmware_t f; memset(&f, 0, sizeof f);
  if (elf_read_firmware(argv[1], &f)) { printf("không đọc được ELF\n"); return 1; }
  strcpy(f.mmcu, "atmega328p"); f.frequency = 16000000;
  avr = avr_make_mcu_by_name("atmega328p"); avr_init(avr); avr_load_firmware(avr, &f);
  uint32_t fl = 0; avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &fl);
  fl &= ~AVR_UART_FLAG_STDIO; avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &fl);
  avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), uart_out, NULL);
  avr_irq_t *uin = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT);
  /* chân encoder kênh A: D2=PD2, D3=PD3, A2=PC2, A3=PC3 */
  avr_irq_t *enc[4] = { avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('D'), 2), avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('D'), 3),
                        avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('C'), 2), avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('C'), 3) };
  int encv[4] = {1,1,1,1}; double phase[4] = {0}, rpm[4] = {0};
  for (int i = 0; i < 4; i++) avr_raise_irq(enc[i], 1);

  /* kịch bản lệnh giống Raspberry Pi */
  add(0.30, "?");
  static char wbuf[200][32];
  int k = 0;
  for (double t = 0.40; t < 2.40; t += 0.05) { snprintf(wbuf[k], 32, "W,50,50,50,50"); add(t, wbuf[k++]); }
  add(1.00, "D,0"); add(1.25, "D,-1");
  add(2.45, "P,gie,7"); add(2.50, "Q");
  /* 2.4 s ngừng gửi -> 2.9 s Arduino phải tự dừng */
  for (double t = 3.60; t < 4.60; t += 0.05) { snprintf(wbuf[k], 32, "W,-40,40,-40,40"); add(t, wbuf[k++]); }
  add(4.10, "E,1"); add(4.70, "E,0"); add(4.75, "S"); add(4.80, "X,bad"); add(4.85, "W,1,2");
  double T_END = 5.2;
  for (int i = 1; i < ncmd; i++)                 /* sắp xếp lệnh theo thời gian */
    for (int j = i; j > 0 && cmds[j].t < cmds[j - 1].t; j--) { cmd_t tmp = cmds[j]; cmds[j] = cmds[j - 1]; cmds[j - 1] = tmp; }

  int ci = 0, cj = 0; double next_char = 0;
  /* thanh ghi: OCR0A 0x47 (D6) OCR0B 0x48 (D5) OCR1AL 0x88 (D9) OCR1BL 0x8A (D10); TCCR0A 0x44, TCCR1A 0x80 */
  /* PORTB 0x25 PORTC 0x28 PORTD 0x2B */
  while (now() < T_END) {
    avr_run(avr);
    double t = now();
    if (avr->state == cpu_Done || avr->state == cpu_Crashed) { printf("CPU dừng/crash ở %.3f\n", t); return 2; }
    /* gửi ký tự lệnh: 1 ký tự / 100 µs */
    if (ci < ncmd && t >= cmds[ci].t && t >= next_char) {
      const char *s = cmds[ci].s; int L = strlen(s);
      if (cj < L) { avr_raise_irq(uin, (uint8_t)s[cj++]); }
      else { avr_raise_irq(uin, '\n'); if (cj == L) printf("%.3f -> %s\n", t, s); cj = 0; ci++; }
      next_char = t + 100e-6;
    }
    /* mô hình động cơ cập nhật mỗi 50 µs */
    static double last = 0; double dt = t - last; if (dt < 50e-6) continue; last = t;
    uint8_t *d = avr->data;
    uint8_t tccr0a = d[0x44], tccr1a = d[0x80], pb = d[0x25], pc = d[0x28], pdd = d[0x2B];
    double duty[4];
    /* FL: EN D5 (OC0B, COM0B1 bit5), IN1 D4 PD4, IN2 D7 PD7 */
    duty[0] = (tccr0a & 0x20) ? d[0x48] / 255.0 : ((pdd >> 5) & 1);
    /* FR: EN D6 (OC0A, COM0A1 bit7), IN3 D8 PB0, IN4 D11 PB3 */
    duty[1] = (tccr0a & 0x80) ? d[0x47] / 255.0 : ((pdd >> 6) & 1);
    /* RL: EN D9 (OC1A, COM1A1 bit7), IN1 D12 PB4, IN2 D13 PB5 */
    duty[2] = (tccr1a & 0x80) ? d[0x88] / 255.0 : ((pb >> 1) & 1);
    /* RR: EN D10 (OC1B, COM1B1 bit5), IN3 A0 PC0, IN4 A1 PC1 */
    duty[3] = (tccr1a & 0x20) ? d[0x8A] / 255.0 : ((pb >> 2) & 1);
    int a[4] = { (pdd >> 4) & 1, pb & 1, (pb >> 4) & 1, pc & 1 };
    int b[4] = { (pdd >> 7) & 1, (pb >> 3) & 1, (pb >> 5) & 1, (pc >> 1) & 1 };
    for (int i = 0; i < 4; i++) {
      double u = (a[i] && !b[i]) ? duty[i] : ((!a[i] && b[i]) ? -duty[i] : 0.0);
      if (a[i] && b[i]) u = 0;                                   /* phanh */
      double drive = (fabs(u) < 0.11) ? 0.0 : 105.0 * (u - copysign(0.11, u)) / 0.89;   /* vùng chết ~28/255 */
      rpm[i] += dt / 0.10 * (drive - rpm[i]);                    /* quán tính 0.1 s */
      phase[i] += fabs(rpm[i]) / 60.0 * 1980.0 * dt;             /* số cạnh encoder (CHANGE) */
      while (phase[i] >= 1.0) { phase[i] -= 1.0; encv[i] ^= 1; avr_raise_irq(enc[i], encv[i]); }
    }
    static double lastp = 0;
    if (t - lastp >= 0.25) { lastp = t; printf("%.3f    [động cơ vật lý RPM: %.1f %.1f %.1f %.1f]\n", t, rpm[0], rpm[1], rpm[2], rpm[3]); }
  }
  return 0;
}
