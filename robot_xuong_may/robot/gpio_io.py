# -*- coding: utf-8 -*-
"""
Nút nhấn START/STOP và còi báo trên GPIO Raspberry Pi (tùy chọn, GPIO_ENABLE = True).
  START : GPIO17 (chân 11) – nút NO nối GND
  STOP  : GPIO27 (chân 13) – nút NO nối GND
  CÒI   : GPIO18 (chân 12) – qua transistor NPN (S8050/2N2222) kéo còi chíp 5 V
Nút dừng khẩn cấp thật phải CẮT NGUỒN ĐỘNG CƠ bằng phần cứng (xem docs/WIRING.md).
"""
from __future__ import annotations

import logging
import threading
import time

log = logging.getLogger("gpio")

BEEP_PATTERNS = {
    "start": [(0.08, 0.0)],
    "arrive": [(0.12, 0.10), (0.12, 0.0)],
    "obstacle": [(0.5, 0.0)],
    "alarm": [(0.1, 0.1), (0.1, 0.1), (0.1, 0.0)],
}


class GpioPanel:
    def __init__(self, start_pin: int, stop_pin: int, buzzer_pin: int):
        from gpiozero import Button, OutputDevice
        self._start_flag = threading.Event()
        self._stop_flag = threading.Event()
        self.btn_start = Button(start_pin, pull_up=True, bounce_time=0.05)
        self.btn_stop = Button(stop_pin, pull_up=True, bounce_time=0.05)
        self.btn_start.when_pressed = self._start_flag.set
        self.btn_stop.when_pressed = self._stop_flag.set
        self.buzzer = OutputDevice(buzzer_pin, active_high=True, initial_value=False)
        self._busy = threading.Lock()
        log.info("GPIO: START=%d STOP=%d CÒI=%d", start_pin, stop_pin, buzzer_pin)

    def take_start(self) -> bool:
        if self._start_flag.is_set():
            self._start_flag.clear()
            return True
        return False

    def take_stop(self) -> bool:
        if self._stop_flag.is_set():
            self._stop_flag.clear()
            return True
        return False

    def beep(self, kind: str) -> None:
        pattern = BEEP_PATTERNS.get(kind)
        if not pattern or not self._busy.acquire(blocking=False):
            return

        def run():
            try:
                for on, off in pattern:
                    self.buzzer.on()
                    time.sleep(on)
                    self.buzzer.off()
                    time.sleep(off)
            finally:
                self._busy.release()
        threading.Thread(target=run, daemon=True).start()

    def close(self) -> None:
        for dev in (self.btn_start, self.btn_stop, self.buzzer):
            try:
                dev.close()
            except Exception:
                pass
