# -*- coding: utf-8 -*-
"""Gói chương trình robot thu gom rác (chạy trên Raspberry Pi)."""


def _utf8_console() -> None:
    """Windows: khi kết quả in ra bị chuyển hướng (pipe / file / chương trình khác thu lại), Python dùng
    bảng mã cp1252 không có chữ tiếng Việt -> UnicodeEncodeError. Ép stdout/stderr sang UTF-8."""
    import sys
    for stream in (sys.stdout, sys.stderr):
        try:
            enc = (getattr(stream, "encoding", "") or "").lower().replace("-", "").replace("_", "")
            if stream is not None and enc != "utf8":
                stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError, OSError):
            pass


_utf8_console()
