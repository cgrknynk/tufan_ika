"""SshTerminalWidget: Qt icine gomulu, gercek bir SSH terminali.

Arac Jetson'una hicbir zaman monitor baglanmayacagi icin, arayuzdeki eski
"log ekrani" (sadece append-only metin kutusu) yerine BURADAN dogrudan
komut calistirilabilen, pyte (VT100/ANSI terminal emulatoru) + pty +
`ssh -tt` ile calisan gercek bir terminal penceresi. ANSI renkleri
(ls, git, prompt vb.) gercek terminaldeki gibi renkli gosterilir.

main.py'deki eski `self.ui.terminal_ekrani.append(...)` cagrilarinin
calismaya devam etmesi icin `append()` uyumluluk metodu var, ama o metod
metni GERCEK terminale YAZMAZ (ssh oturumunu karistirmamak icin) -- sadece
konsola (stdout) yazar. Panelin kendisi artik %100 gercek bir terminal.
"""
import os
import pty
import fcntl
import termios
import struct
import signal
import subprocess

from PyQt5.QtCore import Qt, QSocketNotifier, QTimer
from PyQt5.QtGui import QFont, QFontMetrics, QTextCursor, QColor, QTextCharFormat
from PyQt5.QtWidgets import QTextEdit

import pyte

VARSAYILAN_KULLANICI = "arf203"
VARSAYILAN_HOST = "10.40.64.43"
BASLANGIC_DIZINI = "~/Desktop/tufan_v2_ws"

# pyte'in standart 8 renk ismi (bkz. pyte.graphics.FG_ANSI/BG_ANSI) -> hex.
# NOT: pyte tarihsel nedenlerle sari icin 'brown' ismini kullanir.
ANSI_RENKLER = {
    "black": "#000000",
    "red": "#e5493b",
    "green": "#3aca4f",
    "brown": "#d7ba3d",     # sari
    "yellow": "#d7ba3d",
    "blue": "#4a9cf0",
    "magenta": "#c464d6",
    "cyan": "#37c3c3",
    "white": "#d6d6d6",
    "brightblack": "#7a7a7a",
    "brightred": "#ff6b5b",
    "brightgreen": "#5cf074",
    "brightyellow": "#f0e050",
    "brightblue": "#7ab8ff",
    "brightmagenta": "#e58cf5",
    "brightcyan": "#5fe8e8",
    "brightwhite": "#ffffff",
}
VARSAYILAN_ON_RENK = QColor("#00FF41")
VARSAYILAN_ARKA_RENK = QColor("#000000")


def _renk(isim, varsayilan):
    if not isim or isim == "default":
        return varsayilan
    if isim in ANSI_RENKLER:
        return QColor(ANSI_RENKLER[isim])
    if len(isim) == 6:
        try:
            return QColor(f"#{isim}")
        except ValueError:
            pass
    return varsayilan


class SshTerminalWidget(QTextEdit):
    def __init__(self, parent=None, user=VARSAYILAN_KULLANICI, host=VARSAYILAN_HOST):
        super().__init__(parent)
        self.user = user
        self.host = host

        self.setUndoRedoEnabled(False)
        self.setLineWrapMode(QTextEdit.NoWrap)
        self.setAcceptRichText(False)
        font = QFont("DejaVu Sans Mono")
        font.setStyleHint(QFont.Monospace)
        font.setPointSize(10)
        self.setFont(font)
        self.setStyleSheet(
            "QTextEdit {"
            " background-color: #000000;"
            " color: #00FF41;"
            " border: none;"
            "}"
        )

        self._master_fd = None
        self._child_pid = None
        self._notifier = None
        self._screen = None
        self._stream = None
        self._cols = 80
        self._rows = 24

        self._render_timer = QTimer(self)
        self._render_timer.setSingleShot(True)
        self._render_timer.timeout.connect(self._ekrani_guncelle)

        self._baslat()

    # ------------------------------------------------------------------ #
    # Baglanti kurma / kapama
    # ------------------------------------------------------------------ #
    def _baslat(self):
        self._cols, self._rows = self._widget_boyutundan_hucre_sayisi()
        self._screen = pyte.HistoryScreen(self._cols, self._rows, history=2000)
        self._stream = pyte.Stream(self._screen)
        self._init_bos_ekran()

        master_fd, slave_fd = pty.openpty()
        self._ayarla_pencere_boyutu(master_fd, self._rows, self._cols)

        env = dict(os.environ)
        env["TERM"] = "xterm-256color"

        try:
            proc = subprocess.Popen(
                ["ssh", "-tt", "-o", "StrictHostKeyChecking=accept-new",
                 f"{self.user}@{self.host}"],
                stdin=slave_fd, stdout=slave_fd, stderr=slave_fd,
                preexec_fn=os.setsid, close_fds=True, env=env,
            )
        except FileNotFoundError:
            self.setPlainText("[HATA] 'ssh' komutu bulunamadi.")
            os.close(master_fd)
            os.close(slave_fd)
            return

        os.close(slave_fd)
        self._master_fd = master_fd
        self._child_pid = proc.pid
        self._proc = proc

        os.set_blocking(master_fd, False)
        self._notifier = QSocketNotifier(master_fd, QSocketNotifier.Read, self)
        self._notifier.activated.connect(self._pty_okunabilir)

        # Baglanti kurulur kurulmaz proje dizinine gec ve ekrani temizle.
        # (Girdi pty tamponunda bekler, shell hazir olunca islenir -- MOTD
        # tam basilmadan gonderilse bile sorun cikarmaz.)
        QTimer.singleShot(600, self._proje_dizinine_gec)

    def _proje_dizinine_gec(self):
        if self._master_fd is None:
            return
        try:
            # PROMPT_COMMAND='history -a': her komut CALISTIKTAN HEMEN SONRA
            # ~/.bash_history dosyasina yazilir. Bu olmadan bash gecmisi
            # SADECE duzgun bir "exit" ile kapanan oturumlarda diske yazar --
            # arayuz kapanirken ssh baglantisini SIGTERM ile kestigimiz icin
            # (bkz. baglantiyi_kapat) o oturumdaki komutlar kaybolurdu. Bu
            # sayede yazilan komutlar bir sonraki baglantida (yukari ok /
            # gecmis) hala goruntur.
            os.write(self._master_fd,
                     f"export PROMPT_COMMAND='history -a'; cd {BASLANGIC_DIZINI} && clear\r".encode())
        except OSError:
            pass

    def yeniden_baglan(self):
        self.baglantiyi_kapat()
        self._baslat()

    def baglantiyi_kapat(self):
        if self._notifier is not None:
            self._notifier.setEnabled(False)
            self._notifier = None
        if self._child_pid is not None:
            try:
                os.kill(self._child_pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            self._child_pid = None
        if self._master_fd is not None:
            try:
                os.close(self._master_fd)
            except OSError:
                pass
            self._master_fd = None

    def closeEvent(self, event):
        self.baglantiyi_kapat()
        super().closeEvent(event)

    # ------------------------------------------------------------------ #
    # PTY <-> pyte <-> ekran
    # ------------------------------------------------------------------ #
    def _pty_okunabilir(self):
        try:
            veri = os.read(self._master_fd, 65536)
        except (OSError, BlockingIOError):
            return
        if not veri:
            self._notifier.setEnabled(False)
            self._ekrana_sistem_mesaji("\n[SSH baglantisi kapandi. Yeniden baglanmak icin yeniden_baglan() cagirin.]")
            return
        self._stream.feed(veri.decode("utf-8", errors="replace"))
        # PERFORMANS: her okumada TAM ekran yeniden cizmek (her tus vurusunda
        # butun grid'i baştan olusturmak) arayuzu kilitliyordu. Bunun yerine:
        # (1) sadece pyte'in "dirty" (degisen) satirlarini yeniden ciziyoruz,
        # (2) hizli ardisik okumalari kisa bir sure (~16ms) biriktirip TEK
        # render'da isliyoruz (debounce).
        if not self._render_timer.isActive():
            self._render_timer.start(16)

    def _init_bos_ekran(self):
        self.clear()
        cur = self.textCursor()
        for i in range(self._screen.lines - 1):
            cur.insertBlock()

    def _ekrani_guncelle(self):
        dirty = self._screen.dirty
        doc = self.document()
        # Belge blok sayisi ekran satir sayisiyla senkron degilse (resize
        # sonrasi vb.) -- dirty bos olsa bile -- tamamini yeniden kur.
        boyut_uyumsuz = doc.blockCount() != self._screen.lines
        if not dirty and not boyut_uyumsuz:
            return
        self.setUpdatesEnabled(False)

        if boyut_uyumsuz:
            self._init_bos_ekran()
            satirlar = range(self._screen.lines)
        else:
            satirlar = sorted(r for r in dirty if 0 <= r < self._screen.lines)

        buf = self._screen.buffer
        varsayilan_fmt = QTextCharFormat()
        varsayilan_fmt.setForeground(VARSAYILAN_ON_RENK)

        for row in satirlar:
            blok = doc.findBlockByNumber(row)
            cur = QTextCursor(blok)
            cur.movePosition(QTextCursor.StartOfBlock)
            cur.movePosition(QTextCursor.EndOfBlock, QTextCursor.KeepAnchor)
            cur.removeSelectedText()

            satir = buf[row]
            son_fmt_anahtari = None
            parca = ""
            for col in range(self._screen.columns):
                ch = satir[col]
                fg = ch.reverse and ch.bg or ch.fg
                bg = ch.reverse and ch.fg or ch.bg
                anahtar = (fg, bg, ch.bold)
                if anahtar != son_fmt_anahtari:
                    if parca:
                        self._parca_yaz(cur, parca, son_fmt_anahtari, varsayilan_fmt)
                    parca = ch.data or " "
                    son_fmt_anahtari = anahtar
                else:
                    parca += ch.data or " "
            if parca:
                self._parca_yaz(cur, parca, son_fmt_anahtari, varsayilan_fmt)

        dirty.clear()
        self.setUpdatesEnabled(True)

        # imleci pyte'in kendi imlec konumuna tasi
        cur2 = self.textCursor()
        cur2.movePosition(QTextCursor.Start)
        cur2.movePosition(QTextCursor.Down, QTextCursor.MoveAnchor,
                           min(self._screen.cursor.y, self._screen.lines - 1))
        cur2.movePosition(QTextCursor.Right, QTextCursor.MoveAnchor,
                           min(self._screen.cursor.x, self._screen.columns))
        self.setTextCursor(cur2)
        self.ensureCursorVisible()

    @staticmethod
    def _parca_yaz(cur, metin, anahtar, varsayilan_fmt):
        if anahtar is None:
            cur.insertText(metin, varsayilan_fmt)
            return
        fg, bg, bold = anahtar
        fmt = QTextCharFormat()
        renk = _renk(fg, VARSAYILAN_ON_RENK)
        if bold and fg in ("black", "red", "green", "brown", "yellow",
                            "blue", "magenta", "cyan", "white"):
            renk = _renk("bright" + fg, renk)
        fmt.setForeground(renk)
        if bg and bg != "default":
            fmt.setBackground(_renk(bg, VARSAYILAN_ARKA_RENK))
        if bold:
            fmt.setFontWeight(QFont.Bold)
        cur.insertText(metin, fmt)

    def _ekrana_sistem_mesaji(self, metin):
        fmt = QTextCharFormat()
        fmt.setForeground(QColor("#ff6b5b"))
        cur = self.textCursor()
        cur.movePosition(QTextCursor.End)
        cur.insertText(metin, fmt)

    # ------------------------------------------------------------------ #
    # Klavye -> pty
    # ------------------------------------------------------------------ #
    _OZEL_TUSLAR = {
        Qt.Key_Up: b"\x1b[A",
        Qt.Key_Down: b"\x1b[B",
        Qt.Key_Right: b"\x1b[C",
        Qt.Key_Left: b"\x1b[D",
        Qt.Key_Home: b"\x1b[H",
        Qt.Key_End: b"\x1b[F",
        Qt.Key_PageUp: b"\x1b[5~",
        Qt.Key_PageDown: b"\x1b[6~",
        Qt.Key_Delete: b"\x1b[3~",
        Qt.Key_Backspace: b"\x7f",
        Qt.Key_Tab: b"\t",
        Qt.Key_Escape: b"\x1b",
        Qt.Key_Return: b"\r",
        Qt.Key_Enter: b"\r",
    }

    def keyPressEvent(self, event):
        if self._master_fd is None:
            return
        key = event.key()
        veri = self._OZEL_TUSLAR.get(key)
        if veri is None:
            metin = event.text()
            if not metin:
                return
            veri = metin.encode("utf-8", errors="replace")
        try:
            os.write(self._master_fd, veri)
        except OSError:
            pass
        # NOT: super().keyPressEvent() KASITLI OLARAK cagrilmiyor -- QTextEdit'in
        # kendi yerel duzenlemesi devre disi, tum girdi pty'ye gidiyor; ekran
        # sadece ssh'tan gelen gercek veriyle (_ekrani_guncelle) guncelleniyor.

    def mousePressEvent(self, event):
        self.setFocus()
        super().mousePressEvent(event)

    # ------------------------------------------------------------------ #
    # Boyutlandirma
    # ------------------------------------------------------------------ #
    def _widget_boyutundan_hucre_sayisi(self):
        fm = QFontMetrics(self.font())
        char_w = max(1, fm.horizontalAdvance("M"))
        char_h = max(1, fm.height())
        cols = max(20, self.viewport().width() // char_w) if self.viewport().width() else 80
        rows = max(5, self.viewport().height() // char_h) if self.viewport().height() else 24
        return cols, rows

    @staticmethod
    def _ayarla_pencere_boyutu(fd, rows, cols):
        try:
            fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))
        except OSError:
            pass

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._master_fd is None or self._screen is None:
            return
        cols, rows = self._widget_boyutundan_hucre_sayisi()
        if cols == self._cols and rows == self._rows:
            return
        self._cols, self._rows = cols, rows
        self._screen.resize(rows, cols)
        self._ayarla_pencere_boyutu(self._master_fd, rows, cols)
        if self._child_pid is not None:
            try:
                os.kill(self._child_pid, signal.SIGWINCH)
            except ProcessLookupError:
                pass
        if not self._render_timer.isActive():
            self._render_timer.start(16)

    # ------------------------------------------------------------------ #
    # Eski kod uyumlulugu: main.py bu paneli eskiden append() ile
    # kullaniyordu (durum mesajlari). Terminali KARISTIRMAMAK icin bu
    # mesajlari artik sadece konsola (stdout) yaziyoruz.
    # ------------------------------------------------------------------ #
    def append(self, html_veya_metin):
        print(f"[ARAYUZ LOG] {html_veya_metin}")
