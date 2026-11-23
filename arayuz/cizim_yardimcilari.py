#!/usr/bin/env python3
"""Ortak cizim yardimcilari - YUVARLAK KOSE kirpma ve cerceve cizimi.

NEDEN VAR (2026-10-28, kullanici: "navigasyon ve gorev planlama ekraninda
bazi objelerin koseleri silik gibi gozukmuyordu"):

Bu ekrandaki uc gorsel widget (SuniUfukEkrani, TaktikRadarEkrani,
UyduHaritaWidget) paintEvent'i TAMAMEN kendisi ciziyor ve
super().paintEvent() HIC cagrilmiyor. Bunun iki sonucu var:

  1) Widget'in stil sayfasindaki "border-radius: 15px" ve
     "border: 2px solid ..." OLU KALIYOR - Qt, stil kaynakli zemini ve
     cerceveyi QFrame'in kendi paintEvent'inde cizer; o cagrilmayinca
     ekrana hic ulasmaz.
  2) El ile cizilen dolgular (suni ufkun mavi gokyuzu, radarin zemini,
     uydu karolari) DIKDORTGEN oldugu icin koselere kadar tasiyor.

Ikisi birlesince kullanicinin gordugu sey tam olarak su oluyor: kutunun
koseleri yuvarlak DEGIL, ama cerceve de gorunmuyor - "koseler silik".

COZUM iki parcali ve her ucunde ayni:
  * cizime baslamadan once ressam YUVARLAK bir yola kirpilir
    (yuvarlak_kirp) -> dolgu artik koselerden tasamaz;
  * cizim bitince ayni yaricapla cerceve EL ILE cizilir
    (yuvarlak_cerceve_ciz) -> stil sayfasinin yapamadigini yapar.

DIKKAT: govde icinde setClipRect kullanan yerler Qt.IntersectClip ile
cagrilmali; varsayilan ReplaceClip buradaki yuvarlak kirpmayi SILER ve
sorun geri doner.
"""
from PyQt5.QtCore import QRectF, Qt
from PyQt5.QtGui import QColor, QPainter, QPainterPath, QPen


def yuvarlak_yol(widget, yaricap):
    """Widget'in dis hatlarini veren yuvarlak kose yolu."""
    yol = QPainterPath()
    # 0.5 piksel iceri alinir: antialiasing'de kenar cizgisi piksel
    # merkezine otursun, aksi halde cerceve yarim piksel disarida kalip
    # SOLUK gorunur (sorunun "silik" tarifi kismen bundan geliyordu).
    yol.addRoundedRect(
        QRectF(widget.rect()).adjusted(0.5, 0.5, -0.5, -0.5),
        float(yaricap), float(yaricap))
    return yol


def yuvarlak_kirp(ressam, widget, yaricap):
    """Ressami widget'in yuvarlak kose hattina kirpar."""
    ressam.setClipPath(yuvarlak_yol(widget, yaricap))


def yuvarlak_cerceve_ciz(ressam, widget, yaricap, renk, kalinlik=2):
    """Yuvarlak cerceveyi el ile cizer (stil sayfasi bunu cizemiyor).

    Kirpma kaldirilir, cunku cerceve tam hat uzerinde duruyor; kirpma
    acik kalirsa cizginin dis yarisi kesilir ve cerceve YARIM KALINLIKTA,
    yani soluk gorunur."""
    ressam.save()
    try:
        ressam.setClipping(False)
        ressam.setRenderHint(QPainter.Antialiasing, True)
        kalem = QPen(QColor(renk))
        kalem.setWidth(int(kalinlik))
        ressam.setPen(kalem)
        ressam.setBrush(Qt.NoBrush)
        ressam.drawPath(yuvarlak_yol(widget, yaricap))
    finally:
        ressam.restore()
