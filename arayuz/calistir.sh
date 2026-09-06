#!/bin/bash
# TUFAN Yer Kontrol İstasyonu başlatıcısı (2026-09-01)
#
# "python3 main.py" DOĞRUDAN (VS Code'un entegre terminalinden vb.) çalıştırılırsa
# ROS2 ortamı (rclpy) source edilmemiş olabilir - bu durumda arayüz artık
# çökmüyor (bkz. telemetri_sistemi.py'deki 2026-09-01 düzeltmesi) ama
# telemetri/sürüş/harita verisi TAMAMEN DEVRE DIŞI kalır. Bu script doğru
# ortamı garantiler - arayüzü başlatmak için HER ZAMAN bunu kullanın:
#   ./calistir.sh
cd "$(dirname "${BASH_SOURCE[0]}")"

# DÜZELTME (2026-09-05, kullanıcı: "arayüz sürekli çöküyor") - izole
# testle KANITLANDI: varsayılan ROS_DOMAIN_ID (0) bu ağda (paylaşılan
# WiFi/Ubiquiti) çok kalabalık DDS keşif trafiğine denk geliyor - saf bir
# rclpy Node() oluşturma bile 53 SANİYE sürdü. ROS_DOMAIN_ID=77 (izole)
# ile AYNI işlem 0.98 saniyeye düştü. main.py'nin açılışta (TelemetriThread
# __init__'inde, GUI thread'de SENKRON) rclpy.init()/Node() çağırması
# yüzünden bu gecikme TÜM ARAYÜZÜ dakikalarca dondurup "çökmüş" gibi
# gösteriyordu. KRİTİK: araç Jetson'daki tufan_mppi.launch.py de AYNI
# domain'de (77) çalıştırılmalı - farklı domain'de olsalar birbirlerini
# HİÇ GÖRMEZLER (bkz. DOKUMANTASYON.md). NOT: main.py'nin kendisi de
# artık bunu koda gömülü olarak ayarlıyor (bkz. main.py başı) - burada
# TEKRAR ayarlanması sadece bu script üzerinden çalıştırıldığında en
# baştan (rclpy import edilmeden ÖNCE, öz-onarma yeniden başlatmasına
# hiç gerek kalmadan) doğru olsun diye.
export ROS_DOMAIN_ID=77

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/tufan-yer/ros2_humble/install/setup.bash 2>/dev/null

# DÜZELTME (2026-09-05, kullanıcı: "yine çöktü, çöktüğü zaman sana
# raporlasın neden çöktüğünü") - eskiden burada `exec python3 main.py`
# vardı: script kendini PYTHON İLE DEĞİŞTİRİYORDU (exec), bu yüzden
# python3 çöktükten/öldürüldükten SONRA hiçbir şey (çıkış kodu, hangi
# sinyalle öldüğü) YAKALANAMIYORDU - "neden çöktü" sorusu her seferinde
# sıfırdan, dağınık log parçalarından araştırılmak zorunda kalıyordu.
# Artık `exec` KULLANILMIYOR - script python3'ü normal bir alt-süreç
# olarak çalıştırıp çıkışını BEKLİYOR, çıkış koduna bakıp (128+sinyal =
# dışarıdan bir sinyalle öldürüldü, ör. 137 = SIGKILL, işletim
# sistemi/pencere yöneticisi "yanıt vermiyor" diye kapattı DEMEKTİR) ve
# son log satırlarına bakarak (temiz "Sistem kapatılıyor" var mı) BİR
# SONRAKİ soruşturmanın ilk adımı olacak özet bir rapor
# (crash_rapor_son.txt) yazıyor - artık "neden çöktü" sorusuna doğrudan
# bu dosyayı okuyarak başlanabilir.
ulimit -c unlimited 2>/dev/null || true  # SIGSEGV/SIGABRT olursa core dump - ileri analiz icin

# OTOMATİK KURTARMA (2026-09-05, kullanıcı: "ara ara force quit yapmak
# gerekiyor... otomatize et her seferinde uğraşmayalım") - eskiden burada
# python3 main.py TEK SEFER çalışıp script bitiyordu; arayüz donup
# force-quit edildiğinde (ya da gerçek bir SIGSEGV/kill olduğunda)
# kullanıcının (ya da benim) ELLE kill+yeniden-başlatma yapması
# gerekiyordu. Artık python3 main.py bir SÜPERVİZÖR DÖNGÜSÜ içinde -
# çıkış TEMİZ (kod 0, X butonu/Sistemi Kapat) DEĞİLSE otomatik olarak
# yeniden başlatılır. TEMİZ çıkışta döngü DURUR (kullanıcı BİLEREK
# kapatmışsa saygı duyulur, istenmeden sürekli yeniden açılmaz).
# CRASH-LOOP KORUMASI: art arda ÇOK HIZLI (10sn altı) çökmeler
# MAKS_ARDISIK_HIZLI_COKME'yi aşarsa döngü durur - kalıcı/tekrarlayan bir
# hatada sonsuz döngüye girip kaynak tüketmesin, elle müdahale gereksin.
MAKS_ARDISIK_HIZLI_COKME=5
_ARDISIK_HIZLI_COKME=0
_DENEME=0

while true; do
    _DENEME=$((_DENEME + 1))
    _LOG="/tmp/tufan_son_calisma.log"
    _BASLANGIC_EPOCH=$(date +%s)
    _BASLANGIC=$(date '+%Y-%m-%d %H:%M:%S')
    # DÜZELTME (2026-09-06, kullanıcı: "lidar ekranında çok fazla komut
    # verisi gecikiyor" - STALL izleyicisi kanıtladı) - "-u" (unbuffered)
    # bayrağı ÖNCEKİ bir oturumda (canlı teşhis kolaylığı için) eklenmişti
    # AMA CİDDİ bir yan etkisi ortaya çıktı: log_yaz() içindeki print()
    # çağrısının KENDİSİ, GUI thread'de SENKRON olarak, tam **5.91 SANİYE**
    # takılı yakalandı! -u ile HER print() ANINDA (tampon beklemeden)
    # stdout pipe'ına (tee'ye) yazmaya çalışıyor - eğer sistem genelinde
    # CPU açlığı varsa (bu Jetson paylaşılan bir masaüstü - VS Code,
    # Claude Code CLI, ROS2 node'ları, GUI hepsi aynı anda çalışıyor,
    # session boyunca defalarca kanıtlanan bir örüntü) `tee` process'i
    # pipe'ı yeterince sık boşaltamayabilir - pipe buffer dolunca YAZICI
    # taraf (main.py'nin print() çağrısı, GUI thread'İNDE) OS seviyesinde
    # BLOKE olur. Normal (tamponlu) modda ise yazmalar ~8KB'lık bloklar
    # halinde biriktirilip DAHA SEYREK/DAHA VERİMLİ flush edilir - bu,
    # gerçek zamanlı araç/silah kontrolünü CİDDİ ŞEKİLDE dondurabilen bu
    # riski ortadan kaldırır. Bedel: crash/force-quit anında flush
    # edilmemiş son birkaç satır kaybolabilir - ama gerçek SIGSEGV/SIGABRT
    # durumunda zaten faulthandler (bkz. main.py) AYRI bir dosyaya
    # (crash_log.txt) tam stack trace yazıyor, bu KRİTİK bilgi stdout
    # buffer'ından BAĞIMSIZ - kabul edilebilir bir takas.
    python3 main.py 2>&1 | tee "$_LOG"
    _CIKIS_KODU=${PIPESTATUS[0]}
    _BITIS_EPOCH=$(date +%s)
    _BITIS=$(date '+%Y-%m-%d %H:%M:%S')
    _SURE_SN=$((_BITIS_EPOCH - _BASLANGIC_EPOCH))

    {
        echo "=== TUFAN ÇALIŞMA RAPORU (deneme #$_DENEME) ==="
        echo "Başlangıç : $_BASLANGIC"
        echo "Bitiş     : $_BITIS  (çalışma süresi: ${_SURE_SN}sn)"
        echo "Çıkış kodu: $_CIKIS_KODU"
        if [ "$_CIKIS_KODU" -gt 128 ]; then
            _SINYAL=$((_CIKIS_KODU - 128))
            echo "SONUÇ: DIŞARIDAN BİR SİNYALLE ÖLDÜRÜLDÜ (sinyal $_SINYAL - $(kill -l "$_SINYAL" 2>/dev/null || echo bilinmiyor))."
            echo "  -> Sinyal 9 (KILL): işletim sistemi/pencere yöneticisi 'yanıt vermiyor' diyip zorla kapatmış OLABİLİR, ya da elle 'kill -9'/force-quit yapılmış olabilir."
            echo "  -> Sinyal 15 (TERM)/2 (INT): normal/istekli bir kapatma sinyali."
            echo "  -> Sinyal 11 (SEGV)/6 (ABRT): GERÇEK bir çökme (native/Qt seviyesinde) - varsa core dump'a bakılmalı."
        elif [ "$_CIKIS_KODU" -eq 0 ]; then
            echo "SONUÇ: Temiz çıkış (kod 0)."
        else
            echo "SONUÇ: Python seviyesinde hatayla çıktı (kod $_CIKIS_KODU) - crash_log.txt'ye bakın."
        fi
        if grep -q "Sistem kapatılıyor" "$_LOG" 2>/dev/null; then
            echo "NOT: Log'da 'Sistemi Kapat' butonunun tetiklediği İSTEKLİ bir kapatma mesajı VAR."
        else
            echo "NOT: Log'da İSTEKLİ bir kapatma mesajı YOK - pencereden (X butonu) ya da dışarıdan kapatılmış olabilir."
        fi
        echo ""
        echo "=== SON 60 SATIR (bkz. $_LOG tam log için) ==="
        tail -60 "$_LOG"
    } > /home/tufan-yer/Desktop/tufan/crash_rapor_son.txt

    echo ""
    echo ">>> Çalışma raporu yazıldı: /home/tufan-yer/Desktop/tufan/crash_rapor_son.txt"

    if [ "$_CIKIS_KODU" -eq 0 ]; then
        echo ">>> Temiz çıkış - otomatik yeniden başlatma yapılmıyor."
        break
    fi

    if [ "$_SURE_SN" -lt 10 ]; then
        _ARDISIK_HIZLI_COKME=$((_ARDISIK_HIZLI_COKME + 1))
    else
        _ARDISIK_HIZLI_COKME=0
    fi

    if [ "$_ARDISIK_HIZLI_COKME" -ge "$MAKS_ARDISIK_HIZLI_COKME" ]; then
        echo ">>> $MAKS_ARDISIK_HIZLI_COKME kez art arda HIZLI (10sn altı) çöktü - kalıcı bir hata olabilir, OTOMATİK YENİDEN BAŞLATMA DURDURULDU. crash_rapor_son.txt / crash_log.txt kontrol edilmeli."
        break
    fi

    echo ">>> Anormal çıkış (kod $_CIKIS_KODU) - 2sn sonra otomatik yeniden başlatılıyor (deneme #$((_DENEME + 1)))..."
    sleep 2
done

exit "$_CIKIS_KODU"
