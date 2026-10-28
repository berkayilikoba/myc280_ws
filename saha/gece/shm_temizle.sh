#!/bin/bash
# shm_temizle.sh: FastDDS paylaşımlı bellek artıklarını temizle. kill -9 ile kapanan sim'lerden kalan segment/port
# kilitleri yeni düğümleri başlatmada asılı bırakıyordu (AMCL "launched" basmadan bekliyor). Önce `fastdds shm clean`
# (yalnız zombiler); hiçbir süreç fastrtps dosyası kullanmıyorsa hepsini sil (başkasının canlı DDS'ine dokunmaz).
fastdds shm clean >/dev/null 2>&1
for f in /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*; do [ -e "$f" ] && fuser "$f" >/dev/null 2>&1 && exit 0; done
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null; exit 0
