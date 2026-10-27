#!/bin/bash
# oda_kur.sh: gerçek odanın 3B sim dünyasını kayıtlardan baştan üretir ve limo_car paketine yerleştirir.
#   2B yoğun harita (lidar) -> duvar/mobilya ayrımı (oda_yapi) ; kamera sahneleri -> renkli TSDF mesh (oda_tsdf)
#   -> sade çarpışma mesh'i ; duvar/zemin dokuları (oda_doku) ; SDF dünyası (oda_dunya) ; model://lab_oda_3b paketi
# Kamera pozları lidar haritasına göre düzeltilir (kalib_harita.py: dx +9 cm, dy +1 cm, dyaw -1.0°; v2b +8/+2/-1.5).
# Kullanım: [TAVAN=2.7] [HIZLI=1] oda_kur.sh [sahne ...]   (varsayılan: v2a_lidar v2b_lidar rs720_kapsama_lidar)
set -e
cd "$(dirname "$(readlink -f "$0")")"
WS=$(readlink -f ../..); V=$HOME/limo_veri/vlmaps_sahne; C=cikti/kur
SAHNE=${@:-$V/v2a_lidar $V/v2b_lidar $V/rs720_kapsama_lidar}
HARITA=$WS/src/limo_bringup/maps/lab_oda.yaml; TAVAN=${TAVAN:-2.7}
MODEL=$WS/src/limo_car/models/lab_oda_3b
mkdir -p $C
if [ -z "$HIZLI" ]; then   # HIZLI=1: yalnız dünya + paket (yapı/mesh/doku çıktıları varsa)
python3 oda_yapi.py $HARITA $C/oda_yapi
python3 oda_tsdf.py $C/tsdf $SAHNE --her 2 --ucgen 150000 --renk_yeniden --no-ince  # bulanık süzgeci, tutarlılık, insan maskesi varsayılan açık (ince hizalama kapalı: lidar çerçevesinden kaydırıyor, ölçüldü)
python3 oda_dunya.py --sadelestir $C/tsdf/oda_mesh.obj $C/tsdf/oda_carpisma.obj --ucgen 20000
python3 oda_doku.py $C/doku --yapi $C/oda_yapi.yaml $SAHNE --her 6 --tavan $TAVAN
fi
python3 oda_dunya.py --yapi $C/oda_yapi.yaml --mesh_dizin $C/tsdf --doku_dizin $C/doku --tavan $TAVAN \
  --carpisma_mesh $PWD/$C/tsdf/oda_carpisma.obj --cikti $C/lab_oda_3b.world
# paket: varlıklar model dizinine, dünya mutlak yollar yerine model:// ile
rm -rf $MODEL; mkdir -p $MODEL/meshes
cp $C/tsdf/oda_mesh_kesik.obj $C/tsdf/oda_mesh.mtl $C/tsdf/oda_carpisma.obj $C/tsdf/zemin_doku.obj $C/tsdf/zemin_doku.mtl $C/tsdf/zemin.png $MODEL/meshes/
cp -r $C/doku/materials $MODEL/
cat > $MODEL/model.config <<XML
<?xml version="1.0"?>
<model><name>lab_oda_3b</name><version>1.0</version><sdf version="1.6">model.sdf</sdf>
<description>Gerçek laboratuvar odası (27.10.2026 kayıtları): varlık deposu; dünya limo_car/worlds/lab_oda_3b.world</description></model>
XML
echo '<?xml version="1.0"?><sdf version="1.6"><model name="lab_oda_3b_varlik"><static>true</static><link name="l"/></model></sdf>' > $MODEL/model.sdf
sed -e "s#file://$PWD/$C/tsdf/#model://lab_oda_3b/meshes/#g" -e "s#file://$PWD/$C/doku/materials#model://lab_oda_3b/materials#g" \
    $C/lab_oda_3b.world > $WS/src/limo_car/worlds/lab_oda_3b.world
if grep -q "file://" $WS/src/limo_car/worlds/lab_oda_3b.world; then echo "UYARI: dünyada mutlak yol kaldı"; grep -n "file://" $WS/src/limo_car/worlds/lab_oda_3b.world | head; fi
du -sh $MODEL; echo "dünya: $WS/src/limo_car/worlds/lab_oda_3b.world"
