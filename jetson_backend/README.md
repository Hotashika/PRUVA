# NJORD Jetson Bridge Backend

Bu klasör, Jetson tarafındaki `orange_cube_bridge` node'una eklenecek GUI backend kodunu içerir.

Amaç şudur:

```text
GUI -> HTTP :8000 -> orange_cube_bridge -> MAVLink -> Pixhawk
```

Yani GUI görev dosyasını veya komutu Jetson'a gönderir. Jetson'daki bridge node zaten Pixhawk'a MAVLink ile bağlı olduğu için, bu backend aynı MAVLink bağlantısını kullanarak Pixhawk'a görev yükler veya komut gönderir.

Önemli: Bridge node zaten Pixhawk'a bağlıysa Pixhawk için ikinci bir seri MAVLink bağlantısı açılmamalıdır. `njord_bridge_http_backend.py`, bridge node içindeki mevcut `self.master` bağlantısını kullanır.

## bridge/bridge_node.py İçine Eklenecekler

Importların olduğu yere ekle:

```python
from jetson_backend.njord_bridge_http_backend import attach_http_backend
```

`OrangeCubeBridgeNode.__init__` içinde timerlar oluşturulduktan sonra ekle:

```python
attach_http_backend(self, host="0.0.0.0", port=8000)
```

`_read_mavlink_messages()` içinde MAVLink mesajları işlenirken şu bloğu ekle:

```python
elif msg_type in (
    "MISSION_COUNT",
    "MISSION_ITEM",
    "MISSION_ITEM_INT",
    "MISSION_REQUEST",
    "MISSION_REQUEST_INT",
    "MISSION_ACK",
):
    if hasattr(self, "http_backend_push_mission_message"):
        self.http_backend_push_mission_message(msg)
```

Bu blok çok önemlidir. Görev yükleme MAVLink'te karşılıklı istek-cevap şeklinde çalışır. Pixhawk waypointleri kendisi ister; backend de bu isteklere göre waypointleri gönderir.

Görev yükleme akışı:

```text
GUI -> /api/mission/upload_txt
backend -> MISSION_CLEAR_ALL
backend -> MISSION_COUNT
Pixhawk -> MISSION_REQUEST_INT veya MISSION_REQUEST
backend -> MISSION_ITEM_INT veya MISSION_ITEM
Pixhawk -> MISSION_ACK
backend -> MISSION_REQUEST_LIST
Pixhawk -> MISSION_COUNT + görev noktaları
backend -> dönen koordinatları doğrular
GUI <- yüklenmiş/doğrulanmış waypoint listesi
```

Bu yüzden `MISSION_COUNT`, `MISSION_ITEM`, `MISSION_ITEM_INT`, `MISSION_REQUEST`, `MISSION_REQUEST_INT` ve `MISSION_ACK` mesajları `_read_mavlink_messages()` içinden `self.http_backend_push_mission_message(msg)` fonksiyonuna aktarılmalıdır.

Bu aktarım yapılmazsa TXT görev yükleme zaman aşımına düşer. Çünkü backend Pixhawk'ın "şu waypointi gönder" isteğini göremez.

Hazır örnek patch dosyası:

```text
jetson_backend/bridge_node_integration.patch
```

## GUI'nin Kullandığı Endpointler

```text
POST /api/mission/upload_txt
POST /api/mission/start
GET  /api/mission/current
POST /api/mission/current
POST /api/pixhawk/arm
POST /api/pixhawk/set_mode
POST /api/mission/stop
GET  /health
```

## Jetson Üzerinde Test

Jetson'da backend çalışıyor mu kontrol etmek için:

```bash
curl http://127.0.0.1:8000/health
curl http://127.0.0.1:8000/api/mission/current
```

GUI bilgisayarından test ederken `127.0.0.1` yerine Jetson IP adresini yaz:

```text
http://JETSON_IP:8000/health
```

## Kısa Özet

Eğer Pixhawk doğrudan PC telemetrisine bağlıysa GUI Pixhawk'a doğrudan MAVLink ile erişebilir.

Eğer Pixhawk Jetson'a bağlıysa GUI Pixhawk'ı doğrudan göremez. Bu durumda GUI, Jetson'daki bu backend'e istek gönderir. Backend de bridge node'un mevcut MAVLink bağlantısı üzerinden Pixhawk'a görev veya komut gönderir.
