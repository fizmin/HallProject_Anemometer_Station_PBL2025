# HallProject Anemometer Station PBL 2025

Repozytorium zawiera studencki projekt PBL z Politechniki Śląskiej (2025) – moduł pomiaru odległości z użyciem czujników LiDAR na ESP32 z wyświetlaczem OLED i komunikacją UART/JSON. 

## Opis

- ESP32 (PlatformIO, `esp32dev`, Arduino).  
- Kilka czujników LiDAR przez multiplekser I2C, wyniki na OLED 1.5" Waveshare.  
- Sterowanie z PC przez UART/JSON oraz lokalnie przyciskiem (przekaźnik/laser).

## Struktura

- `src/main.cpp` – logika aplikacji, UART/JSON, przycisk, FreeRTOS.  
- `src/lidar.*` – obsługa LiDAR i multipleksera I2C.  
- `src/measurement.*` – obliczenia średnich i odchyleń.  
- `src/display.*` – obsługa OLED.  
- `platformio.ini` – konfiguracja PlatformIO (ESP32).

## Uruchomienie

1. Zainstaluj PlatformIO.  
2. `git clone https://github.com/fizmin/HallProject_Anemometer_Station_PBL2025.git`  
3. Otwórz projekt, wybierz `env:esp32dev`, zbuduj i wgraj.  
4. Monitor szeregowy: 115200, po starcie pojawia się `Ready`.

## UART / JSON (skrót)

Przykładowe komendy:

{"measure": true}
{"offsetX": 0.2}
{"offsetY": 0.2}
{"test": true}
{"raw": true}

Odpowiedź ma pola `status` i `cmd` (np. `{"status":"OK","cmd":"measure"}`).

## Licencja

Projekt dydaktyczny PBL; zwróć uwagę na licencje zewnętrznych bibliotek (nlohmann/json, Waveshare).
Biblioteki JSON dla C++: https://github.com/nlohmann/json
Moduł OLED Waveshare 1.5": https://www.waveshare.com/wiki/1.5inch_OLED_Module_(B)#Run_the_Demo_2
