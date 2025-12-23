HallProject Anemometer Station PBL 2025
Repozytorium zawiera studencki projekt PBL realizowany na Politechnice Śląskiej w 2025 roku. Celem projektu jest opracowanie modułu do pomiaru odległości z użyciem czujników LiDAR, z komunikacją UART/JSON oraz wizualizacją wyników na wyświetlaczu OLED.

Opis projektu
Platformą sprzętową jest płytka z mikrokontrolerem ESP32 (konfiguracja esp32dev w PlatformIO) pracująca w środowisku Arduino.

System wykorzystuje wielokanałowy pomiar z czujników LiDAR (sterowanych przez multiplekser I2C) oraz prezentuje dane na wyświetlaczu OLED 1.5" Waveshare.

Sterowanie odbywa się zewnętrznie przez UART w formacie JSON (komendy z PC) oraz lokalnie za pomocą przycisku, który przełącza stan przekaźnika (np. zasilanie lasera).

Funkcjonalności
Okresowy pomiar odległości z wielu czujników LiDAR i buforowanie wyników w strukturach LidarBuffer.

Obliczanie wartości średnich oraz odchyleń standardowych dla każdego kanału (tablice avgArray[] i stdArray[], zabezpieczone mutexami FreeRTOS).

Wyświetlanie danych pomiarowych oraz stanu systemu na OLED 1.5" (biblioteki DEV_Config, OLED_1in5_B, GUI_Paint, fonts).

Sterowanie przekaźnikiem (np. zasilanie lasera) przyciskiem lokalnym oraz raportowanie zdarzeń na UART.

Obsługa komend JSON po UART, m.in.: rozpoczęcie pomiaru, żądanie danych RAW, test, ustawianie offsetów offsetX i offsetY.

Przykładowe globalne elementy logiki:

Flagi sterujące: measureRequested, testRequested, rawRequested, oledEnabled, relayState.

Zadania FreeRTOS: w osobnych wątkach realizowane są pomiary LiDAR (lidarMeasurementTask) oraz obsługa OLED (oledControlTask).

Sprzęt i biblioteki
Mikrokontroler: ESP32 DevKit (środowisko esp32dev z PlatformIO, framework Arduino).

Czujniki: LiDAR TF02 pracujące w trybie I2C, przełączane przez multiplekser (funkcja tcaSelect(ch) oraz tf02_init_i2c_mode()).

Wyświetlacz: Waveshare 1.5" OLED (moduł B) z przykładów producenta.

Przekaźnik sterowany z pinu RELAY_PIN oraz przycisk podłączony do BUTTON_PIN (wejście z INPUT_PULLDOWN).

Projekt powstał na bazie:

Biblioteki JSON dla C++: https://github.com/nlohmann/json

Przykładów do modułu OLED Waveshare 1.5": https://www.waveshare.com/wiki/1.5inch_OLED_Module_(B)#Run_the_Demo_2

Struktura repozytorium
src/main.cpp – główny plik aplikacji, inicjalizacja sprzętu, obsługa UART/JSON, przycisku i tworzenie zadań FreeRTOS.

src/lidar.* – obsługa czujników LiDAR oraz multipleksera I2C (pomiar odległości, buforowanie próbek).

src/measurement.* – przetwarzanie danych pomiarowych, wyliczanie średnich i odchyleń.

src/display.* – funkcje do rysowania treści na OLED z wykorzystaniem biblioteki Waveshare.

include/config.h – definicje globalnych parametrów (liczba czujników NUM_LIDARS, piny, konfiguracja itp.).

lib/oled/ – pliki konfiguracyjne i sterujące modułem OLED (DEV_Config.h, OLED_1in5_b.h, GUI_Paint.h, fonts.h).

platformio.ini – konfiguracja projektu PlatformIO dla środowiska esp32dev.

Kompilacja i uruchomienie
Zainstaluj PlatformIO (VS Code lub CLI).

Sklonuj repozytorium:

bash
git clone https://github.com/fizmin/HallProject_Anemometer_Station_PBL2025.git
cd HallProject_Anemometer_Station_PBL2025
Otwórz projekt w PlatformIO i wybierz środowisko env:esp32dev.

Podłącz płytkę ESP32 i wgraj firmware (Build + Upload). Prędkość monitora szeregowego: 115200.

Po poprawnym uruchomieniu w terminalu szeregowym powinny pojawić się komunikaty inicjalizacji kanałów LiDAR (CHx: OK/FAIL) oraz komunikat Ready.

Protokół UART / JSON
Komunikacja z komputerem PC odbywa się w formacie JSON przesyłanym przez UART, z prędkością 115200.
Mikrokontroler parsuje bufor znaków aż do wykrycia } jako końca ramki JSON.

Obsługiwane komendy wejściowe:

{"measure": true} – żądanie pomiaru; ustawia flagę measureRequested, budzi OLED i zwraca status:

json
{"status":"OK","cmd":"measure"}
{"offsetX": 0.2} – ustawienie offsetu osi X; odpowiedź zawiera aktualną wartość:

json
{"status":"OK","cmd":"offsetX","value":0.200}
{"offsetY": 0.2} – analogicznie dla osi Y.

{"test": true} – uruchomienie trybu testowego, odpowiedź:

json
{"status":"OK","cmd":"test"}
{"raw": true} – żądanie danych surowych, odpowiedź:

json
{"status":"OK","cmd":"raw"}
W przypadku nieznanej komendy lub błędu parsowania zwracane są komunikaty błędu:

Nieznana komenda:

json
{"status":"error","reason":"unknown_cmd"}
Błąd parsowania JSON:

json
{"status":"error","reason":"json_parse"}
Obsługa lokalna (przycisk i przekaźnik)
Przycisk (podłączony do BUTTON_PIN) przełącza stan przekaźnika RELAY_PIN przy każdym naciśnięciu (zbocze narastające).

W terminalu pojawia się komunikat:

[BUTTON] Relay (laser) ON

[BUTTON] Relay (laser) OFF
w zależności od nowego stanu.

Licencja
Projekt ma charakter dydaktyczny, realizowany w ramach Projektu Bazowego (PBL) na Politechnice Śląskiej. W przypadku wykorzystania kodu w innych projektach należy pamiętać o licencji bibliotek zewnętrznych (nlohmann/json, biblioteki Waveshare).
