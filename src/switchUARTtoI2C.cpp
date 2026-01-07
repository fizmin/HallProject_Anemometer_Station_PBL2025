/*
// Poniżej program do zmiany trybu pracy czujnika TF02-Pro z UART na I2C.
// UWAGA: Po wykonaniu tego programu, czujnik przestanie odpowiadać na UART.
// Komunikacja będzie możliwa tylko przez I2C (domyślny adres 0x10).

#include <Arduino.h>

// Definicja sprzętowego portu UART2 dla ESP32
HardwareSerial TFSerial(2); 


 //Funkcja realizuje sekwencję przełączania interfejsu.
 //Komendy są zgodne z dokumentacją protokołu Benewake.
 
void tf02_switch_uart_to_i2c()
{
    // 1) Wejście w tryb konfiguracji (Enter Configuration Mode)
    // Instrukcja informuje procesor sensora, że nadchodzące dane to komendy, a nie parametry pracy.
    uint8_t enter_cfg[] = {0xAA, 0x55, 0xF0, 0x00, 0x01, 0x00, 0x00, 0x02};
    TFSerial.write(enter_cfg, sizeof(enter_cfg));
    delay(50); // Krótka pauza na przetworzenie komendy przez sensor

    // 2) Ustawienie interfejsu wyjściowego na I2C
    // Bajt 0x01 w tej sekwencji odpowiada za aktywację protokołu I2C.
    uint8_t set_i2c[]  = {0x5A, 0x05, 0x0A, 0x01, 0x6A};
    TFSerial.write(set_i2c, sizeof(set_i2c));
    delay(50);

    // 3) Zapisanie ustawień w pamięci nieulotnej (Flash) sensora
    // Bez tej komendy, po resecie zasilania sensor wróciłby do trybu UART.
    uint8_t save_cfg[] = {0x5A, 0x04, 0x11, 0x6F};
    TFSerial.write(save_cfg, sizeof(save_cfg));
    delay(50);

    // 4) Wyjście z trybu konfiguracji (Exit Configuration Mode)
    // Powrót do standardowego trybu pracy (już na nowym interfejsie).
    uint8_t exit_cfg[] = {0xAA, 0x55, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x02};
    TFSerial.write(exit_cfg, sizeof(exit_cfg));
    delay(50);

    // Po wysłaniu tych komend sensor wymaga restartu (tzw. Power-Cycle),
    // aby załadować nowy stos komunikacyjny I2C.
}

void setup() {
    // Inicjalizacja portu szeregowego do debugowania (Monitor szeregowy w PC)
    Serial.begin(115200);

    // Inicjalizacja UART2 do komunikacji z czujnikiem:
    // Domyślne piny dla ESP32: RX=GPIO16, TX=GPIO17.
    // Prędkość 115200 jest domyślna dla fabrycznych czujników TF02-Pro.
    TFSerial.begin(115200, SERIAL_8N1, 16, 17);

    Serial.println("Rozpoczynam przelaczanie TF02-Pro z UART na I2C...");
    
    // Wywołanie funkcji konfiguracyjnej
    tf02_switch_uart_to_i2c();
    
    Serial.println("Komendy wyslane pomyślnie.");
    Serial.println("TERAZ ODŁĄCZ I PODŁĄCZ PONOWNIE ZASILANIE CZUJNIKA.");
    Serial.println("Od teraz czujnik bedzie dostepny pod adresem I2C: 0x10.");
}

void loop() {
    // Pętla główna pozostaje pusta - konfiguracja wykonywana jest tylko raz przy starcie.
}
*/