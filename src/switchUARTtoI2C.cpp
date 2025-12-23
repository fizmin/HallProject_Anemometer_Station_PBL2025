//Poniżej program do zmiany TF02-Pro z trybu UART na I2C
/*
#include <Arduino.h>

HardwareSerial TFSerial(2);  // UART2

// Funkcja wysyła komendy: enter config -> ustaw I2C -> save -> exit config
void tf02_switch_uart_to_i2c()
{
    // 1) Wejście w tryb konfiguracji: AA 55 F0 00 01 00 00 02 [web:31]
    uint8_t enter_cfg[] = {0xAA, 0x55, 0xF0, 0x00, 0x01, 0x00, 0x00, 0x02};
    TFSerial.write(enter_cfg, sizeof(enter_cfg));
    delay(50); // krótka pauza na odpowiedź

    // 2) Ustaw interfejs na I2C: 5A 05 0A 01 6A [web:7][web:19]
    uint8_t set_i2c[]  = {0x5A, 0x05, 0x0A, 0x01, 0x6A};
    TFSerial.write(set_i2c, sizeof(set_i2c));
    delay(50);

    // 3) Zapisz ustawienia: 5A 04 11 6F [web:7][web:128]
    uint8_t save_cfg[] = {0x5A, 0x04, 0x11, 0x6F};
    TFSerial.write(save_cfg, sizeof(save_cfg));
    delay(50);

    // 4) Wyjście z trybu konfiguracji: AA 55 F0 00 00 00 00 02 [web:31][web:61]
    uint8_t exit_cfg[] = {0xAA, 0x55, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x02};
    TFSerial.write(exit_cfg, sizeof(exit_cfg));
    delay(50);

    // Opcjonalnie: możesz jeszcze wysłać reset soft, ale najpewniej i tak zrobisz power-cycle
    // lub zrób tu przekaźnikiem / tranzystorem odcięcie VCC na ~1 s.
}

void setup() {
    Serial.begin(115200);

    // UART2: RX=16, TX=17, 115200 8N1 (domyślny dla TF02-Pro) [web:7][web:126]
    TFSerial.begin(115200, SERIAL_8N1, 16, 17);

    Serial.println("Switch TF02-Pro from UART to I2C...");
    tf02_switch_uart_to_i2c();
    Serial.println("Commands sent. Now power-cycle the sensor.");
}

void loop() {
    // tu nic, możesz tylko monitorować
}
*/