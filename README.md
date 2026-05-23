# TMC5160 Low-Level SPI Driver

Niskopoziomowy sterownik w C dla układu TMC5160, napisany pod kątem projektu łodzi solarnej AGH Solar Boat. Kod gada bezpośrednio z rejestrami przez SPI, bez grubych bibliotek.

* **Komunikacja SPI:** Ręczne składanie ramek (5 bajtów) na bazie STM32 HAL i operacji bitowych.
* **Maszyna stanów (FSM):** Blokada ruchu silnika przed pełną inicjalizacją (stany UNINITIALIZED, READY, MOVING itd.).
* **Automatyczna kalibracja (StallGuard2):** Silnik sam jedzie do oporu, wykrywa sprzętowo mechaniczną krańcówkę, zatrzymuje się i zapisuje max zakres kroków. Nie potrzebuje fizycznych przełączników.
* **Diagnostyka błędów:** Funkcja czyta rejestr `DRV_STATUS` i wykrywa na żywo przegrzanie, zwarcie do masy/zasilania czy odłączony kabel.

## Do zrobienia (TODO):
1. Testy na stanowisku i dobranie czułości StallGuard pod obciążeniem.
2. Obsługa enkodera (`ENC_DEVIATION`), żeby pilnować zgubionych kroków.
3. Dodanie Mutexów z FreeRTOS wokół SPI, żeby zapytania z różnych zadań się nie gryzły.
