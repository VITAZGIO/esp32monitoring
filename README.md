# esp32monitoring — PlatformIO проект

## Структура

```
esp32monitoring/
├── platformio.ini      ← конфиг платы, библиотек и пинов TFT_eSPI
├── src/
│   └── main.cpp         ← вся прошивка (бывший esp32monitoring.ino)
└── README.md
```

## Как открыть в VS Code

1. Установить расширение **PlatformIO IDE** (если ещё не стоит) — Extensions → найти "PlatformIO IDE" → Install.
2. File → Open Folder → выбрать папку `esp32monitoring` (эту, целиком, не файл).
3. PlatformIO сам подтянет `platformio.ini`, скачает нужные версии библиотек и создаст структуру — ничего руками ставить не надо.

## Сборка и заливка

Внизу окна VS Code появится синяя панель PlatformIO:

- ✔ (галочка) — Build
- → (стрелка) — Upload (порт определяется автоматически)
- 🔌 (вилка) — Serial Monitor (115200 бод)

Либо через терминал внутри проекта:

```
pio run          # собрать
pio run -t upload  # собрать и залить
pio device monitor # монитор порта
```

## TFT_eSPI — пины дисплея

Заданы в `platformio.ini` через `build_flags`, редактировать библиотеку не нужно:

| Сигнал | GPIO |
|---|---|
| SCLK | 18 |
| MOSI (SDA) | 23 |
| CS | 5 |
| DC | 2 |
| RST | 4 |
| BL (подсветка) | 15 |

Драйвер указан как `ILI9341_DRIVER`. Если после прошивки экран остаётся чёрным
или показывает "кашу" из цветов — вероятно, реальный чип другой (например
ST7789 или ST7735), тогда в `platformio.ini` нужно заменить
`-D ILI9341_DRIVER=1` на `-D ST7789_DRIVER=1` (или другой) и пересобрать.

## Прочие пины (для справки, уже в коде)

- RF433: RX=27, TX=26
- Реле: 13, 14, 25, 33
- Буззер: 32
- Датчик БП (DS18B20): см. `TEMP_SENSOR_PIN` в `main.cpp`

## Wi-Fi / MQTT

Прямо в `src/main.cpp`:

```cpp
static const char* WIFI_SSID = "...";
static const char* WIFI_PASS = "...";
static const char* MQTT_HOST = "192.168.1.195";
```
