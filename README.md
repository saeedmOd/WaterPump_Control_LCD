<img width="1696" height="2528" alt="e3d046e151fbcc384056a129" src="https://github.com/user-attachments/assets/0844ee64-aa1a-4b20-b7b6-b111ff6ff5f0" />

# Water Pump Control System — Arduino Uno

Smart 2-pump water level control system for Arduino Uno with an LCD keypad shield,
a DS1302 RTC, safety protections, per-pump runtime counters and an EEPROM event log.

---

## Features

- **3 operating modes** — OFF / AUTO / MANUAL
- **2 control strategies** in AUTO mode — by **sensors** (well + tank) or by **timed rotation** between pumps
- **Dry-run protection** — configurable grace period (0–30 min), then forced stop + `DRY FAULT`
- **Low-voltage protection** — threshold 9–13 V (divider on A3), 2 s confirm, 5 s recovery hysteresis
- **Interlock** — the two pumps can never run simultaneously (`BOTH-FAULT` latch)
- **Runtime counters** — per-pump operating hours, saved to EEPROM every minute
- **Maintenance reminder** — alert when a pump reaches the configured hour limit (default 1000 h)
- **Event log** — last 15 events with RTC time/date, stored in EEPROM
- **Fault indicators** — shown with priority on the LCD line 2 + buzzer alarm
- **Full menu navigation** — knob-free control with the 6 shield push-buttons
- **Serial command set** — for monitoring and configuration (9600 baud)

---

## Hardware

| Component | Qty | Notes |
|---|---|---|
| Arduino Uno | 1 | ATmega328P |
| D1 Robot LCD Keypad Shield | 1 | 16x2 LCD + 6 buttons |
| DS1302 RTC module | 1 | 3-wire (DAT, CLK, RST/CE) + CR2032 battery |
| Relay modules (active-LOW) | 2 | one per pump |
| Float/level switch (well) | 1 | D2 |
| Float/level switch (tank) | 1 | D3 (interrupt INT1) |
| Resistive divider 10k + 4.7k | 1 | battery voltage -> A3 |
| Buzzer | 1 | D13 (via transistor/driver if needed) |
| LED | 1 | D11, AUTO mode indicator |

---

## Wiring

```
Arduino Uno
┌──────────────────────────────┐
│ D2  <── Well float switch ──>│  (pin -> switch -> GND, 10k pull-up to 5V)
│ D3  <── Tank float switch ──>│  (pin -> switch -> GND, 10k pull-up to 5V, INT1)
│ A1  ── Relay IN1 (Pump 1)    │  (relay modules are active-LOW)
│ A2  ── Relay IN2 (Pump 2)    │
│ A3  ── 12V+ ─[10k]─┬─ A3 ─┐ │
│                    │  [4.7k] │
│                    └──── GND │
│ D11 ── AUTO LED             │
│ D13 ── Buzzer               │
│ DS1302: DAT->A4, CLK->A5, RST->D12, VCC->5V, GND->GND
│ LCD Shield: uses only D4-D10 + A0 (push-buttons)
└──────────────────────────────┘
```

> **Relay note:** the code drives the relays with `PUMP_ON = LOW` (active-LOW modules).
> If your relay modules are active-HIGH, invert that constant in `src/main.cpp`.

---

## Pin Map

| Pin | Function |
|---|---|
| 2 | well level sensor |
| 3 | tank level sensor (INT1) |
| A1 / A2 | pump 1 / pump 2 (active-LOW relay) |
| A3 | battery voltage (resistive divider) |
| A4 / A5 / 12 | DS1302 DAT / CLK / RST |
| 11 | AUTO mode LED |
| 13 | buzzer |
| 4–10 | LCD shield (fixed by hardware) |
| A0 | shield buttons |

---

## EEPROM Layout

| Address | Content |
|---|---|
| 0 | mode |
| 3 | control type (0 sensors / 1 time) |
| 12 / 16 | pump 1 / pump 2 runtime seconds (unsigned long) |
| 60 / 70 | pump 1 / pump 2 timers |
| 90 | dry-run grace minutes |
| 91 | low-voltage threshold |
| 92 | maintenance hours (uint16_t) |
| 94 | event log write index |
| 95 | event log wrapped flag |
| 100–189 | event log (15 x 6 bytes) |

---

## Menu (LCD + Shield Buttons)

`Mode → Control → DryRun → Voltage → PumpTime → RunTime → Events → Faults → Clock → Back`

- **UP / DOWN** — navigate items
- **LEFT / RIGHT** — change value (also selects pump on the main screen)
- **SELECT** — confirm / enter
- Status screen shows: mode + pump states, then priority fault / well-empty / clock

---

## Serial Commands (9600 baud)

| Command | Description |
|---|---|
| `status` | current status |
| `mode0` / `mode1` / `mode2` | OFF / AUTO / MANUAL |
| `pump1` / `pump2` / `pumpoff` | manual pump select |
| `control_sensor` / `control_time` | switch control type |
| `drymin N` | dry-run grace minutes, 0–30 (0=off) |
| `voltlow V` | low-voltage threshold, 0–13 (0=off) |
| `maint H` | maintenance interval hours, 0–3000 |
| `runtime` | pump runtime hours |
| `runtime_pump P` | reset runtime of pump P (1/2) |
| `events` | event log (newest first) |
| `faults` / `fault_clear` | show / clear faults |
| `pump_time,M,h,m,s` | set run time of pump M |
| `set_time,H,M,S,D,Mo,Y` | set date and time |
| `help` | command list |

Example: `set_time,15,28,0,23,9,26`

---

## Build & Upload (PlatformIO)

```bash
pio run                # compile
pio run -t upload      # upload to board
pio device monitor     # serial monitor @ 9600
```

`platformio.ini` dependencies: `makuna/Rtc`, `arduino-libraries/LiquidCrystal`.

---

## First Boot

If the DS1302 has no valid time (new module / dead battery), the code sets a default
`2026-01-01 12:00` and logs it. Set the correct time via the `Clock` menu or
`set_time,H,M,S,D,Mo,Y`. A valid CR2032 keeps the clock running while powered off.

---

# نظام التحكم بمضخات المياه — آردوينو أونو

نظام ذكي للتحكم بمضختي مياه باستخدام آردوينو أونو مع شاشة LCD ولوحة أزرار D1 Robot،
وساعة DS1302، وحمايات متعددة، وعداد ساعات تشغيل لكل مضخة، وسجل أحداث في الـ EEPROM.

## المميزات

- **ثلاثة أوضاع تشغيل** — إيقاف / تلقائي / يدوي
- **طريقتان للتحكم التلقائي** — بالحساسات (بئر + خزان) أو بالتناوب الزمني بين المضختين
- **حماية الجفاف** — فترة سماح (0–30 دقيقة) ثم إيقاف إجباري مع إنذار `DRY FAULT`
- **حماية الجهد المنخفض** — عتبة 9–13 فولت (مقسم على A3) مع تأكيد ثانيتين
- **منع التزامن** — لا يمكن أن تعمل المضختان معًا (`BOTH-FAULT`)
- **عداد ساعات التشغيل** — لكل مضخة، يُحفظ في EEPROM كل دقيقة
- **تنبيه الصيانة** — إنذار عند بلوغ الحد (افتراضي 1000 ساعة)
- **سجل أحداث** — آخر 15 حدثًا مع الوقت والتاريخ في EEPROM
- **مؤشرات أعطال** — على سطر الشاشة الثاني حسب الأولوية + جرس إنذار
- **قائمة كاملة** — تحكم بستة أزرار الشاشة
- **أوامر سيريال** — للمراقبة والإعداد (9600)

## التوصيل

```
آردوينو أونو
┌──────────────────────────────┐
│ D2  ─ حساس البئر (طفو)       │  (دبوس ← حساس ← GND مع 10k سحب لـ 5V)
│ D3  ─ حساس الخزان (طفو)       │  (دبوس ← حساس ← GND مع 10k سحب لـ 5V، مقاطعة INT1)
│ A1  ─ ريلاي المضخة 1          │  (الريلايات من نوع active-LOW)
│ A2  ─ ريلاي المضخة 2          │
│ A3  ─ 12V ─[10k]─┬─ A3       │
│                  ├─[4.7k]──GND│
│ D11 ─ مؤشر الوضع التلقائي    │
│ D13 ─ الجرس                 │
│ DS1302: DAT→A4, CLK→A5, RST→D12, VCC→5V, GND→GND
│ الشاشة تستخدم فقط D4–D10 و A0 (الأزرار)
└──────────────────────────────┘
```

> **ملاحظة الريلاي:** الكود يحرك الريلاي بـ `PUMP_ON = LOW` (وحدات active-LOW).
> إذا كانت وحدتك من النوع عالي الحساسية active-HIGH، اعكس الثابت في `src/main.cpp`.

## أوامر السيريال

| الأمر | الوصف |
|---|---|
| `status` | الحالة الحالية |
| `mode0` / `mode1` / `mode2` | إيقاف / تلقائي / يدوي |
| `pump1` / `pump2` / `pumpoff` | اختيار مضخة يدويًا |
| `control_sensor` / `control_time` | تبديل نوع التحكم |
| `drymin N` | دقائق الجفاف (0–30، 0=مغلق) |
| `voltlow V` | عتبة الجهد (0–13، 0=مغلق) |
| `maint H` | ساعات الصيانة (0–3000) |
| `runtime` | ساعات تشغيل المضخات |
| `runtime_pump P` | تصفير تشغيل مضخة (1/2) |
| `events` | سجل الأحداث |
| `faults` / `fault_clear` | عرض / مسح الأعطال |
| `pump_time,M,h,m,s` | ضبط وقت مضخة M |
| `set_time,H,M,S,D,Mo,Y` | ضبط التاريخ والوقت |
| `help` | قائمة الأوامر |

مثال: `set_time,15,28,0,23,9,26`

## البناء والرفع (PlatformIO)

```bash
pio run                # ترجمة
pio run -t upload      # رفع إلى اللوحة
pio device monitor     # شاشة السيريال @ 9600
```

## أول إقلاع

إذا لم تكن لساعة DS1302 قيمة صحيحة (وحدة جديدة أو بطارية فارغة)، يضبط الكود افتراضيًا
`2026-01-01 12:00`. اضبط الوقت من قائمة `Clock` أو الأمر `set_time,H,M,S,D,Mo,Y`.
بطارية CR2032 سليمة تحافظ على الوقت عند انقطاع الكهرباء.

---

## License

MIT — see [LICENSE](LICENSE) (add if desired).
