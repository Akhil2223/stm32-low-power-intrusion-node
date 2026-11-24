# STM32 Low-Power Intrusion Node

A FreeRTOS security/intrusion node on the **STM32 NUCLEO-L476RG**. It watches a door, a laser tripwire and two tamper sensors, runs an alarm state machine with an entry delay, and drives an RGB status LED, a buzzer and a relay "lock". Between events the MCU sleeps in **STOP2** and wakes only on a sensor interrupt or its low-power timer. A task-supervised **independent watchdog** resets the chip if any task hangs.

**Status:** sensing, alarm logic, outputs, watchdog and STOP2 low power all work on hardware. NEC IR remote control is next.

## Highlights

- **Interrupt → queue → task** design: ISRs only post an event ID; all work happens in FreeRTOS tasks.
- **Tripwire on the analog comparator (COMP2)** instead of the ADC, so the beam is watched in hardware and can wake the chip from STOP2.
- **STOP2 tickless idle** with a custom `vPortSuppressTicksAndSleep()` that uses **LPTIM1 on the 32.768 kHz LSE** as the wake-up timer and corrects the RTOS tick with `vTaskStepTick()`.
- **IWDG with per-task check-ins**: a supervisor task only refreshes the watchdog when every task has checked in through an event-flags group. The reset cause is logged on boot.
- **Idle MCU current cut from 19.3 mA to a STOP2 floor under 1 µA** (DISARMED, door open), measured on the JP6 (IDD) jumper. See [Low power](#low-power).

## Hardware

- NUCLEO-L476RG (Cortex-M4F). System clock: **MSI 4 MHz**, regulator voltage scale 2. LSE 32.768 kHz crystal for LPTIM1.
- Sensors from a 37-in-1 kit: reed switch + magnet (door), laser + photoresistor module (tripwire), spring shock switch and mercury tilt switch (tamper)
- KY-016 RGB LED, KY-012 active buzzer, KY-019 relay module (simulated door lock, 5 V coil)
- Debug log over the ST-LINK virtual COM port (USART2, 115200 baud)

## Pin map

| Signal | Pin | Nucleo | Config |
|---|---|---|---|
| Door reed switch | PA10 | D2 | EXTI10, internal pull-up, both edges |
| Tripwire photoresistor | PB4 | D5 | COMP2 non-inverting input, ¾ VREFINT, high hysteresis, ULP |
| Relay (lock) | PB10 | D6 | GPIO out, active high |
| Shock switch (tamper) | PA8 | D7 | EXTI8, internal pull-up, falling edge |
| Tilt switch (tamper) | PA9 | D8 | EXTI9, internal pull-up, both edges |
| LED red / green / blue | PC7 / PB6 / PA7 | D9 / D10 / D11 | GPIO out |
| Buzzer | PA6 | D12 | GPIO out |
| User button (arm/disarm) | PC13 | B1 | EXTI13 (temporary, until the IR remote) |
| UART TX / RX | PA2 / PA3 | — | USART2 |
| Low-power timer | LPTIM1 | — | LSE clock, STOP2 wake-up |
| Watchdog | IWDG | — | LSI, prescaler 64, reload 4095 (~8 s) |

### Hardware notes

- **Check kit modules with a meter before wiring.** Several of these boards share one PCB where a fixed 10 kΩ sits between the middle and "–" pins and the sensor sits between S and "–", which doesn't match the labels. Wired as labeled, the photoresistor's S pin reads 0 V forever. Here it's wired S → 3V3, middle → GND, "–" → D5, which gives ~3.2 V with the laser on and ~0.5 V without.
- A plain GPIO interrupt on the photoresistor chattered because the "beam on" level sat near the input threshold. COMP2 with a 0.9 V reference and hysteresis fixed it.
- Reed, shock and tilt use the MCU's internal pull-ups; their modules' middle pins are left unconnected.

## FreeRTOS design

| Object | Type | Role |
|---|---|---|
| `alarmTask` | Task, AboveNormal | Owns the alarm state machine; the only task that changes state |
| `sensorTask` | Task, Normal | Debounces raw sensor events and turns them into alarm events |
| `outputTask` | Task, Low | Drives the RGB LED, buzzer and relay for the current state |
| `wdgTask` | Task, High | Refreshes the IWDG only when every task has checked in |
| `sensorQueue` | Queue | ISR → sensorTask (raw events) |
| `alarmQueue` | Queue | sensorTask / timer → alarmTask |
| `entryTimer` | One-shot software timer | 10 s entry delay |
| `uartMutex` | Mutex | Serializes UART log output |
| `wdgEvents` | Event flags | One check-in bit per task |

```
ISRs (EXTI / COMP2) --> sensorQueue --> sensorTask --> alarmQueue --> alarmTask --> UART log
                                                                          |
                                                    state + thread flag --+--> outputTask (LED, buzzer, relay)
entryTimer (10 s) -----------------------------------> alarmQueue

sensorTask / alarmTask / outputTask --check-in bits--> wdgEvents --> wdgTask --> IWDG refresh
idle --> vPortSuppressTicksAndSleep --> LPTIM1 alarm --> STOP2 --> vTaskStepTick
```

ISRs (`HAL_GPIO_EXTI_Callback`, `HAL_COMP_TriggerCallback`) only push an event ID into a queue, and every ISR that calls FreeRTOS runs at NVIC priority 5 (the RTOS syscall limit).

**Debouncing in `sensorTask`:** the door and tripwire wait 20 ms, then re-read. Tilt waits 50 ms for the mercury to settle. Shock uses a 1 s lockout and the button a 300 ms lockout (on the RTOS tick, which stays correct across sleep). A change is reported only when the stable level differs from the last one.

## Alarm state machine

```
DISARMED --button--> ARMED
ARMED --button--> DISARMED
ARMED --door open--> ENTRY_DELAY (starts 10 s timer)
ARMED --tripwire broken / shock / tilt--> ALARM
ENTRY_DELAY --button--> DISARMED
ENTRY_DELAY --timer expires / shock / tilt--> ALARM
ALARM --button--> DISARMED
```

Shock and tilt only alarm while ARMED or in ENTRY_DELAY. Real panels usually treat tamper as a 24-hour zone that alarms even when disarmed; this build doesn't do that yet (see Roadmap).

The tripwire is ignored during the entry delay, since you walk through it after opening the door (a "follower" zone, like real alarm panels).

| State | LED | Buzzer | Relay |
|---|---|---|---|
| DISARMED | Green 30 ms blip every 1 s | Off | Off (unlocked) |
| ARMED | Blue 30 ms blip every 1 s | Off | On (locked) |
| ENTRY_DELAY | Blue blink, 250 ms | Beep, 250 ms | On |
| ALARM | Red blink, 100 ms | Beep, 100 ms | On |

The lock is **fail-safe**: on power loss the relay drops out and the door unlocks. `alarmTask` wakes `outputTask` with a thread flag on every state change, so outputs react right away. When nothing is blinking, `outputTask` sleeps instead of polling.

Every event and state change is logged over UART:

```
Security node ready
Reset cause: normal
>>> State: DISARMED
[DISARMED] Button
>>> State: ARMED
[ARMED] Door OPEN
>>> State: ENTRY DELAY (10 s)
[ENTRY DELAY (10 s)] Entry delay expired
>>> State: *** ALARM ***
```

## Watchdog

The IWDG (~8 s timeout) is only refreshed by `wdgTask`, and only after `sensorTask`, `alarmTask` and `outputTask` have each set their bit in `wdgEvents` within the last 2 s. Every task waits with a 1 s timeout instead of forever, so it checks in even when nothing is happening.

If a task hangs, `wdgTask` logs which check-ins are missing and stops refreshing, and the chip resets. On boot the firmware reads `RCC_FLAG_IWDGRST` and prints the reset cause.

Tested by setting `WDG_TEST_HANG 1`, which makes a knock spin `sensorTask` forever:

```
!!! Simulating sensorTask hang
WDG: missed check-in, mask 0x05
...
Security node ready
Reset cause: WATCHDOG
```

`0x05` = sensorTask + outputTask: the spinning task also starves the lower-priority output task.

## Low power

### How it sleeps

`configUSE_TICKLESS_IDLE = 2`, with a custom `vPortSuppressTicksAndSleep()`:

1. Stop SysTick and confirm with `eTaskConfirmSleepModeStatus()` that nothing became ready.
2. Arm **LPTIM1** (LSE, 32.768 kHz) to fire when the next task is due (capped at ~2 s, the 16-bit limit: 65535 / 32768).
3. Pause the HAL tick and enter **STOP2**. Any EXTI (door, shock, tilt, button), COMP2 (tripwire) or LPTIM1 wakes it.
4. Read how long it actually slept from LPTIM1 and call `vTaskStepTick()`.
5. Restart SysTick. The system clock wakes up on MSI, so nothing needs restoring.

### Measurements

MCU current measured on the **JP6 (IDD)** jumper with a handheld meter in series, system idle. JP6 covers the MCU and anything driven from its pins (LEDs, relay input, the internal pull-ups), not the sensors or the relay coil. Step 0 includes the solid LED; step 2b onward has it unplugged unless noted. The STOP2 floor in step 4 applies with the reed and tilt switches open (door open, tilt tipped); each closed switch adds ~80 µA through its internal pull-up (see below).

| Step | Change | DISARMED | ARMED |
|---|---|---|---|
| 0 | Baseline: 80 MHz PLL, never sleeps, solid LED | 19.3 mA | – |
| 1 | MSI 4 MHz, regulator scale 2 | 5.7 mA | 8.4 mA |
| 2 | + tickless idle (WFI) | 5.8 mA | 8.5 mA |
| 2b | Same, LED unplugged (MCU only) | 0.9 mA | 4.5 mA |
| 3 | + LED heartbeat instead of solid | 0.8 mA | 4.5–5 mA |
| 4 | **+ STOP2 tickless idle via LPTIM1** | **sleep floor below the meter's 1 µA resolution; ~1 Hz wake-ups** | 3.6–4.1 mA |

What the numbers say:

- Lowering the clock gave the first big win. At 4 MHz the core only draws ~0.5 mA, so WFI sleep alone barely shows.
- The solid LED was ~5 mA. A short heartbeat blip keeps the state visible for a fraction of that.
- STOP2 takes the idle floor from ~0.8 mA into the µA range. A handheld meter can't average the 1 Hz wake-up pulses; a real average needs an energy monitor (e.g. ST X-NUCLEO-LPM01A).
- ARMED is dominated by the relay module's input (~3.6 mA of transistor base drive from the GPIO). A battery design would use a logic-level MOSFET or a latching relay.
- Internal pull-ups (~40 kΩ) leak ~80 µA each through a closed switch (reed with the magnet near, tilt upright), so with the door closed the idle floor is set by the pull-ups (~160 µA), not by STOP2. External high-value pull-ups or periodic sampling would remove that.

## Build and flash

1. Open STM32CubeIDE and choose **File → Import → Existing Projects into Workspace**, then select this folder.
2. Build (the HAL, CMSIS and FreeRTOS sources are included).
3. In **Run/Debug Configurations → Debugger**, set **Reset behaviour = Connect under reset**. The chip spends most of its time in STOP2, and the debugger can't attach to it otherwise.
4. Flash to the NUCLEO-L476RG and open a terminal on the ST-LINK COM port at 115200 baud.

Compile-time switches in `main.c`:

- `BUZZER_ENABLE` — set to 0 to mute the buzzer while testing
- `WDG_TEST_HANG` — set to 1 to make a knock hang `sensorTask` and test the watchdog

`security node.ioc` holds the CubeMX configuration. All application code is inside `USER CODE` blocks, so regenerating from CubeMX keeps it (with **Keep User Code when re-generating** ticked).

## Roadmap

- [x] Door, tripwire (COMP2), shock and tilt sensing with interrupt → queue → task flow
- [x] Alarm state machine with a software-timer entry delay
- [x] RGB status LED, buzzer patterns, mutex-protected UART logging
- [x] Relay as a simulated, fail-safe door lock
- [x] Independent watchdog with per-task check-ins and reset-cause logging
- [x] Clock reduction, LED heartbeat, tickless idle
- [x] STOP2 tickless idle with LPTIM1 on the 32.768 kHz LSE
- [x] MCU current measured on the JP6 IDD jumper, before and after
- [ ] NEC IR remote arm/disarm, decoded by hand with timer input capture (no library)
- [ ] 24-hour tamper zone (shock/tilt alarm even when disarmed)
- [ ] Longer watchdog/check-in period and pulsed laser to cut wake-ups further
