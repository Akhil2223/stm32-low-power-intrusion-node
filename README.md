# STM32 Low-Power Intrusion Node

A FreeRTOS security/intrusion node on the **STM32 NUCLEO-L476RG**. It watches a door, a light tripwire and tamper sensors, runs an alarm state machine with an entry delay, and drives an RGB status LED and a buzzer. The end goal is a battery-style node that sleeps in STOP2 and wakes only on sensor events.

**Status: in progress.** The sensing, alarm logic and outputs work on hardware. Low-power mode, IR remote control, the relay lock and the watchdog are next.

## Hardware

- NUCLEO-L476RG (Cortex-M4F, 80 MHz from HSI + PLL)
- Sensors from a 37-in-1 kit: reed switch + magnet, laser + photoresistor module, shock switch, tilt switch
- RGB LED module, active buzzer
- Debug log over the ST-LINK virtual COM port (USART2, 115200 baud)

## Pin map

| Signal | Pin | Config |
|---|---|---|
| Door reed switch | PA10 | EXTI, both edges |
| Shock switch (tamper) | PA8 | EXTI |
| Tilt switch (tamper) | PA9 | EXTI, both edges |
| Tripwire photoresistor | PB4 | COMP2 non-inverting input |
| User button (arm/disarm) | PC13 | EXTI |
| LED red / green / blue | PC7 / PB6 / PA7 | GPIO out |
| Buzzer | PA6 | GPIO out |
| UART TX / RX | PA2 / PA3 | USART2 |

The tripwire doesn't use the ADC. The photoresistor feeds **COMP2** in ultra-low-power mode against ¾ VREFINT with high hysteresis, and the comparator interrupts on both edges. That keeps the beam monitored without polling, and it can work in STOP mode later.

## FreeRTOS design

| Object | Type | Role |
|---|---|---|
| `alarmTask` | Task (highest priority) | Owns the alarm state machine; the only task that changes state |
| `sensorTask` | Task | Debounces raw sensor events and turns them into alarm events |
| `outputTask` | Task (low priority) | Drives the RGB LED and buzzer patterns for the current state |
| `sensorQueue` | Queue | ISR → sensorTask (raw events) |
| `alarmQueue` | Queue | sensorTask / timer → alarmTask |
| `entryTimer` | One-shot software timer | 10 s entry delay |
| `uartMutex` | Mutex | Serializes UART log output |

ISRs (`HAL_GPIO_EXTI_Callback`, `HAL_COMP_TriggerCallback`) only push an event ID into a queue. All the real work happens in tasks.

**Debouncing in `sensorTask`:** the door and tripwire wait 20 ms, then re-read the pin. Tilt waits 50 ms. Shock uses a 1 s lockout and the button a 300 ms lockout. A change is reported only when the stable level differs from the last one.

## Alarm state machine

```
DISARMED --button--> ARMED
ARMED --button--> DISARMED
ARMED --door open--> ENTRY_DELAY (starts 10 s timer)
ARMED --tripwire broken / shock / tilt--> ALARM
ENTRY_DELAY --button--> DISARMED
ENTRY_DELAY --timer expires / tamper--> ALARM
ALARM --button--> DISARMED
```

| State | LED | Buzzer |
|---|---|---|
| DISARMED | Green | Off |
| ARMED | Blue | Off |
| ENTRY_DELAY | Blue blink, 250 ms | Beep, 250 ms |
| ALARM | Red blink, 100 ms | Beep, 100 ms |

`alarmTask` wakes `outputTask` with a thread flag on every state change, so the outputs react right away instead of waiting out a blink period.

Every event and state change is logged over UART, e.g.:

```
>>> State: DISARMED
[DISARMED] Button
>>> State: ARMED
[ARMED] Door OPEN
>>> State: ENTRY DELAY (10 s)
```

## Build and flash

1. Open STM32CubeIDE and choose **File → Import → Existing Projects into Workspace**, then select this folder.
2. Build (the HAL, CMSIS and FreeRTOS sources are included).
3. Flash to the NUCLEO-L476RG and open a terminal on the ST-LINK COM port at 115200 baud.

`security node.ioc` holds the CubeMX configuration. All application code is inside `USER CODE` blocks in `Core/Src/main.c`, so regenerating from CubeMX keeps it.

## Roadmap

- [x] Door, tripwire (COMP2), shock and tilt sensing with interrupt → queue → task flow
- [x] Alarm state machine with a software-timer entry delay
- [x] RGB status LED, buzzer patterns, mutex-protected UART logging
- [ ] FreeRTOS tickless idle + STOP2, tick on LPTIM1 from the 32.768 kHz LSE
- [ ] Measure MCU current on the JP6 IDD jumper, before and after low power
- [ ] NEC IR remote arm/disarm, decoded by hand with timer input capture (no library)
- [ ] Relay as a simulated door lock
- [ ] Independent watchdog with per-task check-ins
