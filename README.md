# Foinse

Foinse is the power/source board in the Ogma flight-computer stack. It owns the source-side power hardware and measures the board current-sensor outputs.

## Role In Ogma

- Provides the stack source-side power hardware.
- Uses the common Ogma power chain: stack 5 V, local 3.8 V preregulator, local 3.3 V LDO.
- Uses the common STM32F072 and TJA1051 CAN hardware pattern.
- Reads two current-sensor sense lines on MCU ADC inputs.

## Firmware

Current firmware target:

```bash
cd firmware
pio run -e stm32f072c8t6
```

The firmware exposes:

- `ogma_board_identity`: SWD-readable board identity for Ogma Console.
- `foinse_status`: SWD-readable status block.
- `Sense1`: PA3 / ADC_IN3.
- `Sense2`: PA4 / ADC_IN4.

## Ogma Console Support

Ogma Console can:

- identify Foinse over SWD,
- build and flash `stm32f072c8t6`,
- read `foinse_status`,
- show raw ADC counts, millivolts, and mapped current in mA for Sense1/Sense2.

## Current Limits

- CAN broadcasts a Foinse heartbeat at 1 Hz and battery/servo current status at 5 Hz.
- Foinse does not measure battery voltage, servo voltage, state of charge, or board temperature. CAN validity flags mark only current fields valid.
- Current conversion uses the ACS71240KEXBLT-010B3 nominal 1.65 V zero-current point and 132 mV/A sensitivity. Bench calibration can refine offset/gain later.
- Sense1 is battery-to-board current. Sense2 is servo 5 V rail current.
- ADC validity flags retain last-good readings instead of publishing false current on conversion timeout.
- Hardware watchdog reset status is reported in `foinse_status` version 5.
- CAN IDs and payload structs come from shared `comheadan`.

## Dependency Lock

Use the exact shared-library pins in `../dependencies.lock.json`:

- `braiteoiri`: `ogma/flight-hardening`
- `comheadan`: `ogma/flight-hardening`

Ogma Console doctor fails a board when these submodule SHAs do not match the lock file.
