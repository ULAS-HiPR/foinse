# Foinse Firmware

Firmware for Foinse, the Ogma power/source board.

## Target

Build:

```bash
pio run -e stm32f072c8t6
```

Flash with ST-Link:

```bash
pio run -e stm32f072c8t6 -t upload
```

## Host Interface

Ogma Console talks to Foinse over SWD. No UART is required.

Host-visible symbols:

- `ogma_board_identity`
- `foinse_status`

`foinse_status` contains:

- uptime,
- loop counter,
- ADC health/fault code,
- Sense1 raw ADC and millivolt value,
- Sense2 raw ADC and millivolt value,
- Sense1/Sense2 current in mA.

## ADC Mapping

- Sense1: PA3 / ADC_IN3.
- Sense2: PA4 / ADC_IN4.

These mappings come from the current KiCad net names.

The current conversion assumes ACS71240KEXBLT-010B3 nominal behavior:

- zero current: 1650 mV,
- sensitivity: 132 mV/A,
- current mA: `(sense_mv - 1650) * 1000 / 132`.

## Current Limits

- CAN sends Foinse heartbeats at 1 Hz and current status at 5 Hz. It marks battery current and servo current valid; it does not claim unmeasured rail voltage, SOC, or temperature.
- Offset/gain should still be bench-calibrated against known loads.
