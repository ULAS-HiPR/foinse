# Foinse

Foinse is the power-source and monitoring node in [Ogma](https://sean-osullivan.com/projects/ogma/), the University of Limerick Aeronautics Society's modular avionics stack for high-powered rocketry.

## Role

- Generates and distributes the shared stack power rails.
- Measures battery and servo current.
- Reports power health over the 500 kbit/s CAN bus.
- Provides the stack's electrical entry point while each node performs its own local regulation.

## Repository

- `hardware/` - KiCad design and manufacturing outputs.
- `firmware/` - embedded firmware, power-path probes, and tests.

## Status

Rev 1 firmware and hardware bring-up continue on [`ogma/flight-hardening`](https://github.com/ULAS-HiPR/foinse/tree/ogma/flight-hardening). Release-candidate firmware builds and software tests pass, but hardware-in-the-loop validation is still underway. Foinse is not yet flight-proven.

## Manufacturing support

Rev 1 PCB fabrication was sponsored through [EasyEDA Education](https://easyeda.com/education) and manufactured by [JLCPCB](https://jlcpcb.com/). The board was designed in KiCad and imported into EasyEDA Pro for the sponsorship and manufacturing workflow.

## More information

See the [Ogma project write-up](https://sean-osullivan.com/projects/ogma/) for the complete stack, bring-up work, and current status.
