# Robot

Code that runs on or directly communicates with the physical ELF robot lives
here, including:

- hardware drivers and vendor SDK adapters;
- sensor and actuator bridges;
- robot bring-up, locomotion, and control;
- health monitoring, emergency-stop, and safety logic;
- on-robot launch and service definitions.

Hardware-independent algorithms and message definitions belong in `../common/`.
Secrets and per-robot calibration overrides must not be committed.
