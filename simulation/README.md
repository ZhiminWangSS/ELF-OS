# Simulation

Simulation-only code lives here, including:

- simulator adapters and plugins;
- robot descriptions, virtual sensors, scenes, and assets;
- simulation launch files and evaluation scenarios;
- synthetic-data and domain-randomization utilities.

Keep interfaces that must also run on the physical robot in `../common/`.
Large simulator assets should be downloaded by a script in `../tools/` rather
than committed directly.
