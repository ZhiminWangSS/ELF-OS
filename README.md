# ELF-OS

ELF-OS is the official software repository for the ELF quadruped robot. It
keeps simulation, on-robot software, and model inference separated while
sharing stable interfaces and reusable components.

## Repository layout

```text
ELF-OS/
├── simulation/   # Simulator adapters, robot models, scenes, and launch files
├── robot/        # Hardware drivers, bring-up, control, and safety code
├── inference/    # Model loading, preprocessing, inference, and postprocessing
├── common/       # Shared interfaces, messages, utilities, and algorithms
├── configs/      # Version-controlled configuration for each runtime
├── tools/        # Setup, data, conversion, and developer utilities
├── docs/         # Architecture, setup, deployment, and operation guides
└── tests/        # Unit, integration, simulation, and hardware-in-loop tests
```

Each top-level directory contains a README describing what belongs there.

## Development principles

- Keep simulator-specific dependencies under `simulation/` and hardware-specific
  dependencies under `robot/`.
- Define contracts shared by simulation and the physical robot in `common/`.
- Keep model weights, datasets, logs, build outputs, and local secrets out of Git.
- Put reproducible runtime settings in `configs/`; do not hard-code machine paths.
- Add tests alongside every production feature, using the closest matching suite
  under `tests/`.

## Status

The repository is currently being bootstrapped. Build, installation, and runtime
instructions will be added as the first software components land.
