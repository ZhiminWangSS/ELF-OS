# Tests

Cross-component tests and test infrastructure live here. Organize future suites
by execution environment:

- `unit/` for fast isolated tests;
- `integration/` for component interactions;
- `simulation/` for end-to-end simulated scenarios;
- `hardware_in_loop/` for tests requiring physical devices.

Hardware-in-loop tests must default to a safe, non-actuating state and clearly
document all prerequisites.
