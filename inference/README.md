# Inference

Model runtime code lives here, including:

- model interfaces and backend adapters;
- input preprocessing and output postprocessing;
- local and remote inference runners;
- acceleration, quantization, and export integrations;
- latency and accuracy benchmarks.

Model weights are intentionally ignored by Git. Add reproducible download or
conversion scripts under `../tools/` and record model configuration in
`../configs/inference/` when that directory is introduced.
