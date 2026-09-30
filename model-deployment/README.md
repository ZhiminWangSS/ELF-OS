# ELF-OS model deployment

This package separates model-specific perception and parsing from the local
Go2 safety executor. It registers `navila` and `seekvln-4k-sft` independently.

SeekVLN uses up to nine real history frames. A `<seek>` decision performs a
guarded left/right 90-degree scan and sends the three auxiliary views to the
GPU service. The default is dry-run; invalid model output never selects a
random motion.

Frames are prepared to a square canvas before upload: the Go2 front camera
(videohub service) publishes 16:9 frames only — 1080p or 720p, the SDK exposes
no square or 4:3 mode — so the full wide-FOV frame is uniformly scaled into a
512x512 canvas and letterboxed vertically. This keeps the entire field of view
without geometric distortion while matching the square geometry SeekVLN saw in
training. See `elf_model_deployment/sampling.py`.

```bash
PYTHONPATH=/home/unitree/pingandog/ELF-OS/model-deployment \
python3 -m model_deployment run --model seekvln-4k-sft \
  --instruction "走到门口" --ssh-host gnode3 \
  --max-decisions 10 --max-forward-m 3 --max-seconds 300
```

The command opens the SSH tunnel in both dry-run and execute modes. Dry-run
still captures observations and calls the model, but never adds `--execute` to
Go2 control commands. A `<seek>` result cannot fabricate side views: without
an approved physical scan it ends with `safety_stopped`.
