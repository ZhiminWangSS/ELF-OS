# ELF-OS model deployment

This package separates model-specific perception and parsing from the local
Go2 safety executor. It registers `navila` and `seekvln-4k-sft` independently.

SeekVLN uses up to nine real history frames. A `<seek>` decision performs a
guarded left/right 90-degree scan and sends the three auxiliary views to the
GPU service. The default is dry-run; invalid model output never selects a
random motion.

```bash
PYTHONPATH=/home/unitree/pingandog/ELF-OS/model-deployment \
python3 -m elf_model_deployment run --model seekvln-4k-sft \
  --instruction "走到门口" --ssh-host gnode3 \
  --max-decisions 10 --max-forward-m 3 --max-seconds 300
```
