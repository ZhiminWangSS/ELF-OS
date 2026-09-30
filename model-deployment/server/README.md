# SeekVLN real-world deployment service

This service is intended to be copied to
`/mnt/storage2/users/zmwang/SeekVLN/real-world-deployment` on gnode3. It loads
the shared SeekVLN HF export (currently the RL checkpoint
`20260923_klfix_v26_resume5_global_step_20_hf_full`, produced by
`eval/export_seekvln_verl_sft_checkpoint.py` from the verl run
`20260922_152649_8gpu_renyi_wo_cf_0921_klfix_lr4e6_v26_resume5/global_step_20`)
and exposes localhost-only `/healthz` and `/v1/navigate` endpoints. The NX
reaches it through the SSH forward `127.0.0.1:18012 -> 127.0.0.1:8012`.

```bash
./launch_seekvln_server.sh
```

The launcher defaults to physical GPU 6 remapped to `cuda:0`. Set
`CUDA_VISIBLE_DEVICES` and `SEEKVLN_DEVICE` explicitly to select another GPU.

The server accepts JPEG bytes, never NX file paths, and rejects bad hashes,
frame counts, schemas, or mode/view combinations before model inference.
