# SeekVLN real-world deployment service

This service is intended to be copied to
`/mnt/storage2/users/zmwang/SeekVLN/real-world-deployment` on gnode3. It loads
the shared `SeekVLN-Full-SFT-V2-3-4K` HF export and exposes localhost-only
`/healthz` and `/v1/navigate` endpoints. The NX reaches it through the SSH
forward `127.0.0.1:18012 -> 127.0.0.1:8012`.

```bash
./launch_seekvln_server.sh
```

The server accepts JPEG bytes, never NX file paths, and rejects bad hashes,
frame counts, schemas, or mode/view combinations before model inference.
