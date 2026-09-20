# model-deployment-test

ELF-OS 的 NaVILA 真机实验执行模块。NX 采集图像和状态，通过 SSH 本地端口转发访问 GPU 服务器上的 `server/navila_server.py`，然后把模型的离散动作交给 `ELF-OS/control/go2.py`。

默认是 dry-run。真实闭环必须显式提供 SSH host、`--execute` 和三项预算：

```bash
cd /home/unitree/pingandog/ELF-OS/planner/model-deployment-test
PYTHONPATH=. python3 -m model_deployment_test run \
  --instruction "走到门口" --ssh-host gpu-server \
  --max-decisions 12 --max-forward-m 3 --max-seconds 300 --execute
```

服务器端在 NaVILA 环境运行：

```bash
NAVILA_MODEL_PATH=/path/to/checkpoint python3 server/navila_server.py
```

模型服务接收 `elf.navigate-request.v1` multipart 请求，不接受来自 NX 的文件路径。`stop` 只代表模型声明完成，不代表物理到达。
