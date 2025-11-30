import psutil
from typing import List

try:
    import pynvml
except Exception:  # pragma: no cover
    pynvml = None  # type: ignore

_NVML_AVAILABLE = pynvml is not None


class ResourceMonitor:
    def __init__(self, enable_gpu: bool = True, gpu_id: int = 0):
        self.enable_gpu = enable_gpu and _NVML_AVAILABLE
        self.gpu_id = gpu_id
        if self.enable_gpu and pynvml is not None:
            pynvml.nvmlInit()

    def sample_cpu_ram(self):
        process = psutil.Process()
        cpu = process.cpu_percent(interval=None)
        ram = process.memory_info().rss / (1024 * 1024)
        return cpu, ram

    def sample_gpu(self) -> List[dict]:
        if not self.enable_gpu or pynvml is None:
            return []
        handle = pynvml.nvmlDeviceGetHandleByIndex(self.gpu_id)
        util = pynvml.nvmlDeviceGetUtilizationRates(handle)
        mem = pynvml.nvmlDeviceGetMemoryInfo(handle)
        return [
            {'key': 'gpu_util_percent', 'value': f'{util.gpu}'},
            {'key': 'gpu_mem_util_percent', 'value': f'{util.memory}'},
            {'key': 'gpu_mem_used_mb', 'value': f'{mem.used / (1024 * 1024):.2f}'},
        ]
