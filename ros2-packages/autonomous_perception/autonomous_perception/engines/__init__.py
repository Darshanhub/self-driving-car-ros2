from .base import Engine
from .mock import MockEngine

_ENGINE_MAP = {
    'mock': MockEngine,
}


def build_engine(engine_type: str, model_path: str, device: str, logger):
    engine_type = engine_type.lower()
    if engine_type not in _ENGINE_MAP:
        raise ValueError(f'Unknown engine type: {engine_type}. Available: {list(_ENGINE_MAP.keys())}')
    engine_cls = _ENGINE_MAP[engine_type]
    return engine_cls(model_path=model_path, device=device, logger=logger)
