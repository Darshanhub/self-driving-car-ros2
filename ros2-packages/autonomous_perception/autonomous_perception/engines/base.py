from abc import ABC, abstractmethod
from typing import Any, Dict


class Engine(ABC):
    """Abstract interface for inference backends."""

    def __init__(self, model_path: str, device: str, logger):
        self.model_path = model_path
        self.device = device
        self.logger = logger
        self._load_model()

    @abstractmethod
    def _load_model(self):
        """Load the underlying weights/engine."""

    @abstractmethod
    def infer(self, inputs: Dict[str, Any]) -> Dict[str, Any]:
        """Run forward pass and return detections/occupancy."""
