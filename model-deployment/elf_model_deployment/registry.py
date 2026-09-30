from dataclasses import dataclass
from typing import Protocol, Sequence


class ObservationAdapter(Protocol):
    """Model-specific observation contract implemented by the local runner."""

    def history_frames(self, paths: Sequence[str]): ...

    def auxiliary_views(self, directory, execute): ...


class ActionAdapter(Protocol):
    """Converts a model response into a guarded local discrete action."""

    def parse(self, raw_text: str, mode: str): ...

    def to_control_args(self, action): ...


@dataclass(frozen=True)
class ModelBackend:
    model_id: str
    frame_limit: int
    supports_seek: bool
    protocol: str


_REGISTRY = {
    "navila": ModelBackend("navila", 8, False, "elf.navigate-request.v1"),
    "seekvln-4k-sft": ModelBackend("seekvln-4k-sft", 9, True, "elf.seekvln-request.v1"),
}


class ModelRegistry:
    """Explicit registry so planners do not hard-code model perception rules."""

    def __init__(self, backends=()):
        self._backends = {}
        for backend in backends:
            self.register(backend)

    def register(self, backend):
        if not isinstance(backend, ModelBackend):
            raise TypeError("backend must be ModelBackend")
        if backend.model_id in self._backends:
            raise ValueError("duplicate model backend: " + backend.model_id)
        self._backends[backend.model_id] = backend

    def get(self, model_id):
        try:
            return self._backends[model_id]
        except KeyError:
            raise ValueError("unsupported model_id: " + str(model_id))

    def models(self):
        return tuple(sorted(self._backends))


REGISTRY = ModelRegistry(_REGISTRY.values())


def get_backend(model_id):
    return REGISTRY.get(model_id)


def registered_models():
    return REGISTRY.models()
