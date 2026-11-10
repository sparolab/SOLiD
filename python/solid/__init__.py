"""SOLiD — Spatially Organized and Lightweight Global Descriptor.

Two interchangeable backends with an identical API
(``Config`` / ``Descriptor`` / ``Candidate`` / ``Extractor`` / ``Database``,
methods ``extract`` / ``loop_similarity`` / ``pose_yaw_deg`` / ``add`` / ``query``):

* the C++ core binding (fast) — exposed at the top level when available::

      from solid import Extractor, Database, Config

* the pure-Python fallback (NumPy only) — always available::

      from solid.pure import Extractor, Database, Config

``solid.backend`` is ``"cpp"`` when the compiled core is present, else ``"python"``
(the top-level names then fall back to the pure implementation).
"""
from . import pure

try:
    from ._core import Candidate, Config, Database, Descriptor, Extractor  # noqa: F401
    from ._core import slam  # noqa: F401  (C++ keyframe/loop detector + session writer)
    backend = "cpp"
except ImportError:  # compiled extension not built — fall back to pure Python
    from .pure import Candidate, Config, Database, Descriptor, Extractor  # noqa: F401
    backend = "python"
    slam = None  # solid.slam needs the compiled core

# Parameters are managed via YAML config (bundled profiles or a custom path).
from .config_io import list_profiles, load_config  # noqa: E402

__all__ = ["Config", "Descriptor", "Candidate", "Extractor", "Database",
           "pure", "slam", "backend", "load_config", "list_profiles"]
__version__ = "2.1.0"
