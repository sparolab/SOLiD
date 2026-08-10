"""Pure-Python SOLiD (NumPy only) — API-compatible with the C++ binding.

    from solid.pure import Extractor, Database, Config, Descriptor

The legacy functional API (``ptcloud2solid`` / ``get_descriptor``) remains
available in ``solid.pure.legacy``.
"""
from .core import Candidate, Config, Database, Descriptor, Extractor

__all__ = ["Config", "Descriptor", "Candidate", "Extractor", "Database"]
