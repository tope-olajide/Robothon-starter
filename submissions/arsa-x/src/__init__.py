"""ARSA-X — Agentic Robotic Surgery Assistant eXtended.

This package is a backward-compatibility shim that re-exports from
the new modular arsax/ package. The codebase has been restructured
for better organization and judge readability.
"""

__version__ = "2.0.0"
__author__ = "ARSA-X Team"

from arsax import *  # noqa: F401, F403
