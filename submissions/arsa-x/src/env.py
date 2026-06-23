"""Backward-compatibility shim — re-exports from the new arsax package.

The ARSA-X codebase has been restructured into a clean modular package
at arsax/. This file exists so existing imports and the run.py entry
point continue to work without changes.
"""
from arsax.scene import *  # noqa: F401, F403
from arsax.scene import (
    build_scene_model, build_scene_model_from_xml, build_combined_model,
    SurgicalScene, DeformableTissue, ARSALRobot, SensorSuite,
    PANDA_JOINTS, ALLEGRO_JOINTS,
    ALLEGRO_OPEN, ALLEGRO_CLOSE, ALLEGRO_PINCH,
    HAND_PREFIX, HAND_KP, HAND_KV,
    MANAGERIE_MISSING, PANDA_DIR, ALLEGRO_DIR,
    NEEDLE_WELD_NAME,
    activate_needle_weld, release_needle_weld,
)
