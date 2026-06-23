"""Shared fixtures for ARSA-X unit tests."""

from __future__ import annotations

from pathlib import Path
from textwrap import dedent

import mujoco
import numpy as np
import pytest


# ---------------------------------------------------------------------------
# Minimal MuJoCo model with joints the skills reference
# ---------------------------------------------------------------------------
# Concrete skills call set_joint() / _get_joint() on specific names like
# "joint1".."joint7" (Panda) and "hand_ffj0".."hand_thj3" (Allegro).
# This minimal XML provides those joints so skills can be unit-tested.
# ---------------------------------------------------------------------------

_MINIMAL_XML = dedent("""\
<mujoco>
  <compiler angle="radian" />

  <option timestep="0.002" gravity="0 0 -9.81" />

  <worldbody>
    <!-- Fixed base (no joint — grounded) -->
    <body name="base" pos="0 0 0">
      <geom name="base_geom" type="sphere" size="0.1" rgba="0.5 0.5 0.5 1"/>

      <!-- Panda arm joints: joint1..joint7 -->
      <body name="body1" pos="0 0 0">
        <joint name="joint1" type="hinge" axis="0 0 1" limited="true" range="-2.9 2.9"/>
        <geom name="g1" type="sphere" size="0.05" rgba="0 0 1 1"/>
        <inertial pos="0 0 0" mass="0.5" diaginertia="0.001 0.001 0.001"/>

        <body name="body2" pos="0 0 0">
          <joint name="joint2" type="hinge" axis="0 1 0" limited="true" range="-2.9 2.9"/>
          <geom name="g2" type="sphere" size="0.05" rgba="0 1 0 1"/>
          <inertial pos="0 0 0" mass="0.5" diaginertia="0.001 0.001 0.001"/>

          <body name="body3" pos="0 0 0">
            <joint name="joint3" type="hinge" axis="0 1 0" limited="true" range="-2.9 2.9"/>
            <geom name="g3" type="sphere" size="0.05" rgba="0 0 1 1"/>
            <inertial pos="0 0 0" mass="0.5" diaginertia="0.001 0.001 0.001"/>

            <body name="body4" pos="0 0 0">
              <joint name="joint4" type="hinge" axis="0 1 0" limited="true" range="-2.9 2.9"/>
              <geom name="g4" type="sphere" size="0.05" rgba="1 0 0 1"/>
              <inertial pos="0 0 0" mass="0.5" diaginertia="0.001 0.001 0.001"/>

              <body name="body5" pos="0 0 0">
                <joint name="joint5" type="hinge" axis="0 0 1" limited="true" range="-2.9 2.9"/>
                <geom name="g5" type="sphere" size="0.05" rgba="1 0 1 1"/>
                <inertial pos="0 0 0" mass="0.5" diaginertia="0.001 0.001 0.001"/>

                <body name="body6" pos="0 0 0">
                  <joint name="joint6" type="hinge" axis="0 1 0" limited="true" range="-2.9 2.9"/>
                  <geom name="g6" type="sphere" size="0.05" rgba="1 1 0 1"/>
                  <inertial pos="0 0 0" mass="0.5" diaginertia="0.001 0.001 0.001"/>

                  <body name="body7" pos="0 0 0">
                    <joint name="joint7" type="hinge" axis="0 1 0" limited="true" range="-2.9 2.9"/>
                    <geom name="g7" type="sphere" size="0.05" rgba="0 1 1 1"/>
                    <inertial pos="0 0 0" mass="0.5" diaginertia="0.001 0.001 0.001"/>

                    <!-- Allegro hand joints (10 of 16 — sufficient for testing) -->
                    <body name="hand_body" pos="0 0 0">
                      <body name="hand_ffj0_body" pos="0 0 0">
                        <joint name="hand_ffj0" type="hinge" axis="1 0 0" limited="true" range="-0.5 0.5"/>
                        <geom name="h_ffj0" type="sphere" size="0.02" rgba="0 1 0 1"/>
                        <inertial pos="0 0 0" mass="0.1" diaginertia="0.0001 0.0001 0.0001"/>
                        <body name="hand_ffj1_body" pos="0 0 0">
                          <joint name="hand_ffj1" type="hinge" axis="0 1 0" limited="true" range="0 1.6"/>
                          <geom name="h_ffj1" type="sphere" size="0.02" rgba="0 1 0 1"/>
                          <inertial pos="0 0 0" mass="0.1" diaginertia="0.0001 0.0001 0.0001"/>
                          <body name="hand_ffj2_body" pos="0 0 0">
                            <joint name="hand_ffj2" type="hinge" axis="0 0 1" limited="true" range="0 1.6"/>
                            <geom name="h_ffj2" type="sphere" size="0.02" rgba="0 1 0 1"/>
                            <inertial pos="0 0 0" mass="0.1" diaginertia="0.0001 0.0001 0.0001"/>
                            <body name="hand_ffj3_body" pos="0 0 0">
                              <joint name="hand_ffj3" type="hinge" axis="0 1 0" limited="true" range="0 1.6"/>
                              <geom name="h_ffj3" type="sphere" size="0.02" rgba="0 1 0 1"/>
                              <inertial pos="0 0 0" mass="0.1" diaginertia="0.0001 0.0001 0.0001"/>
                            </body>
                          </body>
                        </body>
                      </body>
                      <body name="hand_mfj0_body" pos="0 0 0">
                        <joint name="hand_mfj0" type="hinge" axis="1 0 0" limited="true" range="-0.5 0.5"/>
                        <geom name="h_mfj0" type="sphere" size="0.02" rgba="0 0 1 1"/>
                        <inertial pos="0 0 0" mass="0.1" diaginertia="0.0001 0.0001 0.0001"/>
                        <body name="hand_mfj1_body" pos="0 0 0">
                          <joint name="hand_mfj1" type="hinge" axis="0 1 0" limited="true" range="0 1.6"/>
                          <geom name="h_mfj1" type="sphere" size="0.02" rgba="0 0 1 1"/>
                          <inertial pos="0 0 0" mass="0.1" diaginertia="0.0001 0.0001 0.0001"/>
                        </body>
                      </body>
                      <body name="hand_thj0_body" pos="0 0 0">
                        <joint name="hand_thj0" type="hinge" axis="0 0 1" limited="true" range="-0.5 0.5"/>
                        <geom name="h_thj0" type="sphere" size="0.02" rgba="1 0 0 1"/>
                        <inertial pos="0 0 0" mass="0.1" diaginertia="0.0001 0.0001 0.0001"/>
                        <body name="hand_thj1_body" pos="0 0 0">
                          <joint name="hand_thj1" type="hinge" axis="0 1 0" limited="true" range="0 1.6"/>
                          <geom name="h_thj1" type="sphere" size="0.02" rgba="1 0 0 1"/>
                          <inertial pos="0 0 0" mass="0.1" diaginertia="0.0001 0.0001 0.0001"/>
                          <body name="hand_thj2_body" pos="0 0 0">
                            <joint name="hand_thj2" type="hinge" axis="0 0 1" limited="true" range="0 1.6"/>
                            <geom name="h_thj2" type="sphere" size="0.02" rgba="1 0 0 1"/>
                            <inertial pos="0 0 0" mass="0.1" diaginertia="0.0001 0.0001 0.0001"/>
                            <body name="hand_thj3_body" pos="0 0 0">
                              <joint name="hand_thj3" type="hinge" axis="0 1 0" limited="true" range="0 1.6"/>
                              <geom name="h_thj3" type="sphere" size="0.02" rgba="1 0 0 1"/>
                              <inertial pos="0 0 0" mass="0.1" diaginertia="0.0001 0.0001 0.0001"/>
                            </body>
                          </body>
                        </body>
                      </body>
                      <body name="hand_palm" pos="0 0 0">
                        <geom name="h_end" type="sphere" size="0.01" rgba="1 1 1 1"/>
                        <!-- grasp-center site used by the IK-based grasp skill -->
                        <site name="grasp_center" pos="0.05 0 0" size="0.005" rgba="0 1 0 0.5"/>
                      </body>
                    </body>
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>

    <!-- Needle body (needed for monitor tests) -->
    <body name="needle" pos="0.5 0 0.5">
      <freejoint name="needle_test_joint"/>
      <geom name="needle_geom" type="capsule" size="0.003 0.02" rgba="0.8 0.8 0.8 1"/>
      <inertial pos="0 0 0" mass="0.05" diaginertia="0.0001 0.0001 0.0001"/>
    </body>

    <!-- Attachment site (needed for wrist sensor tests) -->
    <body name="attachment" pos="0 0 0">
      <geom name="attach_geom" type="sphere" size="0.01" rgba="1 1 1 1"/>
      <inertial pos="0 0 0" mass="0.1" diaginertia="0.0001 0.0001 0.0001"/>
    </body>
  </worldbody>

  <!-- Activated grasp weld (inactive), matching the real scene -->
  <equality>
    <weld name="needle_grasp_weld" body1="hand_palm" body2="needle" active="false"/>
  </equality>

  <!-- Wrist force sensor for force-guided descent tests -->
  <sensor>
    <force name="sensor_wrist_force" site="grasp_center"/>
  </sensor>
</mujoco>
""")


@pytest.fixture(scope="session")
def minimal_xml() -> str:
    """Return the minimal test model XML string."""
    return _MINIMAL_XML


@pytest.fixture(scope="function")
def model_and_data(minimal_xml: str) -> tuple[mujoco.MjModel, mujoco.MjData]:
    """Compile the minimal model and return (model, data)."""
    model = mujoco.MjModel.from_xml_string(minimal_xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data
