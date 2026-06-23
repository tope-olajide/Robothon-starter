"""Unit tests for the physics-grounded surgical audit system.

Tests that each audit check:
  - Runs without errors
  - Produces an AuditEvidence object with the expected structure
  - Passes (all 8 checks must pass for the system to be credible)
"""

from __future__ import annotations

import json
from dataclasses import fields
from pathlib import Path

import pytest

from arsax.evaluation.audit import SurgicalAuditor, AuditEvidence


class TestAuditImports:
    """Verify the audit module and its types are importable."""

    def test_audit_evidence_importable(self):
        assert AuditEvidence is not None

    def test_surgical_auditor_importable(self):
        assert SurgicalAuditor is not None


class TestSurgicalAuditor:
    """Integration tests for the full audit pipeline."""

    def test_auditor_initialises(self):
        auditor = SurgicalAuditor(output_dir="/tmp/arsax_audit_test")
        assert auditor is not None
        assert auditor._evidence == []

    def test_run_audit_returns_8_checks(self):
        auditor = SurgicalAuditor(output_dir="/tmp/arsax_audit_test")
        evidence = auditor.run_audit()
        assert len(evidence) == 8

    def test_all_checks_pass(self):
        auditor = SurgicalAuditor(output_dir="/tmp/arsax_audit_test")
        evidence = auditor.run_audit()
        for e in evidence:
            assert e.passed, (
                f"Audit check '{e.check_name}' FAILED: {e.failure_reason}"
            )

    def test_each_check_has_required_fields(self):
        auditor = SurgicalAuditor(output_dir="/tmp/arsax_audit_test")
        evidence = auditor.run_audit()
        required = {"check_name", "description", "passed", "metrics", "failure_reason"}
        for e in evidence:
            actual = {f.name for f in fields(e)}
            missing = required - actual
            assert not missing, (
                f"Check '{e.check_name}' missing fields: {missing}"
            )
            assert isinstance(e.metrics, dict), (
                f"Check '{e.check_name}' metrics should be a dict"
            )

    def test_each_check_has_non_empty_metrics(self):
        auditor = SurgicalAuditor(output_dir="/tmp/arsax_audit_test")
        evidence = auditor.run_audit()
        for e in evidence:
            assert len(e.metrics) >= 1, (
                f"Check '{e.check_name}' has empty metrics"
            )


class TestAuditReport:
    """Verify the generated audit report structure."""

    def test_generate_report_structure(self):
        output_dir = Path("/tmp/arsax_audit_report_test")
        output_dir.mkdir(parents=True, exist_ok=True)
        auditor = SurgicalAuditor(output_dir=str(output_dir))
        auditor.run_audit()
        report = auditor.generate_report(
            output_path=str(output_dir / "arsax_surgical_audit.json")
        )

        assert "audit_timestamp" in report
        assert "summary" in report
        assert report["summary"]["checks_total"] == 8
        assert report["summary"]["all_passed"] is True

    def test_report_saved_to_disk(self):
        output_dir = Path("/tmp/arsax_audit_save_test")
        output_dir.mkdir(parents=True, exist_ok=True)
        auditor = SurgicalAuditor(output_dir=str(output_dir))
        auditor.run_audit()
        auditor.generate_report(
            output_path=str(output_dir / "arsax_surgical_audit.json")
        )
        report_path = output_dir / "arsax_surgical_audit.json"
        assert report_path.exists()
        data = json.loads(report_path.read_text())
        assert data["summary"]["checks_passed"] == 8

    def test_report_contains_architecture_inference(self):
        output_dir = Path("/tmp/arsax_audit_arch_test")
        output_dir.mkdir(parents=True, exist_ok=True)
        auditor = SurgicalAuditor(output_dir=str(output_dir))
        auditor.run_audit()
        report = auditor.generate_report(
            output_path=str(output_dir / "arsax_surgical_audit.json")
        )
        assert "architecture_inference" in report
        arch = report["architecture_inference"]
        assert "controller_type" in arch
        assert arch["controller_type"] == "closed-loop residual policy"
        assert "physics_channels_read" in arch
        assert len(arch["physics_channels_read"]) >= 6


class TestIndividualCheckNames:
    """Verify each check has a meaningful name and description."""

    CHECK_NAMES = {
        "contact_force_proof",
        "needle_displacement",
        "weld_engagement",
        "tissue_deformation",
        "joint_actuation",
        "sensor_correlation",
        "slip_detection",
        "hand_pose_transition",
    }

    PHYSICS_KEYWORDS = [
        "MuJoCo", "mjSENS", "qpos", "physics", "contact",
        "mjEQ", "constraint", "eq_data", "xpos", "actuator",
    ]

    def test_all_checks_have_expected_names(self):
        auditor = SurgicalAuditor(output_dir="/tmp/arsax_audit_names_test")
        evidence = auditor.run_audit()
        names = {e.check_name for e in evidence}
        assert names == self.CHECK_NAMES, (
            f"Missing: {self.CHECK_NAMES - names}, "
            f"Unexpected: {names - self.CHECK_NAMES}"
        )

    def test_each_description_is_meaningful(self):
        auditor = SurgicalAuditor(output_dir="/tmp/arsax_audit_desc_test")
        evidence = auditor.run_audit()
        for e in evidence:
            assert len(e.description) > 30, (
                f"Check '{e.check_name}' description too short: {e.description}"
            )
            has_keyword = any(kw in e.description for kw in self.PHYSICS_KEYWORDS)
            assert has_keyword, (
                f"Check '{e.check_name}' description missing physics reference: "
                f"{e.description}"
            )


class TestAuditSmoke:
    """Quick smoke tests — verify the auditor runs without crashing."""

    def test_full_audit_runs_without_error(self):
        auditor = SurgicalAuditor(output_dir="/tmp/arsax_audit_smoke")
        evidence = auditor.run_audit()
        assert len(evidence) == 8

    def test_contact_force_check_returns_valid_evidence(self):
        auditor = SurgicalAuditor(output_dir="/tmp/arsax_audit_smoke")
        auditor.run_audit()  # Creates the shared scene
        e = auditor._check_contact_force_proof()
        assert isinstance(e, AuditEvidence)
        assert "peak_wrist_force_n" in e.metrics or "peak_contacts_ncon" in e.metrics

    def test_weld_engagement_check_returns_valid_evidence(self):
        auditor = SurgicalAuditor(output_dir="/tmp/arsax_audit_smoke")
        auditor.run_audit()
        e = auditor._check_weld_engagement()
        assert isinstance(e, AuditEvidence)
        assert "initial_active" in e.metrics

    def test_hand_pose_check_returns_valid_evidence(self):
        auditor = SurgicalAuditor(output_dir="/tmp/arsax_audit_smoke")
        auditor.run_audit()
        e = auditor._check_hand_pose_transition()
        assert isinstance(e, AuditEvidence)
        assert "dof_with_significant_change" in e.metrics
