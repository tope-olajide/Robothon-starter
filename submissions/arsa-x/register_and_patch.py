#!/usr/bin/env python3
"""Registration and patching utility for ARSA-X Robothon submission.

Usage:
    python submissions/arsa-x/register_and_patch.py --nickname "Your Name" --email "you@example.com"
    python submissions/arsa-x/register_and_patch.py --uuid YOUR-UUID-HERE

Registers with the Robothon API and patches local files with the UUID.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import urllib.request
import urllib.error
from pathlib import Path

_HERE = Path(__file__).resolve().parent

UUID_REGEX = re.compile(
    r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}"
)


def validate_uuid(value: str) -> bool:
    return bool(UUID_REGEX.fullmatch(value.strip()))


def register(nickname: str, email: str, agent: str = "codebuff") -> str | None:
    """Register with the Robothon API and return a UUID."""
    payload = json.dumps({
        "nickname": nickname,
        "email": email,
        "agent": agent,
        "direction": "submission/arsa-x",
    }).encode("utf-8")

    req = urllib.request.Request(
        "https://robothon.ff.com/api/register",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            uuid = str(data.get("uuid", ""))
            if validate_uuid(uuid):
                return uuid
            print(f"  WARN: Invalid UUID returned: {uuid}")
            return None
    except urllib.error.HTTPError as e:
        print(f"  ERROR: HTTP {e.code}: {e.reason}")
        return None
    except Exception as e:
        print(f"  ERROR: {e}")
        return None


def patch_files(uuid: str, participant_name: str = "Participant") -> bool:
    """Patch registration.json and PR description with the UUID."""
    # Patch registration.json
    reg_path = _HERE / "registration.json"
    if reg_path.exists():
        try:
            data = json.loads(reg_path.read_text(encoding="utf-8"))
            data["uuid"] = uuid
            data["participant_name"] = participant_name
            data["project_name"] = "ARSA-X — Agentic Robotic Surgery Assistant eXtended"
            reg_path.write_text(
                json.dumps(data, indent=2) + "\n", encoding="utf-8"
            )
            print(f"  [OK] Updated registration.json with UUID: {uuid}")
        except Exception as e:
            print(f"  ERROR patching registration.json: {e}")
            return False
    else:
        # Create registration.json
        data = {
            "uuid": uuid,
            "participant_name": participant_name,
            "project_name": "ARSA-X — Agentic Robotic Surgery Assistant eXtended",
            "ai_tools": "Codebuff",
            "original_work": True,
        }
        reg_path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        print(f"  [OK] Created registration.json with UUID: {uuid}")

    # Patch manifest
    manifest_path = _HERE / "submission_manifest.json"
    if manifest_path.exists():
        try:
            data = json.loads(manifest_path.read_text(encoding="utf-8"))
            data["registration_uuid"] = uuid
            manifest_path.write_text(
                json.dumps(data, indent=2) + "\n", encoding="utf-8"
            )
            print(f"  [OK] Updated submission_manifest.json")
        except Exception as e:
            print(f"  WARN: Could not update manifest: {e}")

    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Register ARSA-X with Robothon API and patch local files",
    )
    parser.add_argument(
        "--nickname", type=str, default="",
        help="Participant nickname for registration",
    )
    parser.add_argument(
        "--email", type=str, default="",
        help="Participant email for registration",
    )
    parser.add_argument(
        "--uuid", type=str, default=None,
        help="UUID directly (skip API registration)",
    )
    parser.add_argument(
        "--name", type=str, default="Participant",
        help="Participant display name (default: Participant)",
    )
    parser.add_argument(
        "--validate-only", action="store_true",
        help="Only run validation, skip registration",
    )
    args = parser.parse_args()

    print(f"\n{'=' * 60}")
    print(f"  ARSA-X — Registration & Patching")
    print(f"{'=' * 60}")

    uuid = args.uuid

    if args.validate_only:
        # Run validation
        from validate_submission import main as validate_main
        return validate_main()

    if not uuid and args.nickname and args.email:
        print(f"  Registering: {args.nickname} <{args.email}>")
        uuid = register(args.nickname, args.email)
        if not uuid:
            print(f"\n  Registration failed. Use --uuid to set directly.")
            return 1
    elif not uuid:
        print(f"  Provide --nickname and --email for API registration,")
        print(f"  or --uuid to set a UUID directly.")
        print(f"\n  Example:")
        print(f"    python register_and_patch.py --uuid YOUR-UUID-HERE")
        return 1

    if uuid:
        if patch_files(uuid, args.name):
            print(f"\n  Registration complete!")
        else:
            print(f"\n  Patching failed.")
            return 1

    # Run validation
    from validate_submission import main as validate_main
    return validate_main()


if __name__ == "__main__":
    sys.exit(main())
