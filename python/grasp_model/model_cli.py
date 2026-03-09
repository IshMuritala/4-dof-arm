"""

Usage:
  python -m grasp_model.model_cli --instruction "pick the blue ball" \
    --scene-json-file /tmp/scene.json --image-path /path/to/image.png
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from typing import Any, Dict

from .model_client import call_intent_model, call_model
from .schemas import SceneSummary


def _load_scene_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def _load_history(path: str) -> list[str]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if isinstance(data, list):
        return [str(x) for x in data]
    if isinstance(data, dict) and "history" in data and isinstance(data["history"], list):
        return [str(x) for x in data["history"]]
    return []

def _parse_bool(raw: str) -> bool:
    val = raw.strip().lower()
    if val in {"1", "true", "yes", "y", "on"}:
        return True
    if val in {"0", "false", "no", "n", "off"}:
        return False
    raise ValueError(f"Invalid boolean value: {raw}")


def main(argv: list[str]) -> int:
    try:
        parser = argparse.ArgumentParser(description="Call GPT-5.2 model for target selection.")
        parser.add_argument("--instruction", required=True, help="Natural language instruction.")
        parser.add_argument("--scene-json-file", help="Path to scene summary JSON.")
        parser.add_argument("--image-path", help="Path to annotated RGB image.")
        parser.add_argument("--model-id", default=None, help="Override model ID.")
        parser.add_argument("--history-file", default=None, help="Optional JSON list of prior instructions.")
        parser.add_argument("--intent-only", action="store_true", help="Run text-only intent classifier.")
        parser.add_argument("--holding-item", default=None, help="Required for intent-only (true/false).")
        args = parser.parse_args(argv)

        start = time.time()
        if args.intent_only:
            if args.holding_item is None:
                raise SystemExit("--holding-item is required when --intent-only is set.")
            holding_item = _parse_bool(args.holding_item)
            response = call_intent_model(
                args.instruction,
                holding_item,
                model_id=args.model_id,
            )
            elapsed_ms = int((time.time() - start) * 1000.0)
            out = {
                "instruction": args.instruction,
                "holding_item": holding_item,
                "action": response.action,
                "clarification_question": response.clarification_question,
                "latency_ms": elapsed_ms,
            }
        else:
            if not args.scene_json_file or not args.image_path:
                raise SystemExit("--scene-json-file and --image-path are required for pick mode.")
            scene_payload = _load_scene_json(args.scene_json_file)
            scene = SceneSummary.model_validate(scene_payload)
            history: list[str] = []
            if args.history_file:
                history = _load_history(args.history_file)

            response = call_model(
                args.instruction,
                scene,
                image_path=args.image_path,
                model_id=args.model_id,
                instruction_history=history,
            )
            elapsed_ms = int((time.time() - start) * 1000.0)

            out = {
                "timestamp_ms": scene.timestamp_ms,
                "instruction": args.instruction,
                "target_object_id": response.target_object_id,
                "confirm_needed": response.confirm_needed,
                "reason": response.reason,
                "latency_ms": elapsed_ms,
                "instruction_history": history,
            }
        sys.stdout.write(json.dumps(out, ensure_ascii=True))
        sys.stdout.write("\n")
        return 0
    except SystemExit:
        raise
    except Exception as exc:
        sys.stdout.write(f"ERROR: {exc}\n")
        return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
