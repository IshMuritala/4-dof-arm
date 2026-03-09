
from __future__ import annotations

import base64
import json
import mimetypes
import os
from pathlib import Path
from typing import Any, Dict, Optional

from openai import OpenAI
import yaml

from .schemas import IntentResponse, ModelResponse, SceneSummary

DEFAULT_MODEL_ID = "gpt-5.2-2025-12-11"
DEFAULT_IMAGE_DETAIL = "high"
DEFAULT_TEMPERATURE = 0.2
DEFAULT_MAX_OUTPUT_TOKENS = 256
DEFAULT_REASONING_EFFORT = "medium"

SYSTEM_PROMPT = (
    "You select a single target object ID for a robot pick task.\n"
    "You will receive a scene summary (IDs + attributes) and an annotated image with IDs/bboxes.\n"
    "If instruction_history is provided, it contains prior user instructions; "
    "use it to interpret the current instruction.\n"
    "Rules:\n"
    "- Only choose IDs that appear in the scene summary.\n"
    "- If the instruction is ambiguous, refers to multiple objects, or no valid target exists, "
    "set confirm_needed=true and target_object_id=null.\n"
    "- Keep reason short (1-2 sentences, <= 160 chars).\n"
    "Return JSON only with keys: target_object_id, confirm_needed, reason.\n"
)

INTENT_SYSTEM_PROMPT = (
    "You are an intent classifier for a robot arm.\n"
    "You receive JSON with: instruction (string) and holding_item (bool).\n"
    "Decide one action:\n"
    "- pick: user wants to pick something.\n"
    "- place_back: user wants to put down the currently held item.\n"
    "- clarify: instruction is ambiguous; ask a short question.\n"
    "Rules:\n"
    "- If holding_item is false, you must NOT return place_back.\n"
    "- If the user asks to place/put down/release and holding_item is false, ask a question.\n"
    "- Keep clarification_question short (<= 120 chars).\n"
    "Return JSON only with keys: action, clarification_question.\n"
    "- If action=clarify, clarification_question must be non-empty.\n"
    "- Otherwise clarification_question must be null.\n"
)

_CONFIG_CACHE: Optional[Dict[str, Any]] = None


def _env(name: str, default: str) -> str:
    return os.environ.get(name, default)


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except ValueError:
        return default


def _load_model_config() -> Dict[str, Any]:
    global _CONFIG_CACHE
    if _CONFIG_CACHE is not None:
        return _CONFIG_CACHE

    config: Dict[str, Any] = {}
    try:
        repo_root = Path(__file__).resolve().parents[2]
        config_path = repo_root / "config" / "model.yaml"
        if config_path.exists():
            with config_path.open("r", encoding="utf-8") as f:
                config = yaml.safe_load(f) or {}
    except Exception:
        config = {}

    _CONFIG_CACHE = config
    return config


def _cfg_get(cfg: Dict[str, Any], *keys: str, default: Any) -> Any:
    cur: Any = cfg
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def _load_api_key() -> Optional[str]:
    env_key = os.environ.get("OPENAI_API_KEY")
    if env_key:
        return env_key.strip()

    try:
        repo_root = Path(__file__).resolve().parents[2]
        env_path = repo_root / ".env"
        if env_path.exists():
            with env_path.open("r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    if line.startswith("OPENAI_API_KEY="):
                        candidate = line.split("=", 1)[1].strip().strip('"').strip("'")
                        if candidate:
                            return candidate
    except Exception:
        return None
    return None


def _unwrap_text_dict(data: Dict[str, Any]) -> Any:
    if "text" in data:
        return data.get("text")
    if "output_text" in data:
        return data.get("output_text")
    return data


def _image_file_to_data_url(image_path: str) -> str:
    mime_type, _ = mimetypes.guess_type(image_path)
    if not mime_type:
        mime_type = "image/png"
    with open(image_path, "rb") as f:
        encoded = base64.b64encode(f.read()).decode("ascii")
    return f"data:{mime_type};base64,{encoded}"


def _build_user_text(instruction: str,
                     scene: SceneSummary,
                     instruction_history: Optional[list[str]] = None) -> str:
    payload = {
        "instruction": instruction,
        "scene_summary": scene.model_dump(),
        "note": "Image is annotated with IDs/bboxes; use IDs from summary only.",
        "response_format": "json",
    }
    if instruction_history:
        payload["instruction_history"] = instruction_history
    return json.dumps(payload, ensure_ascii=True, separators=(",", ":"))


def call_model(
    instruction: str,
    scene: SceneSummary,
    *,
    image_path: Optional[str] = None,
    image_data_url: Optional[str] = None,
    model_id: Optional[str] = None,
    instruction_history: Optional[list[str]] = None,
) -> ModelResponse:
    if not image_path and not image_data_url:
        raise ValueError("Either image_path or image_data_url must be provided.")
    if image_path:
        image_data_url = _image_file_to_data_url(image_path)

    cfg = _load_model_config()
    model = model_id or _env(
        "GRASP_MODEL_ID",
        _cfg_get(cfg, "selection", "chosen_model", default=DEFAULT_MODEL_ID),
    )
    image_detail = _env(
        "GRASP_MODEL_IMAGE_DETAIL",
        _cfg_get(cfg, "vision", "detail", default=DEFAULT_IMAGE_DETAIL),
    )
    temperature = _env_float(
        "GRASP_MODEL_TEMPERATURE",
        float(_cfg_get(cfg, "request", "temperature", default=DEFAULT_TEMPERATURE)),
    )
    max_output_tokens = _env_int(
        "GRASP_MODEL_MAX_OUTPUT_TOKENS",
        int(_cfg_get(cfg, "request", "max_output_tokens", default=DEFAULT_MAX_OUTPUT_TOKENS)),
    )
    reasoning_effort = _env(
        "GRASP_MODEL_REASONING_EFFORT",
        _cfg_get(cfg, "request", "reasoning_effort", default=DEFAULT_REASONING_EFFORT),
    )
    timeout_ms = _env_int(
        "GRASP_MODEL_TIMEOUT_MS",
        int(_cfg_get(cfg, "request", "timeout_ms", default=20000)),
    )

    user_text = _build_user_text(instruction, scene, instruction_history)
    input_content = [
        {"type": "input_text", "text": user_text},
        {"type": "input_image", "image_url": image_data_url, "detail": image_detail},
    ]

    api_key = _load_api_key()
    if api_key:
        client = OpenAI(api_key=api_key, timeout=timeout_ms / 1000.0)
    else:
        raise ValueError(
            "OPENAI_API_KEY not set and .env not found. "
            "Set env var or add OPENAI_API_KEY to .env at repo root."
        )
    response_schema: Dict[str, Any] = {
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "target_object_id": {
                "anyOf": [
                    {"type": "string", "minLength": 1},
                    {"type": "null"},
                ]
            },
            "confirm_needed": {"type": "boolean"},
            "reason": {"type": "string", "minLength": 1, "maxLength": 160},
        },
        "required": ["target_object_id", "confirm_needed", "reason"],
    }

    request_kwargs: Dict[str, Any] = {
        "model": model,
        "instructions": SYSTEM_PROMPT,
        "input": [{"role": "user", "content": input_content}],
        "max_output_tokens": max_output_tokens,
        "reasoning": {"effort": reasoning_effort},
        "text": {
            "format": {
                "type": "json_schema",
                "name": "model_response",
                "schema": response_schema,
                "strict": True,
            }
        },
    }
    # Note: GPT-5.2 rejects the temperature parameter, so I omit it.
    _ = temperature

    def request_once() -> Any:
        response = client.responses.create(**request_kwargs)
        return extract_payload(response)

    def extract_payload(resp: Any) -> Any:
        # Prefer structured JSON if available
        try:
            for out in getattr(resp, "output", []) or []:
                for content in getattr(out, "content", []) or []:
                    if hasattr(content, "json"):
                        try:
                            val = content.json() if callable(content.json) else content.json
                        except Exception:
                            val = None
                        if val is not None and val != "":
                            return val
        except Exception:
            pass

        # Otherwise concatenate all text segments
        texts = []
        try:
            for out in getattr(resp, "output", []) or []:
                for content in getattr(out, "content", []) or []:
                    if hasattr(content, "text") and content.text:
                        texts.append(content.text)
        except Exception:
            pass

        raw = "".join(texts) if texts else (resp.output_text or "")
        return raw

    payload = None
    for _ in range(2):
        payload = request_once()
        if payload is not None and payload != "":
            break
    if isinstance(payload, dict) and "text" in payload and len(payload) == 1:
        payload = payload.get("text")
    if isinstance(payload, dict) and "text" in payload and "type" in payload:
        # Handle ResponseOutputText-style dicts.
        payload = payload.get("text")
    if payload is None or payload == "":
        raise ValueError("Empty response from model.")

    if isinstance(payload, dict) and "target_object_id" in payload:
        return ModelResponse.model_validate(payload)

    raw_text = str(payload)
    try:
        data: Dict[str, Any] = json.loads(raw_text)
        if isinstance(data, dict) and "target_object_id" not in data:
            data = _unwrap_text_dict(data)
            if isinstance(data, str):
                data = json.loads(data)
        return ModelResponse.model_validate(data)
    except json.JSONDecodeError:
        # Try to salvage a JSON object if extra text exists
        start = raw_text.find("{")
        end = raw_text.rfind("}")
        if start != -1 and end != -1 and end > start:
            snippet = raw_text[start:end + 1]
            try:
                data = json.loads(snippet)
                if isinstance(data, dict) and "target_object_id" not in data:
                    data = _unwrap_text_dict(data)
                    if isinstance(data, str):
                        data = json.loads(data)
                return ModelResponse.model_validate(data)
            except json.JSONDecodeError:
                pass
        raise ValueError(f"Invalid JSON from model: {raw_text}")


def call_intent_model(
    instruction: str,
    holding_item: bool,
    *,
    model_id: Optional[str] = None,
) -> IntentResponse:
    cfg = _load_model_config()
    model = model_id or _env(
        "GRASP_MODEL_ID",
        _cfg_get(cfg, "selection", "chosen_model", default=DEFAULT_MODEL_ID),
    )
    max_output_tokens = _env_int(
        "GRASP_MODEL_MAX_OUTPUT_TOKENS",
        int(_cfg_get(cfg, "request", "max_output_tokens", default=DEFAULT_MAX_OUTPUT_TOKENS)),
    )
    reasoning_effort = _env(
        "GRASP_MODEL_REASONING_EFFORT",
        _cfg_get(cfg, "request", "reasoning_effort", default=DEFAULT_REASONING_EFFORT),
    )
    timeout_ms = _env_int(
        "GRASP_MODEL_TIMEOUT_MS",
        int(_cfg_get(cfg, "request", "timeout_ms", default=20000)),
    )

    payload = {
        "instruction": instruction,
        "holding_item": bool(holding_item),
        "response_format": "json",
    }
    input_content = [
        {"type": "input_text", "text": json.dumps(payload, ensure_ascii=True, separators=(",", ":"))},
    ]

    api_key = _load_api_key()
    if api_key:
        client = OpenAI(api_key=api_key, timeout=timeout_ms / 1000.0)
    else:
        raise ValueError(
            "OPENAI_API_KEY not set and .env not found. "
            "Set env var or add OPENAI_API_KEY to .env at repo root."
        )

    response_schema: Dict[str, Any] = {
        "type": "object",
        "additionalProperties": False,
        "properties": {
            "action": {
                "type": "string",
                "enum": ["pick", "place_back", "clarify"],
            },
            "clarification_question": {
                "anyOf": [
                    {"type": "string", "minLength": 1, "maxLength": 120},
                    {"type": "null"},
                ]
            },
        },
        "required": ["action", "clarification_question"],
    }

    request_kwargs: Dict[str, Any] = {
        "model": model,
        "instructions": INTENT_SYSTEM_PROMPT,
        "input": [{"role": "user", "content": input_content}],
        "max_output_tokens": max_output_tokens,
        "reasoning": {"effort": reasoning_effort},
        "text": {
            "format": {
                "type": "json_schema",
                "name": "intent_response",
                "schema": response_schema,
                "strict": True,
            }
        },
    }

    def request_once() -> Any:
        response = client.responses.create(**request_kwargs)
        return extract_payload(response)

    def extract_payload(resp: Any) -> Any:
        try:
            for out in getattr(resp, "output", []) or []:
                for content in getattr(out, "content", []) or []:
                    if hasattr(content, "json"):
                        try:
                            val = content.json() if callable(content.json) else content.json
                        except Exception:
                            val = None
                        if val is not None and val != "":
                            return val
        except Exception:
            pass
        texts = []
        try:
            for out in getattr(resp, "output", []) or []:
                for content in getattr(out, "content", []) or []:
                    if hasattr(content, "text") and content.text:
                        texts.append(content.text)
        except Exception:
            pass
        raw = "".join(texts) if texts else (resp.output_text or "")
        return raw

    result = None
    for _ in range(2):
        result = request_once()
        if result is not None and result != "":
            break
    if isinstance(result, dict) and "text" in result and len(result) == 1:
        result = result.get("text")
    if isinstance(result, dict) and "text" in result and "type" in result:
        result = result.get("text")
    if result is None or result == "":
        raise ValueError("Empty response from model.")
    if isinstance(result, dict) and "action" in result:
        return IntentResponse.model_validate(result)

    raw_text = str(result)
    try:
        data: Dict[str, Any] = json.loads(raw_text)
        if isinstance(data, dict) and "action" not in data:
            data = _unwrap_text_dict(data)
            if isinstance(data, str):
                data = json.loads(data)
        return IntentResponse.model_validate(data)
    except json.JSONDecodeError:
        start = raw_text.find("{")
        end = raw_text.rfind("}")
        if start != -1 and end != -1 and end > start:
            snippet = raw_text[start:end + 1]
            try:
                data = json.loads(snippet)
                if isinstance(data, dict) and "action" not in data:
                    data = _unwrap_text_dict(data)
                    if isinstance(data, str):
                        data = json.loads(data)
                return IntentResponse.model_validate(data)
            except json.JSONDecodeError:
                pass
        raise ValueError(f"Invalid JSON from model: {raw_text}")
