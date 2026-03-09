
from __future__ import annotations

from typing import List, Optional

from pydantic import BaseModel, Field, field_validator, model_validator


class SceneObject(BaseModel):
    id: str = Field(..., min_length=1)
    width_m: float = Field(..., ge=0.0)


class SceneSummary(BaseModel):
    timestamp_ms: int = Field(..., ge=0)
    objects: List[SceneObject] = Field(default_factory=list)


class ModelResponse(BaseModel):
    target_object_id: Optional[str] = None
    confirm_needed: bool
    reason: str = Field(..., min_length=1)

    @field_validator("target_object_id")
    @classmethod
    def normalize_target_object_id(cls, v: Optional[str]) -> Optional[str]:
        if v is None:
            return None
        v = v.strip()
        return v or None

    @field_validator("reason")
    @classmethod
    def normalize_reason(cls, v: str) -> str:
        v = v.strip()
        if not v:
            raise ValueError("reason must be non-empty")
        max_len = 200
        if len(v) > max_len:
            v = v[:max_len]
        return v

    @model_validator(mode="after")
    def validate_target_vs_confirm(self) -> "ModelResponse":
        if self.confirm_needed and self.target_object_id is not None:
            raise ValueError("target_object_id must be null when confirm_needed is true")
        if not self.confirm_needed and not self.target_object_id:
            raise ValueError("target_object_id required when confirm_needed is false")
        return self


class IntentResponse(BaseModel):
    action: str
    clarification_question: Optional[str] = None

    @field_validator("action")
    @classmethod
    def normalize_action(cls, v: str) -> str:
        v = v.strip().lower()
        if v not in {"pick", "place_back", "clarify"}:
            raise ValueError("action must be pick, place_back, or clarify")
        return v

    @field_validator("clarification_question")
    @classmethod
    def normalize_question(cls, v: Optional[str]) -> Optional[str]:
        if v is None:
            return None
        v = v.strip()
        return v or None

    @model_validator(mode="after")
    def validate_question(self) -> "IntentResponse":
        if self.action == "clarify":
            if not self.clarification_question:
                raise ValueError("clarification_question required when action=clarify")
        else:
            if self.clarification_question:
                raise ValueError("clarification_question must be null when action!=clarify")
        return self
