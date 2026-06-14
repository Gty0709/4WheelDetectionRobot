"""Deep embedding ReID (OSNet) with ORB BoW fallback for waypoint association."""

from __future__ import annotations

import os
import sys
import tempfile
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

from detection_pkg.waypoint_reid import OrbBowReID, crop_detection_patch

__all__ = ['OsnetReID', 'WaypointReID', 'crop_detection_patch']


def _append_user_site() -> None:
    import site
    user_site = site.getusersitepackages()
    if user_site and user_site not in sys.path:
        sys.path.append(user_site)


class OsnetReID:
    """OSNet-AIN x0_25 appearance embeddings with cosine matching."""

    def __init__(self, cosine_min: float = 0.55, ema_alpha: float = 0.3) -> None:
        self._cosine_min = cosine_min
        self._ema_alpha = ema_alpha
        self._entries: Dict[int, np.ndarray] = {}
        self._extractor = None
        self._load_error: Optional[str] = None

    @property
    def size(self) -> int:
        return len(self._entries)

    def _ensure_extractor(self) -> bool:
        if self._extractor is not None:
            return True
        if self._load_error is not None:
            return False
        try:
            _append_user_site()
            import torch
            from torchreid.utils import FeatureExtractor

            device = 'cuda' if torch.cuda.is_available() else 'cpu'
            self._extractor = FeatureExtractor(
                model_name='osnet_ain_x0_25',
                device=device,
            )
            return True
        except Exception as exc:
            self._load_error = str(exc)
            return False

    @staticmethod
    def _normalize(vec: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(vec))
        if norm < 1e-8:
            return vec.astype(np.float32)
        return (vec / norm).astype(np.float32)

    def _embed(self, bgr: np.ndarray) -> Optional[np.ndarray]:
        if not self._ensure_extractor():
            return None
        tmp_path = ''
        try:
            with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as handle:
                tmp_path = handle.name
            if not cv2.imwrite(tmp_path, bgr):
                return None
            feat = self._extractor(tmp_path)
            vec = feat.cpu().numpy().astype(np.float32).ravel()
            return self._normalize(vec)
        except Exception:
            return None
        finally:
            if tmp_path and os.path.isfile(tmp_path):
                os.unlink(tmp_path)

    @staticmethod
    def _cosine(a: np.ndarray, b: np.ndarray) -> float:
        return float(np.dot(a, b))

    def query(self, bgr: np.ndarray) -> Tuple[Optional[int], float]:
        if not self._entries:
            return None, 0.0
        vec = self._embed(bgr)
        if vec is None:
            return None, 0.0
        best_idx: Optional[int] = None
        best_score = -1.0
        for idx, entry in self._entries.items():
            score = self._cosine(vec, entry)
            if score > best_score:
                best_score = score
                best_idx = idx
        if best_idx is not None and best_score >= self._cosine_min:
            return best_idx, best_score
        return None, max(best_score, 0.0)

    def register(self, waypoint_index: int, bgr: np.ndarray) -> None:
        vec = self._embed(bgr)
        if vec is None:
            return
        if waypoint_index in self._entries:
            old = self._entries[waypoint_index]
            blended = (1.0 - self._ema_alpha) * old + self._ema_alpha * vec
            self._entries[waypoint_index] = self._normalize(blended)
        else:
            self._entries[waypoint_index] = vec


class WaypointReID:
    """Facade: OSNet primary, ORB BoW fallback on import/inference failure."""

    def __init__(
        self,
        backend: str = 'osnet',
        cosine_min: float = 0.55,
        orb_vocab_size: int = 48,
        orb_match_score_min: float = 0.42,
    ) -> None:
        self._backend = backend.strip().lower()
        self._osnet: Optional[OsnetReID] = None
        self._orb: Optional[OrbBowReID] = None
        self._active = 'none'
        self._fallback_reason = ''

        if self._backend == 'orb':
            self._orb = OrbBowReID(
                vocab_size=max(orb_vocab_size, 8),
                match_score_min=orb_match_score_min,
            )
            self._active = 'orb'
            return

        self._osnet = OsnetReID(cosine_min=cosine_min)
        if self._osnet._ensure_extractor():
            self._active = 'osnet'
        else:
            self._fallback_reason = self._osnet._load_error or 'unknown'
            self._osnet = None
            self._orb = OrbBowReID(
                vocab_size=max(orb_vocab_size, 8),
                match_score_min=orb_match_score_min,
            )
            self._active = 'orb'

    @property
    def active_backend(self) -> str:
        return self._active

    @property
    def fallback_reason(self) -> str:
        return self._fallback_reason

    @property
    def size(self) -> int:
        if self._active == 'osnet' and self._osnet is not None:
            return self._osnet.size
        if self._orb is not None:
            return self._orb.size
        return 0

    def query(self, bgr: np.ndarray) -> Tuple[Optional[int], float]:
        if self._active == 'osnet' and self._osnet is not None:
            return self._osnet.query(bgr)
        if self._orb is not None:
            return self._orb.query(bgr)
        return None, 0.0

    def register(self, waypoint_index: int, bgr: np.ndarray) -> None:
        if self._active == 'osnet' and self._osnet is not None:
            self._osnet.register(waypoint_index, bgr)
        elif self._orb is not None:
            self._orb.register(waypoint_index, bgr)
