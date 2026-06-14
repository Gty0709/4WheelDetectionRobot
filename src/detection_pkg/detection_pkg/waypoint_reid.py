"""ORB bag-of-words visual re-identification for map waypoints.

Inspired by DBoW2/DBoW3 place recognition (Galvez-Lopez & Tardos, 2012;
rmsalinas/DBow3) but implemented with OpenCV ORB + k-means vocabulary so it
runs inside the existing ``.detection-python`` layer without compiling pyDBoW3.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

ORB_DIM = 32


def crop_detection_patch(
    bgr: np.ndarray,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    pad_ratio: float = 0.12,
    min_side: int = 24,
) -> Optional[np.ndarray]:
    """Crop BGR patch around bbox with relative padding."""
    h, w = bgr.shape[:2]
    bw = max(1.0, x2 - x1)
    bh = max(1.0, y2 - y1)
    px = pad_ratio * bw
    py = pad_ratio * bh
    ix1 = max(0, int(math.floor(x1 - px)))
    iy1 = max(0, int(math.floor(y1 - py)))
    ix2 = min(w, int(math.ceil(x2 + px)))
    iy2 = min(h, int(math.ceil(y2 + py)))
    if ix2 - ix1 < min_side or iy2 - iy1 < min_side:
        return None
    return bgr[iy1:iy2, ix1:ix2].copy()


@dataclass
class ReIDEntry:
    bow: np.ndarray
    color_hist: np.ndarray
    descriptors: Optional[np.ndarray] = None


class OrbBowReID:
    """Online ORB bag-of-words database keyed by waypoint index."""

    def __init__(
        self,
        vocab_size: int = 48,
        match_score_min: float = 0.42,
        orb_features: int = 200,
        min_descriptors: int = 6,
        vocab_warmup: int = 4,
    ) -> None:
        self._vocab_size = max(8, vocab_size)
        self._match_score_min = match_score_min
        self._min_descriptors = min_descriptors
        self._vocab_warmup = vocab_warmup
        self._orb = cv2.ORB_create(nfeatures=orb_features)
        self._vocab: Optional[np.ndarray] = None
        self._entries: Dict[int, ReIDEntry] = {}
        self._pending_descriptors: List[np.ndarray] = []

    @property
    def size(self) -> int:
        return len(self._entries)

    def _bow_dim(self) -> int:
        return len(self._vocab) if self._vocab is not None else ORB_DIM

    def _empty_bow(self) -> np.ndarray:
        return np.zeros(self._bow_dim(), dtype=np.float32)

    def _extract_descriptors(self, bgr: np.ndarray) -> Optional[np.ndarray]:
        if bgr is None or bgr.size == 0:
            return None
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        chunks: List[np.ndarray] = []
        for scale in (1.0, 2.0):
            g = gray if scale == 1.0 else cv2.resize(gray, None, fx=scale, fy=scale)
            _kp, des = self._orb.detectAndCompute(g, None)
            if des is not None and len(des) > 0:
                chunks.append(des)
        if not chunks:
            return None
        merged = np.vstack(chunks)
        if merged.dtype != np.uint8:
            merged = merged.astype(np.uint8)
        return merged if len(merged) >= self._min_descriptors else None

    @staticmethod
    def _descriptors_for_kmeans(des: np.ndarray) -> np.ndarray:
        return des.astype(np.float32)

    @staticmethod
    def _color_histogram(bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1, 2], None, [16, 4, 4], [0, 180, 0, 256, 0, 256])
        hist = hist.astype(np.float32).ravel()
        s = float(hist.sum())
        if s > 1e-6:
            hist /= s
        return hist

    def _build_vocab(self) -> None:
        if self._vocab is not None or len(self._pending_descriptors) < self._vocab_warmup:
            return
        stacked = np.vstack([self._descriptors_for_kmeans(d) for d in self._pending_descriptors])
        k = int(min(self._vocab_size, len(stacked)))
        if k < 4:
            return
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 12, 0.5)
        _compact, _labels, centers = cv2.kmeans(
            stacked, k, None, criteria, 3, cv2.KMEANS_PP_CENTERS)
        self._vocab = centers
        for entry in self._entries.values():
            if entry.descriptors is not None:
                entry.bow = self._descriptors_to_bow(entry.descriptors)
            else:
                entry.bow = self._empty_bow()

    def _descriptors_to_bow(self, des: np.ndarray) -> np.ndarray:
        des_f = self._descriptors_for_kmeans(des)
        if self._vocab is None:
            vec = des_f.mean(axis=0)
            norm = np.linalg.norm(vec)
            return (vec / norm) if norm > 1e-6 else self._empty_bow()
        diff = des_f[:, None, :] - self._vocab[None, :, :]
        labels = np.argmin(np.linalg.norm(diff, axis=2), axis=1)
        hist = np.bincount(labels, minlength=len(self._vocab)).astype(np.float32)
        s = float(hist.sum())
        if s > 1e-6:
            hist /= s
        return hist

    @staticmethod
    def _histogram_intersection(a: np.ndarray, b: np.ndarray) -> float:
        n = min(len(a), len(b))
        if n == 0:
            return 0.0
        return float(np.minimum(a[:n], b[:n]).sum())

    def _descriptor_match_score(self, des: np.ndarray, entry: ReIDEntry) -> float:
        if entry.descriptors is None or len(entry.descriptors) < 4:
            return 0.0
        q = des.astype(np.uint8) if des.dtype != np.uint8 else des
        t = entry.descriptors.astype(np.uint8) if entry.descriptors.dtype != np.uint8 else entry.descriptors
        try:
            bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
            pairs = bf.knnMatch(q, t, k=2)
        except cv2.error:
            return 0.0
        good = 0
        for pair in pairs:
            if len(pair) < 2:
                continue
            m, n = pair[0], pair[1]
            if m.distance < 0.75 * n.distance:
                good += 1
        return good / max(len(des), 1)

    def query(self, bgr: np.ndarray) -> Tuple[Optional[int], float]:
        """Return (waypoint_index, score) of best match, or (None, 0)."""
        if not self._entries:
            return None, 0.0
        des = self._extract_descriptors(bgr)
        color = self._color_histogram(bgr)
        bow = self._descriptors_to_bow(des) if des is not None else None
        best_idx: Optional[int] = None
        best_score = 0.0
        for idx, entry in self._entries.items():
            color_score = self._histogram_intersection(color, entry.color_hist)
            if bow is not None and entry.descriptors is not None:
                bow_score = self._histogram_intersection(bow, entry.bow)
                des_score = self._descriptor_match_score(des, entry)
                score = 0.40 * bow_score + 0.35 * min(des_score, 1.0) + 0.25 * color_score
            else:
                score = color_score
            if score > best_score:
                best_score = score
                best_idx = idx
        if best_idx is not None and best_score >= self._match_score_min:
            return best_idx, best_score
        return None, best_score

    def register(self, waypoint_index: int, bgr: np.ndarray) -> None:
        """Create or refresh appearance for ``waypoint_index``."""
        des = self._extract_descriptors(bgr)
        color = self._color_histogram(bgr)
        if des is not None:
            self._pending_descriptors.append(des)
            self._build_vocab()
            bow = self._descriptors_to_bow(des)
        elif waypoint_index in self._entries:
            entry = self._entries[waypoint_index]
            entry.color_hist = 0.7 * entry.color_hist + 0.3 * color
            s = float(entry.color_hist.sum())
            if s > 1e-6:
                entry.color_hist /= s
            return
        else:
            bow = self._empty_bow()
            des = None
        if waypoint_index in self._entries:
            entry = self._entries[waypoint_index]
            entry.color_hist = 0.7 * entry.color_hist + 0.3 * color
            s = float(entry.color_hist.sum())
            if s > 1e-6:
                entry.color_hist /= s
            if des is not None:
                new_bow = self._descriptors_to_bow(des)
                if entry.bow.shape != new_bow.shape:
                    entry.bow = new_bow
                else:
                    entry.bow = 0.7 * entry.bow + 0.3 * new_bow
                    s = float(entry.bow.sum())
                    if s > 1e-6:
                        entry.bow /= s
                entry.descriptors = des
        else:
            self._entries[waypoint_index] = ReIDEntry(
                bow=bow, color_hist=color, descriptors=des)
