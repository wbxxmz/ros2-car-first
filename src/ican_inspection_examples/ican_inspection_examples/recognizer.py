#!/usr/bin/env python3
"""LBP + LDA 图片识别器（《图片识别合并方案》核心算法）。

管道：patch → 多尺度 LBP 特征(256维) → LDA 投影(40维) → 最近类均值 → 票差。
票差（最近两类距离之差）小于阈值判"未识别"，交给上层兜底。
只依赖 numpy + OpenCV，闭式解秒级训练，模型 ~44KB。
"""

from typing import Dict, Optional, Tuple

import cv2
import numpy as np


def lbp_codes(gray: np.ndarray, radius: int) -> np.ndarray:
    """8 邻域圆形 LBP 编码图（uint8，256 种编码）。"""
    h, w = gray.shape
    out = np.zeros((h, w), dtype=np.uint8)
    k = 0
    for dy in (-radius, 0, radius):
        for dx in (-radius, 0, radius):
            if dy == 0 and dx == 0:
                continue
            y2 = np.clip(np.arange(h)[:, None] + dy, 0, h - 1)
            x2 = np.clip(np.arange(w)[None, :] + dx, 0, w - 1)
            out |= (gray[y2, x2] > gray).astype(np.uint8) << k
            k += 1
    return out


def feature(patch: np.ndarray) -> np.ndarray:
    """多尺度 LBP 特征：三个半径的直方图各自归一化后取平均，再 L2 归一化。"""
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY) if patch.ndim == 3 else patch
    gray = cv2.resize(gray, (160, 120))
    total = np.zeros(256)
    for radius in (1, 2, 3):  # 单尺度对距离敏感，三半径平均才稳（实测教训）
        hist = np.bincount(lbp_codes(gray, radius).ravel(), minlength=256)
        total += hist / max(hist.sum(), 1)
    vec = total / 3.0
    norm = np.linalg.norm(vec)
    return vec / norm if norm > 0 else vec


def train(features_by_class: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
    """LDA + 最近类均值。features_by_class: {类别: (N,256) 特征矩阵}。

    类内散度用"总散度 - 类间散度"差集计算（等价且少一次遍历）；
    白化必须加正则，否则小特征值放大噪声（实测准确率会掉）。
    C 类最多 C-1 个判别方向——多取的方向是白化放大的纯噪声，
    会把最近类均值距离搅浑（实测 5 类取 40 维自检只有 0.20）。
    """
    classes = list(features_by_class.keys())
    feats = [np.asarray(v, dtype=np.float64) for v in features_by_class.values()]
    x = np.vstack(feats)
    means = [f.mean(0) for f in feats]
    mean = x.mean(0)
    # 总散度 - 类间散度 = 类内散度
    sw = (x - mean).T @ (x - mean)
    for f, mc in zip(feats, means):
        sw -= len(f) * np.outer(mc - mean, mc - mean)
    sw += (1e-3 * np.trace(sw) / sw.shape[0]) * np.eye(sw.shape[0])
    # 截断 PCA 白化：256 维特征只有几十个样本，小特征值方向是纯噪声，
    # 1/sqrt(λ) 会把它们放大（实测最近类距离被抬到 55~80，自检掉到 0.20）。
    # 只保留特征值 > cut×最大值的方向——相当于先 PCA 降维再白化。
    # cut=1e-4 由留一轮验证选出（每轮停点位置不同）：0.99，而 1e-2 只有 0.63、
    # 1e-3 只有 0.93——截太狠会把判别方向一起截掉。
    eigval, eigvec = np.linalg.eigh(sw)
    keep = eigval > 1e-4 * max(eigval.max(), 1e-12)
    whitened = eigvec[:, keep] @ np.diag(1.0 / np.sqrt(eigval[keep])) @ eigvec[:, keep].T
    sb = sum(len(f) * np.outer(mc - mean, mc - mean) for f, mc in zip(feats, means))
    _, u = np.linalg.eigh(whitened @ sb @ whitened.T)
    k = max(1, min(40, len(classes) - 1, int(keep.sum())))
    basis = whitened.T @ u[:, -k:]
    # 类中心必须与 predict() 的投影方式一致：predict 直接投原始特征（不减均值），
    # 中心就不能减 mean——一边减一边不减会差出公共偏移 basis.T@mean，
    # 最近邻全体判错（实测自检 0.99 → 0.25 就是因为这个错配）
    centers = np.stack([basis.T @ mc for mc in means])
    return {
        "basis": basis,
        "centers": centers,
        "classes": np.array(classes),
    }


def predict(model: Dict[str, np.ndarray], vec: np.ndarray) -> Tuple[int, float]:
    """返回 (最佳类别下标, 票差=次近距离-最近距离)。"""
    p = model["basis"].T @ vec
    dists = np.linalg.norm(model["centers"] - p, axis=1)
    order = np.argsort(dists)
    return int(order[0]), float(dists[order[1]] - dists[order[0]])


class Recognizer:
    """加载 .npz 模型做分类；票差不足返回 None（未识别）。"""

    def __init__(self, model_path: str, margin_thresh: float = 0.5) -> None:
        data = np.load(model_path, allow_pickle=False)
        self.basis = data["basis"]
        self.centers = data["centers"]
        self.classes = [str(c) for c in data["classes"]]
        self.margin_thresh = margin_thresh

    def classify(self, patch: np.ndarray) -> Tuple[Optional[str], float]:
        """返回 (类别|None, 票差)。票差 < 阈值 → (None, 票差)。"""
        idx, margin = predict(
            {"basis": self.basis, "centers": self.centers}, feature(patch)
        )
        cls = self.classes[idx] if margin >= self.margin_thresh else None
        return cls, margin
