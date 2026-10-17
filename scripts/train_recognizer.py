#!/usr/bin/env python3
"""训练 LBP+LDA 识别器（离线，无 ROS）。

数据来源：minimal_user_node 以 collect_dir 参数跑任务时存下的 patch
（地面=矫正板图，空中/窗户=ROI 裁剪图）。类别不靠文件名猜——
按点位名查 texture_assignment.json 反推（randomize 脚本白送的标签）。

用法：
  python3 scripts/train_recognizer.py --data data/collected --holdout-seeds 7,13,21

产出：
  - 包内 models/ground_recognizer.npz（地面+空中 5 类）
  - 包内 models/window_recognizer.npz（窗户 2 类）
  - 分 seed 准确率报告（按 seed 留出验证，红线：绝不随机留帧）
"""

import argparse
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src" / "ican_inspection_examples"))
from ican_inspection_examples.recognizer import feature, predict, train  # noqa: E402

# 类别 key → 话术类别（与判分细则一致）
CATEGORY_TO_CLASS = {
    "collapsed_ruins": "坍塌废墟", "landslide": "滑坡塌方",
    "trapped_people": "被困人员", "damaged_vehicle": "受损车辆",
    "rescue_camp": "救援营地",
}
# 点位 → 材质（与 randomize 脚本 MODEL_MATERIALS 一致）
POINT_TO_MATERIAL = {
    "ground_0": "InspectionGroundTarget0", "ground_1": "InspectionGroundTarget1",
    "ground_2": "InspectionGroundTarget2", "ground_3": "InspectionGroundTarget3",
    "ground_4": "InspectionGroundTarget4",
    "aerial_1": "InspectionAerialTarget1", "aerial_2": "InspectionAerialTarget2",
    "window_1": "InspectionWindowTarget1", "window_2": "InspectionWindowTarget2",
}
ROT_MARGIN = 0.06  # 与节点推理一致的 6% 边距裁剪


def label_of(point: str, report: dict) -> str:
    """点位 → 类别：材质 → 图片/类别key → 话术类别。

    地面查 ground_type_assignment；空中该表没有，从贴图文件名前缀反推
    （collapsed_ruins_02.jpg → collapsed_ruins → 坍塌废墟）。
    """
    mat = POINT_TO_MATERIAL[point]
    if mat.startswith("InspectionWindow"):
        img = report["assignment"][mat]
        return "救援无人机" if "target" in img else "飞鸟"
    if mat.startswith("InspectionAerial"):
        img = report["assignment"][mat]
        base = re.sub(r"_\d+\.\w+$", "", img)
        return CATEGORY_TO_CLASS[base]
    return CATEGORY_TO_CLASS[report["ground_type_assignment"][mat]]


def load_dataset(data_dir: Path):
    """扫描 {点位}[_{tag}]_{时间戳}.png → (路径, 点位, seed标签)。"""
    pattern = re.compile(r"^(?P<point>[a-z_]+?\d)_(?:(?P<tag>seed\d+)_)?\d+\.png$")
    samples = []
    for path in sorted(data_dir.glob("*.png")):
        m = pattern.match(path.name)
        if not m or m.group("point") not in POINT_TO_MATERIAL:
            continue
        samples.append((path, m.group("point"), m.group("tag") or "default"))
    return samples


def patch_to_rois(patch: np.ndarray):
    """与推理一致的预处理：4 旋转 × 6% 边距裁剪（训练即增广）。"""
    h, w = patch.shape[:2]
    mx, my = int(w * ROT_MARGIN), int(h * ROT_MARGIN)
    for k in range(4):
        cand = np.ascontiguousarray(np.rot90(patch, k))
        yield cand[my:h - my, mx:w - mx]


def evaluate(model, samples, report):
    """逐帧预测，返回 {(tag, point): 正确数/总数}。"""
    stats = defaultdict(lambda: [0, 0])
    for path, point, tag in samples:
        patch = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if patch is None:
            continue
        truth = label_of(point, report)
        best_margin, best_cls = -1.0, None
        for roi in patch_to_rois(patch):
            if roi.size == 0:
                continue
            cls, margin = predict(model, feature(roi))
            cls = model["classes"][cls]
            if margin > best_margin:
                best_margin, best_cls = margin, cls
        stats[(tag, point)][1] += 1
        if best_cls == truth:
            stats[(tag, point)][0] += 1
    return stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="data/collected", help="采集 patch 目录")
    ap.add_argument("--assignment", default="texture_assignment.json")
    ap.add_argument("--holdout-seeds", default="", help="留出 seed 标记，逗号分隔（如 seed07）")
    args = ap.parse_args()

    report = json.loads(Path(args.assignment).read_text(encoding="utf-8"))
    samples = load_dataset(Path(args.data))
    if not samples:
        print(f"未找到数据：{args.data} 下没有可识别的 patch。")
        print("先用采集模式跑一遍任务：ros2 run ican_inspection_examples minimal_user_node "
              "--ros-args -p collect_dir:=data/collected -p collect_tag:=seed07")
        return 1

    holdout = {s.strip() for s in args.holdout_seeds.split(",") if s.strip()}
    tags = sorted({tag for _, _, tag in samples})
    print(f"共 {len(samples)} 帧，seed 标记：{tags}，留出：{sorted(holdout) or '无'}")

    # 三个模型分开训：地面 5 类 / 空中专用（跨域混训会被域差主导，实测空中票差
    # 从专用模型的 1.1~1.65 掉到 0.03~0.44）/ 窗户 2 类。
    # 注意：空中模型只认识采集过的类别——多 seed 比赛需对每个类别的空中贴图都采过数据。
    for scope, points_filter in (("ground", lambda p: p.startswith("ground")),
                                 ("aerial", lambda p: p.startswith("aerial")),
                                 ("window", lambda p: p.startswith("window"))):
        scope_samples = [s for s in samples if points_filter(s[1])]
        if not scope_samples:
            print(f"\n[{scope}] 无数据，跳过。")
            continue
        train_set = [s for s in scope_samples if s[2] not in holdout]
        test_set = [s for s in scope_samples if s[2] in holdout]
        feats = defaultdict(list)
        for path, point, _ in train_set:
            patch = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if patch is None:
                continue
            cls = label_of(point, report)
            for roi in patch_to_rois(patch):
                if roi.size:
                    feats[cls].append(feature(roi))
        if len(feats) < 2:
            print(f"\n[{scope}] 类别数不足 2（现有 {list(feats)}），无法训练。")
            continue
        model = train(feats)
        n_feat = model["basis"].shape[1]
        print(f"\n[{scope}] 训练 {sum(len(v) for v in feats.values())} 个增广样本，"
              f"特征 256 维 → LDA {n_feat} 维，类别：{model['classes'].tolist()}")

        # 训练集自检
        correct = total = 0
        for path, point, _ in train_set:
            patch = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if patch is None:
                continue
            truth = label_of(point, report)
            best_margin, best_cls = -1.0, None
            for roi in patch_to_rois(patch):
                if roi.size:
                    idx, margin = predict(model, feature(roi))
                    if margin > best_margin:
                        best_margin, best_cls = margin, model["classes"][idx]
            total += 1
            correct += int(best_cls == truth)
        print(f"[{scope}] 训练集自检：{correct}/{total} = {correct / max(total, 1):.2f}")

        # 留出 seed 验证（比赛现场=没见过的贴图组合）
        if test_set:
            stats = evaluate(model, test_set, report)
            print(f"[{scope}] 留出验证（按 seed）：")
            for (tag, point), (ok, n) in sorted(stats.items()):
                print(f"  {tag} {point}: {ok}/{n}")
            ok = sum(v[0] for v in stats.values())
            n = sum(v[1] for v in stats.values())
            print(f"[{scope}] 留出合计：{ok}/{n} = {ok / max(n, 1):.2f}")
        else:
            print(f"[{scope}] ⚠ 无留出 seed，以上仅为训练集自检——"
                  f"多采几个 seed 后用 --holdout-seeds 重跑才能反映真实水平。")

        # 模型输出路径：包内 models/，节点默认从这里加载
        out_dir = Path(__file__).resolve().parent.parent / "src" / "ican_inspection_examples" \
            / "ican_inspection_examples" / "models"
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / f"{scope}_recognizer.npz"
        np.savez_compressed(out_path, **model)
        print(f"[{scope}] 模型已保存：{out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
