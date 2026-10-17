#!/usr/bin/env python3

"""为最简工作空间中的二进制 iCAN 仿真包随机替换识别贴图。"""

from __future__ import annotations

import argparse
import json
import os
import random
import re
import shutil
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path


WORKSPACE_DIR = Path(__file__).resolve().parents[1]
PACKAGE_SHARE = WORKSPACE_DIR / "binary_install/share/ican_inspection_sim"
WORLD_FILE = PACKAGE_SHARE / "worlds/inspection_task1.world"
MATERIAL_FILE = PACKAGE_SHARE / "materials/scripts/ican_inspection.material"
TEXTURE_DIR = PACKAGE_SHARE / "materials/textures/point_targets"
LOCAL_SOURCE_DIR = WORKSPACE_DIR / "point_targets"
FULL_WORKSPACE = os.environ.get("ICAN_FULL_INSPECTION_WS", "").strip()
FULL_SOURCE_DIR = (
    Path(FULL_WORKSPACE)
    / "src/ican_inspection_sim/materials/textures/point_targets"
    if FULL_WORKSPACE
    else None
)
REPORT_FILE = WORKSPACE_DIR / "texture_assignment.json"

GROUND_POOLS = {
    "collapsed_ruins": [
        "collapsed_ruins_01.png",
        "collapsed_ruins_02.jpg",
    ],
    "landslide": [
        "landslide_01.jpg",
        "landslide_02.jpg",
    ],
    "trapped_people": [
        "trapped_people_01.png",
        "trapped_people_02.jpg",
    ],
    "damaged_vehicle": [
        "damaged_vehicle_01.png",
        "damaged_vehicle_02.jpg",
    ],
    "rescue_camp": [
        "rescue_camp_01.png",
        "rescue_camp_02.jpg",
    ],
}

WINDOW_CANDIDATES = [
    "window_bird_01.jpg",
    "window_bird_02.jpg",
    "window_target_1.jpg",
    "window_target_2.jpg",
]

GROUND_MATERIALS = [
    "InspectionGroundTarget0",
    "InspectionGroundTarget1",
    "InspectionGroundTarget2",
    "InspectionGroundTarget3",
    "InspectionGroundTarget4",
]

AERIAL_MATERIALS = ["InspectionAerialTarget1", "InspectionAerialTarget2"]
WINDOW_MATERIALS = ["InspectionWindowTarget1", "InspectionWindowTarget2"]

MODEL_MATERIALS = {
    "ground_task_point_0": "InspectionGroundTarget0",
    "ground_task_point_1": "InspectionGroundTarget1",
    "ground_task_point_2": "InspectionGroundTarget2",
    "ground_task_point_3": "InspectionGroundTarget3",
    "ground_task_point_4": "InspectionGroundTarget4",
    "aerial_task_point_1": "InspectionAerialTarget1",
    "aerial_task_point_2": "InspectionAerialTarget2",
    "window_target_1": "InspectionWindowTarget1",
    "window_target_2": "InspectionWindowTarget2",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="随机替换最简二进制仿真包中的地面、空中和窗口识别贴图。"
    )
    parser.add_argument("--seed", type=int, default=None, help="可选随机种子。")
    parser.add_argument(
        "--dry-run", action="store_true", help="只显示映射，不修改二进制包。"
    )
    parser.add_argument(
        "--no-report", action="store_true", help="不写入 texture_assignment.json。"
    )
    return parser.parse_args()


def validate_runtime_files() -> None:
    required = [WORLD_FILE, MATERIAL_FILE]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError("二进制仿真包缺少文件：\n" + "\n".join(missing))


def prepare_texture_candidates(stage_runtime: bool) -> None:
    source_dir = LOCAL_SOURCE_DIR
    if not source_dir.is_dir() and FULL_SOURCE_DIR is not None:
        source_dir = FULL_SOURCE_DIR
    # demo 工作区把候选图随二进制环境包放在 materials/textures/point_targets
    # 中；没有额外拷贝到工作区根目录时，直接使用这份运行时候选图。
    if not source_dir.is_dir() and TEXTURE_DIR.is_dir():
        source_dir = TEXTURE_DIR
    if not source_dir.is_dir():
        fallback = (
            f"\n也可通过 ICAN_FULL_INSPECTION_WS 指定完整工作区：{FULL_SOURCE_DIR}"
            if FULL_SOURCE_DIR
            else ""
        )
        raise FileNotFoundError(
            f"找不到候选图片目录：{LOCAL_SOURCE_DIR} 或 {TEXTURE_DIR}{fallback}"
        )

    if source_dir.resolve() != LOCAL_SOURCE_DIR.resolve():
        LOCAL_SOURCE_DIR.mkdir(parents=True, exist_ok=True)
        for source_path in source_dir.iterdir():
            if source_path.is_file() and source_path.suffix.lower() in {".jpg", ".jpeg", ".png"}:
                shutil.copy2(source_path, LOCAL_SOURCE_DIR / source_path.name)
        source_dir = LOCAL_SOURCE_DIR

    if not stage_runtime:
        return

    TEXTURE_DIR.mkdir(parents=True, exist_ok=True)
    for source_path in source_dir.iterdir():
        if source_path.is_file() and source_path.suffix.lower() in {".jpg", ".jpeg", ".png"}:
            target_path = TEXTURE_DIR / source_path.name
            if source_path.resolve() != target_path.resolve():
                shutil.copy2(source_path, target_path)


def candidate_dir() -> Path:
    return LOCAL_SOURCE_DIR if LOCAL_SOURCE_DIR.is_dir() else TEXTURE_DIR


def validate_ground_candidates() -> None:
    names = [name for images in GROUND_POOLS.values() for name in images]
    names += WINDOW_CANDIDATES
    missing = [name for name in names if not (candidate_dir() / name).is_file()]
    if missing:
        raise FileNotFoundError("缺少地面或窗户候选图片：\n" + "\n".join(missing))


def find_window_candidates() -> list[str]:
    source_dir = candidate_dir()
    candidates = sorted(
        name for name in WINDOW_CANDIDATES if (source_dir / name).is_file()
    )
    if len(candidates) < len(WINDOW_MATERIALS):
        raise RuntimeError("窗口候选图片至少需要两张。")
    return candidates


def make_assignment(rng: random.Random) -> tuple[dict[str, str], dict[str, str]]:
    assignment: dict[str, str] = {}
    ground_types: dict[str, str] = {}
    ground_used: set[str] = set()

    # 每次先随机打乱五种地面目标类型，再一一分配到五个地面点。
    # 这样点位的类型也会变化，而不是每个点永远绑定同一种类型。
    categories = rng.sample(list(GROUND_POOLS), len(GROUND_MATERIALS))
    for material_name, category in zip(GROUND_MATERIALS, categories):
        selected = rng.choice(GROUND_POOLS[category])
        assignment[material_name] = selected
        ground_types[material_name] = category
        ground_used.add(selected)

    all_ground_images = {
        image_name for images in GROUND_POOLS.values() for image_name in images
    }
    aerial_candidates = sorted(all_ground_images - ground_used)
    if len(aerial_candidates) < len(AERIAL_MATERIALS):
        raise RuntimeError("空中图片候选数量不足。")
    for material_name, selected in zip(
        AERIAL_MATERIALS, rng.sample(aerial_candidates, len(AERIAL_MATERIALS))
    ):
        assignment[material_name] = selected

    window_candidates = find_window_candidates()
    for material_name, selected in zip(
        WINDOW_MATERIALS, rng.sample(window_candidates, len(WINDOW_MATERIALS))
    ):
        assignment[material_name] = selected

    return assignment, ground_types


def find_material_block(material_text: str, material_name: str) -> tuple[int, int]:
    start_match = re.search(
        rf"^material\s+{re.escape(material_name)}\s*\{{", material_text, re.MULTILINE
    )
    if start_match is None:
        raise ValueError(f"材质脚本中找不到材质：{material_name}")

    depth = 0
    for index in range(start_match.start(), len(material_text)):
        if material_text[index] == "{":
            depth += 1
        elif material_text[index] == "}":
            depth -= 1
            if depth == 0:
                return start_match.start(), index + 1
    raise ValueError(f"材质块没有正常结束：{material_name}")


def replace_material_texture(material_text: str, material_name: str, texture_name: str) -> str:
    start, end = find_material_block(material_text, material_name)
    block = material_text[start:end]
    updated, replacements = re.subn(
        r"(^[ \t]*texture[ \t]+)(\S+)([ \t]*)$",
        rf"\g<1>{texture_name}\g<3>",
        block,
        count=1,
        flags=re.MULTILINE,
    )
    if replacements != 1:
        raise ValueError(f"材质块中找不到 texture 行：{material_name}")
    return material_text[:start] + updated + material_text[end:]


def create_material_block(material_name: str, texture_name: str) -> str:
    return (
        f"\n\nmaterial {material_name}\n"
        "{\n"
        "  technique\n"
        "  {\n"
        "    pass\n"
        "    {\n"
        "      ambient 1 1 1 1\n"
        "      diffuse 1 1 1 1\n"
        "      texture_unit\n"
        "      {\n"
        f"        texture {texture_name}\n"
        "        rotate 90\n"
        "      }\n"
        "    }\n"
        "  }\n"
        "}\n"
    )


def ensure_material_texture(material_text: str, material_name: str, texture_name: str) -> str:
    try:
        return replace_material_texture(material_text, material_name, texture_name)
    except ValueError as error:
        if f"找不到材质：{material_name}" not in str(error):
            raise
        return material_text.rstrip() + create_material_block(material_name, texture_name)


def material_script(material_name: str) -> str:
    scripts_uri = (PACKAGE_SHARE / "materials/scripts").as_uri()
    textures_uri = (PACKAGE_SHARE / "materials/textures").as_uri()
    point_targets_uri = TEXTURE_DIR.as_uri()
    return (
        "<material><script>"
        f"<uri>{scripts_uri}</uri>"
        f"<uri>{textures_uri}</uri>"
        f"<uri>{point_targets_uri}</uri>"
        f"<name>{material_name}</name>"
        "</script></material>"
    )


def patch_world_materials(world_text: str) -> str:
    for model_name, material_name in MODEL_MATERIALS.items():
        model_pattern = re.compile(
            rf'(<model\s+name="{re.escape(model_name)}"\s*>.*?</model>)', re.DOTALL
        )
        model_match = model_pattern.search(world_text)
        if model_match is None:
            raise ValueError(f"世界文件中找不到模型：{model_name}")

        model_block = model_match.group(1)
        visual_pattern = re.compile(
            r'(<visual\s+name="visual">.*?<geometry>.*?</geometry>)'
            r"<material>.*?</material>"
            r"(.*?</visual>)",
            re.DOTALL,
        )
        updated_block, replacements = visual_pattern.subn(
            rf"\1{material_script(material_name)}\2", model_block, count=1
        )
        if replacements != 1:
            raise ValueError(f"模型中找不到可替换的 visual 材质：{model_name}")
        world_text = (
            world_text[: model_match.start()]
            + updated_block
            + world_text[model_match.end() :]
        )
    return world_text


def backup_once(path: Path) -> None:
    backup = path.with_name(path.name + ".original")
    if not backup.exists():
        shutil.copy2(path, backup)


def write_atomic(path: Path, content: str) -> None:
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
        delete=False,
    ) as temporary_file:
        temporary_path = Path(temporary_file.name)
        temporary_file.write(content)
        temporary_file.flush()
        os.fsync(temporary_file.fileno())
    try:
        os.chmod(temporary_path, path.stat().st_mode)
        os.replace(temporary_path, path)
    finally:
        temporary_path.unlink(missing_ok=True)


def write_report(
    assignment: dict[str, str], ground_types: dict[str, str], seed: int | None
) -> None:
    report = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "seed": seed,
        "binary_package": str(PACKAGE_SHARE),
        "texture_directory": str(TEXTURE_DIR),
        "ground_type_assignment": ground_types,
        "assignment": assignment,
    }
    REPORT_FILE.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def main() -> int:
    args = parse_args()
    try:
        validate_runtime_files()
        prepare_texture_candidates(not args.dry_run)
        validate_ground_candidates()
        assignment, ground_types = make_assignment(random.Random(args.seed))

        print("本次二进制仿真贴图映射：")
        for material_name, texture_name in assignment.items():
            category = ground_types.get(
                material_name,
                "aerial_remaining" if material_name in AERIAL_MATERIALS else "window_candidate",
            )
            print(f"  {material_name} [{category}]: {texture_name}")
        if args.dry_run:
            print("试运行完成，未修改二进制包。")
            return 0

        material_text = MATERIAL_FILE.read_text(encoding="utf-8")
        for material_name, texture_name in assignment.items():
            material_text = ensure_material_texture(
                material_text, material_name, texture_name
            )

        world_text = WORLD_FILE.read_text(encoding="utf-8")
        patched_world = patch_world_materials(world_text)
        backup_once(MATERIAL_FILE)
        backup_once(WORLD_FILE)
        write_atomic(MATERIAL_FILE, material_text)
        write_atomic(WORLD_FILE, patched_world)

        if not args.no_report:
            write_report(assignment, ground_types, args.seed)
            print(f"映射记录已保存：{REPORT_FILE}")
        print("二进制包资源已更新，不需要重新编译；请重启 Gazebo。")
        return 0
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
