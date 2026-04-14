import os
from pathlib import Path

import cv2
import yaml


def load_map_config(map_path):
    input_path = Path(map_path).expanduser()
    if input_path.is_absolute():
        resolved_map_path = input_path.resolve()
    else:
        package_dir = Path(__file__).resolve().parent
        repo_root = package_dir.parent
        candidates = [
            Path.cwd() / input_path,
            package_dir / input_path,
            repo_root / input_path,
        ]
        resolved_map_path = None
        for candidate in candidates:
            if candidate.exists():
                resolved_map_path = candidate.resolve()
                break
        if resolved_map_path is None:
            resolved_map_path = (Path.cwd() / input_path).resolve()
    if not resolved_map_path.exists():
        raise FileNotFoundError(f'地图文件不存在: {resolved_map_path}')

    if resolved_map_path.suffix.lower() in {'.yaml', '.yml'}:
        yaml_data = yaml.safe_load(resolved_map_path.read_text(encoding='utf-8')) or {}
        image_entry = yaml_data.get('image')
        if not image_entry:
            raise ValueError(f'地图 YAML 缺少 image 字段: {resolved_map_path}')

        image_path = Path(image_entry)
        if not image_path.is_absolute():
            image_path = (resolved_map_path.parent / image_path).resolve()

        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise FileNotFoundError(f'无法加载地图图像: {image_path}')

        height_px, width_px = image.shape[:2]
        resolution = float(yaml_data.get('resolution', 0.0))
        if resolution <= 0.0:
            raise ValueError(f'地图 YAML resolution 非法: {resolved_map_path}')

        origin = yaml_data.get('origin', [0.0, 0.0, 0.0])
        if len(origin) < 2:
            raise ValueError(f'地图 YAML origin 非法: {resolved_map_path}')

        real_length = width_px * resolution
        origin_pixel = [0, 0]
        origin_world = [float(origin[0]), float(origin[1]) + height_px * resolution]
        frame_id = 'map'

        return {
            'map_path': str(resolved_map_path),
            'image_path': str(image_path),
            'image': image,
            'width_px': width_px,
            'height_px': height_px,
            'resolution': resolution,
            'real_length': real_length,
            'origin_pixel': origin_pixel,
            'origin_world': origin_world,
            'frame_id': frame_id,
            'derived_from_yaml': True,
            'yaml_origin': [float(origin[0]), float(origin[1]), float(origin[2]) if len(origin) > 2 else 0.0],
        }

    image = cv2.imread(str(resolved_map_path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise FileNotFoundError(f'无法加载地图: {resolved_map_path}')

    height_px, width_px = image.shape[:2]
    return {
        'map_path': str(resolved_map_path),
        'image_path': str(resolved_map_path),
        'image': image,
        'width_px': width_px,
        'height_px': height_px,
        'resolution': None,
        'real_length': None,
        'origin_pixel': None,
        'origin_world': None,
        'frame_id': 'map',
        'derived_from_yaml': False,
        'yaml_origin': None,
    }
