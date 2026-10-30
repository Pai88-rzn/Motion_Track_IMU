#!/usr/bin/env python3
"""
IMU Enclosure Visual Detector Training Script.
Extracts, aligns, and saves multi-view reference templates, color profiles,
and geometric features from real device photographs for real-time camera tracking.
"""

import os
import glob
import cv2
import numpy as np
from typing import Dict, Tuple, List, Optional

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
MODELS_DIR = os.path.join(SCRIPT_DIR, "models", "imu_templates")


# Known training image definitions with ground-truth bounding box regions
TRAINING_IMAGES = {
    "154827_front_textured": {
        "patterns": ["2026-10-30-154827.jpg", "*154827*.jpg"],
        "bbox": (350, 180, 560, 250),
        "view": "front",
        "description": "Front Textured Surface with Texture Pattern",
    },
    "154851_vertical_text_imu_cprp": {
        "patterns": ["2026-10-30-154851.jpg", "*154851*.jpg"],
        "bbox": (180, 160, 240, 500),
        "view": "vertical",
        "description": "Vertical Face with Embossed 'IMU CPRP' Text",
    },
    "154900_side_groove_led": {
        "patterns": ["2026-10-30-154900.jpg", "*154900*.jpg"],
        "bbox": (320, 340, 480, 85),
        "view": "side",
        "description": "Side Edge with Horizontal Grooves and Green LED",
    },
    "154913_end_cross_section": {
        "patterns": ["2026-10-30-154913.jpg", "*154913*.jpg"],
        "bbox": (560, 340, 100, 110),
        "view": "end",
        "description": "End-Face Cross Section Profile",
    },
    "154928_back_mounting_ears": {
        "patterns": ["2026-10-30-154928.jpg", "*154928*.jpg"],
        "bbox": (330, 140, 680, 290),
        "view": "back",
        "description": "Back Plate with Side Mounting Ears and Holes",
    },
}


def find_image_file(patterns: List[str]) -> Optional[str]:
    """Finds image matching one of the file patterns in root or tools."""
    search_dirs = [PROJECT_ROOT, SCRIPT_DIR, os.path.join(PROJECT_ROOT, "STM32_IMU")]
    for d in search_dirs:
        for pat in patterns:
            matches = glob.glob(os.path.join(d, pat))
            if matches:
                return matches[0]
    return None


def train_and_extract_templates() -> Dict[str, Dict]:
    """Extracts, normalizes, and saves reference templates for all views."""
    os.makedirs(MODELS_DIR, exist_ok=True)
    summary = {}

    print("=" * 64)
    print(" Training IMU Enclosure Multi-View Detector (IMU CPRP)")
    print(f" Target Output Directory: {MODELS_DIR}")
    print("=" * 64)

    for template_key, cfg in TRAINING_IMAGES.items():
        img_path = find_image_file(cfg["patterns"])
        if not img_path or not os.path.exists(img_path):
            print(f"[!] Warning: Image for {template_key} not found (patterns: {cfg['patterns']})")
            continue

        full_img = cv2.imread(img_path)
        if full_img is None:
            continue

        bx, by, bw, bh = cfg["bbox"]
        h_img, w_img = full_img.shape[:2]
        bx = max(0, min(w_img - 10, bx))
        by = max(0, min(h_img - 10, by))
        bw = max(10, min(w_img - bx, bw))
        bh = max(10, min(h_img - by, bh))

        crop = full_img[by:by + bh, bx:bx + bw]
        out_file = os.path.join(MODELS_DIR, f"{template_key}.png")
        cv2.imwrite(out_file, crop)

        # Compute color & aspect statistics
        hsv_crop = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        aspect = float(bw) / float(bh)
        mean_v = float(np.mean(hsv_crop[:, :, 2]))
        mean_s = float(np.mean(hsv_crop[:, :, 1]))

        summary[template_key] = {
            "source_image": os.path.basename(img_path),
            "output_template": out_file,
            "dimensions": (bw, bh),
            "aspect_ratio": round(aspect, 2),
            "mean_luminance_v": round(mean_v, 1),
            "mean_saturation_s": round(mean_s, 1),
            "view": cfg["view"],
            "description": cfg["description"],
        }

        print(f"[✓] Extracted: {template_key}.png")
        print(f"    Source : {os.path.basename(img_path)} ({w_img}x{h_img})")
        print(f"    Region : x={bx}, y={by}, w={bw}, h={bh} (Aspect: {aspect:.2f})")
        print(f"    Stats  : Mean V={mean_v:.1f}, Mean S={mean_s:.1f}")

    print("\n[✓] Training complete. Saved templates to", MODELS_DIR)
    return summary


if __name__ == "__main__":
    train_and_extract_templates()
