"""Unit tests for the model-free parts of hybrid_segmenter (no GPU / weights needed)."""
import math

import numpy as np
import pytest

from quadcopter_perception.hybrid_segmenter import (MASK_SOURCE_NONE,
                                                    MASK_SOURCE_SAM,
                                                    MASK_SOURCE_YOLO,
                                                    _is_sam1_family, expand_box,
                                                    fuse_masks, mask_iou,
                                                    rasterize)

H, W = 100, 120


def rect(x1, y1, x2, y2):
    m = np.zeros((H, W), bool)
    m[y1:y2, x1:x2] = True
    return m


BOX = np.array([[20.0, 20.0, 60.0, 60.0]])          # 40x40 box
YOLO = rect(22, 22, 58, 58)[None]                     # coarse mask inside it


def test_mask_iou():
    assert mask_iou(rect(0, 0, 10, 10), rect(0, 0, 10, 10)) == 1.0
    assert mask_iou(rect(0, 0, 10, 10), rect(10, 10, 20, 20)) == 0.0
    assert mask_iou(np.zeros((H, W), bool), np.zeros((H, W), bool)) == 0.0
    assert mask_iou(rect(0, 0, 10, 10), rect(0, 0, 10, 5)) == pytest.approx(0.5)


def test_expand_box_clips_to_image():
    assert expand_box([0, 0, 10, 10], 0.1, W, H) == (0, 0, 12, 12)  # min 2 px margin
    assert expand_box([100, 80, 120, 100], 0.5, W, H) == (90, 70, 120, 100)


def test_good_sam_mask_replaces_yolo():
    sam = rect(20, 20, 60, 60)[None]
    masks, src, scores, agree = fuse_masks(BOX, YOLO, sam, np.array([0.95]), np.array([0]))
    assert src == [MASK_SOURCE_SAM]
    assert np.array_equal(masks[0], sam[0])
    assert scores[0] == pytest.approx(0.95)
    assert 0.8 < agree[0] <= 1.0


def test_low_sam_score_falls_back_to_yolo():
    sam = rect(20, 20, 60, 60)[None]
    masks, src, _, _ = fuse_masks(BOX, YOLO, sam, np.array([0.2]), np.array([0]))
    assert src == [MASK_SOURCE_YOLO]
    assert np.array_equal(masks[0], YOLO[0])


def test_sam_disagreeing_with_yolo_falls_back():
    # SAM grabbed a small sub-part (e.g. a shirt instead of the person)
    sam = rect(20, 20, 30, 30)[None]
    _, src, _, agree = fuse_masks(BOX, YOLO, sam, np.array([0.99]), np.array([0]),
                                  min_fill=0.0)
    assert src == [MASK_SOURCE_YOLO]
    assert agree[0] < 0.3


def test_sam_leak_is_clipped_to_grown_box():
    leaky = rect(0, 0, W, H)[None]                   # SAM segmented the whole frame
    masks, src, _, _ = fuse_masks(BOX, YOLO, leaky, np.array([0.9]), np.array([0]),
                                  min_agreement=0.0, box_margin=0.1)
    assert src == [MASK_SOURCE_SAM]
    ys, xs = np.nonzero(masks[0])
    assert xs.min() >= 16 and xs.max() < 64 and ys.min() >= 16 and ys.max() < 64


def test_empty_sam_mask_rejected():
    sam = np.zeros((1, H, W), bool)
    _, src, _, _ = fuse_masks(BOX, YOLO, sam, np.array([0.9]), np.array([0]))
    assert src == [MASK_SOURCE_YOLO]


def test_only_refined_subset_is_touched():
    boxes = np.array([[20.0, 20, 60, 60], [70, 10, 110, 50]])
    yolo = np.stack([rect(22, 22, 58, 58), rect(72, 12, 108, 48)])
    sam = rect(70, 10, 110, 50)[None]                 # SAM ran on box #1 only
    masks, src, scores, _ = fuse_masks(boxes, yolo, sam, np.array([0.9]), np.array([1]))
    assert src == [MASK_SOURCE_YOLO, MASK_SOURCE_SAM]
    assert np.array_equal(masks[0], yolo[0])
    assert math.isnan(scores[0]) and scores[1] == pytest.approx(0.9)


def test_detect_only_model_uses_sam_without_agreement_gate():
    sam = rect(25, 25, 55, 55)[None]
    masks, src, _, agree = fuse_masks(BOX, None, sam, np.array([0.9]), np.array([0]))
    assert src == [MASK_SOURCE_SAM] and math.isnan(agree[0])
    masks, src, _, _ = fuse_masks(BOX, None, sam, np.array([0.1]), np.array([0]))
    assert src == [MASK_SOURCE_NONE] and not masks[0].any()


def test_no_sam_returns_yolo_unchanged():
    masks, src, scores, _ = fuse_masks(BOX, YOLO, None, None, np.zeros(0, int))
    assert src == [MASK_SOURCE_YOLO] and np.array_equal(masks, YOLO)
    assert math.isnan(scores[0])


def test_rasterize_equal_area_most_confident_wins_overlap():
    masks = np.stack([rect(0, 0, 50, 50), rect(25, 25, 75, 75)])
    cls_m, inst_m = rasterize(masks, np.array([3, 7]), np.array([0.9, 0.4]), (H, W))
    assert cls_m[30, 30] == 4 and inst_m[30, 30] == 1      # instance 0 (conf .9) on top
    assert cls_m[70, 70] == 8 and inst_m[70, 70] == 2
    assert cls_m[90, 110] == 0 and inst_m[90, 110] == 0


def test_rasterize_small_object_on_big_one_survives():
    # person (big, conf .9) with a tie (small, conf .5) inside it
    masks = np.stack([rect(10, 10, 90, 90), rect(45, 30, 55, 70)])
    cls_m, inst_m = rasterize(masks, np.array([0, 27]), np.array([0.9, 0.5]), (H, W))
    assert cls_m[50, 50] == 28 and inst_m[50, 50] == 2      # tie visible
    assert cls_m[20, 20] == 1 and inst_m[20, 20] == 1       # person elsewhere


@pytest.mark.parametrize('name,expected', [
    ('mobile_sam.pt', True), ('sam_b.pt', True), ('/x/sam2.1_t.pt', False),
    ('sam2_b.pt', False), ('weights/SAM2.1_S.PT', False)])
def test_sam_family_detection(name, expected):
    assert _is_sam1_family(name) is expected
