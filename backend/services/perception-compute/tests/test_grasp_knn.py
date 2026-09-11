"""Real patched dependency parity; no production mock or synthetic scene injection."""

import numpy as np
import torch
from graspgenx.utils.point_cloud import knn_points, point_cloud_outlier_removal


def test_knn_matches_official_dense_metric_and_self_exclusion():
    generator = torch.Generator().manual_seed(9)
    points = torch.rand((257, 3), generator=generator) * 0.1
    points[1:25] = points[0]  # More identical points than K, not distinct neighbours.
    for norm in (1, 2):
        dense = torch.cdist(points, points, p=norm,
                            compute_mode="donot_use_mm_for_euclid_dist")
        dense.fill_diagonal_(float("inf"))
        expected = dense.topk(20, largest=False).values
        actual, indices = knn_points(points, 20, norm)
        torch.testing.assert_close(actual.sort(dim=1).values, expected, atol=1e-7, rtol=1e-6)
        assert not (indices == torch.arange(len(points))[:, None]).any()
    mask = torch.cdist(points, points, p=1)
    mask.fill_diagonal_(float("inf"))
    keep = mask.topk(20, largest=False).values.mean(1) < 0.014
    filtered, removed = point_cloud_outlier_removal(points)
    torch.testing.assert_close(filtered, points[keep])
    torch.testing.assert_close(removed, points[~keep])


def test_dense_rgbd_mask_does_not_call_quadratic_cdist(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("Dense RGB-D preprocessing must not allocate N*N")
    monkeypatch.setattr(torch, "cdist", forbidden)
    points = torch.from_numpy(np.random.default_rng(4).uniform(size=(100_000, 3)).astype("float32"))
    distances, indices = knn_points(points, 20, 1)
    assert distances.shape == indices.shape == (100_000, 20)
    assert torch.isfinite(distances).all()
