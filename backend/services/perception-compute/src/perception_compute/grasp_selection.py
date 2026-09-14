"""Represent nearby grasp poses with at most 100 original scored proposals.

SciPy average-linkage clustering, not a new sampler or IK solver. Distance is
the displacement of the gripper's CAD bounding-box corners under each pose:
translation and rotation therefore use the same physical length unit, without
an arbitrary degrees/metres weight. No pose or model score is synthesized.
"""
import numpy as np
from scipy.cluster.hierarchy import cut_tree, linkage


# User-requested model output budget; downstream uses this same candidate pool.
MAX_GRASP_CANDIDATES = 100


def representative_grasps(poses: np.ndarray, scores: np.ndarray,
                          gripper_corners: np.ndarray) -> np.ndarray:
    if len(poses) <= MAX_GRASP_CANDIDATES:
        return np.arange(len(poses))
    order = np.argsort(-np.asarray(scores), kind="stable")
    ordered = np.asarray(poses)[order]
    corners = (ordered[:, None, :3, 3]
               + np.einsum("nij,pj->npi", ordered[:, :3, :3], gripper_corners))
    # Exact duplicates retain their highest-scored original member first.
    features, first = np.unique(corners.reshape(len(poses), -1), axis=0, return_index=True)
    originals = order[first]
    if len(features) <= MAX_GRASP_CANDIDATES:
        return np.sort(originals)
    groups = cut_tree(linkage(features, method="average"),
                      n_clusters=MAX_GRASP_CANDIDATES).ravel()
    selected = []
    for group in np.unique(groups):
        members = originals[groups == group]
        selected.append(min(members, key=lambda i: (-scores[i], i)))
    return np.sort(selected)
