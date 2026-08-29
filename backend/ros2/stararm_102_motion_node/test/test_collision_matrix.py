import unittest

from moveit_msgs.msg import AllowedCollisionEntry, AllowedCollisionMatrix
from stararm_102_motion_node.moveit_backend import MoveItBackend


class CollisionMatrixTests(unittest.TestCase):
    def test_only_detected_pairs_are_added_to_the_request_matrix(self) -> None:
        matrix = AllowedCollisionMatrix()
        matrix.entry_names = ["base_link", "link1"]
        first = AllowedCollisionEntry()
        first.enabled = [False, False]
        second = AllowedCollisionEntry()
        second.enabled = [False, False]
        matrix.entry_values = [first, second]

        MoveItBackend.allow_pairs(
            matrix,
            (("base_link", "link1"), ("link2", "link3")),
        )

        indices = {name: index for index, name in enumerate(matrix.entry_names)}
        enabled = {
            tuple(sorted((left, right)))
            for left in matrix.entry_names
            for right in matrix.entry_names
            if left != right
            and matrix.entry_values[indices[left]].enabled[indices[right]]
        }
        self.assertEqual(
            enabled,
            {("base_link", "link1"), ("link2", "link3")},
        )

    def test_non_square_source_matrix_is_an_error(self) -> None:
        matrix = AllowedCollisionMatrix()
        matrix.entry_names = ["base_link"]
        with self.assertRaisesRegex(ValueError, "不是方阵"):
            MoveItBackend.allow_pairs(matrix, (("base_link", "link1"),))


if __name__ == "__main__":
    unittest.main()
