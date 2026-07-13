import unittest

from flexiv_data_collection import schema


class Stage1SchemaTests(unittest.TestCase):
    def test_single_left_arm_fills_right_arm_with_zeros(self):
        parts = schema.unitree_parts_from_single_arm(
            [1, 2, 3, 4, 5, 6, 7],
            qvel=[0.1] * 7,
            torque=[0.2] * 7,
            arm="left",
        )

        self.assertEqual(parts["left_arm"]["qpos"], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])
        self.assertEqual(parts["right_arm"]["qpos"], [0.0] * 7)
        self.assertEqual(parts["actions" if False else "left_arm"]["torque"], [0.2] * 7)
        self.assertEqual(len(schema.unitree_parts_to_vector(parts)), schema.FLEXIV_VECTOR_DIM)

    def test_validate_unitree_sample_accepts_canonical_parts(self):
        parts = schema.unitree_parts_from_single_arm([0.0] * 7, qvel=[0.0] * 7, torque=[0.0] * 7)
        sample = {"states": parts, "actions": parts}

        schema.validate_unitree_sample(sample)

    def test_split_vector_roundtrip(self):
        vector = [float(i) for i in range(schema.FLEXIV_VECTOR_DIM)]
        parts = schema.split_flexiv_vector(vector)

        self.assertEqual(parts.as_vector()[0:7], vector[0:7])
        self.assertEqual(parts.as_vector()[8:15], vector[8:15])


if __name__ == "__main__":
    unittest.main()
