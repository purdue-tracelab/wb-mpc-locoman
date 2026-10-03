import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin

from robots.go2_yam_description.urdf.combined_urdf import build_go2_yam_urdf
from utils.robot import Go2_YAM


class Go2YAMIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = Go2_YAM()

    def test_combined_topology_and_meshes(self):
        root = ET.parse(build_go2_yam_urdf()).getroot()
        names = [link.get("name") for link in root.findall("link")]
        self.assertEqual(len(names), len(set(names)))
        children = {joint.find("child").get("link") for joint in root.findall("joint")}
        self.assertEqual(set(names) - children, {"base_link"})
        mount = root.find("joint[@name='yam_mount_joint']")
        self.assertEqual(mount.find("parent").get("link"), "base_link")
        self.assertEqual(mount.find("child").get("link"), "base")
        for mesh in root.iter("mesh"):
            self.assertTrue(Path(mesh.get("filename")).is_file())
        self.assertGreater(self.robot.robot.visual_model.ngeoms, 0)
        self.assertGreater(self.robot.robot.collision_model.ngeoms, 0)

    def test_go2_mass_and_joint_order(self):
        robot = self.robot
        self.assertEqual(robot.model.name, "go2_yam_pro")
        self.assertEqual((robot.nq, robot.nv, robot.nj, robot.nf), (25, 24, 18, 15))
        expected = [f"{leg}_{joint}_joint" for leg in ("FL", "FR", "RL", "RR")
                    for joint in ("hip", "thigh", "calf")]
        expected += [f"joint{i}" for i in range(1, 7)]
        self.assertEqual(list(robot.model.names)[2:], expected)
        # Sum of the pinned Go2 and YAM Pro source inertias, including fingers.
        self.assertAlmostEqual(pin.computeTotalMass(robot.model), 21.12851, places=5)
        for name in ("base_link", "gripper", "FL_foot", "FR_foot", "RL_foot", "RR_foot"):
            self.assertTrue(robot.model.existFrame(name))

    def test_standing_pose_and_arm_effort(self):
        robot = self.robot
        pin.framesForwardKinematics(robot.model, robot.data, robot.q0)
        for leg in ("FL", "FR", "RL", "RR"):
            frame = robot.model.getFrameId(f"{leg}_foot")
            self.assertAlmostEqual(robot.data.oMf[frame].translation[2], 0, places=7)
        self.assertTrue(np.all(robot.q0[7:] >= robot.joint_pos_min))
        self.assertTrue(np.all(robot.q0[7:] <= robot.joint_pos_max))
        gravity = pin.computeGeneralizedGravity(robot.model, robot.data, robot.q0)
        self.assertTrue(np.all(np.abs(gravity[-6:]) < robot.joint_torque_max[-6:]))
        self.assertGreater(robot.front_force_ratio, 0)
        self.assertLess(robot.front_force_ratio, 1)

    def test_reduced_arm_keeps_fingers_locked(self):
        for count in (0, 4):
            with self.subTest(arm_joints=count):
                robot = Go2_YAM(arm_joints=count)
                self.assertEqual((robot.nq, robot.nv), (19 + count, 18 + count))
                self.assertEqual(robot.nf, 15 if count else 12)
                self.assertAlmostEqual(pin.computeTotalMass(robot.model), 21.12851, places=5)
                for index in range(count + 1, 9):
                    self.assertFalse(robot.model.existJointName(f"joint{index}"))

    def test_invalid_arm_count(self):
        for count in (-1, 7, 8, 2.5):
            with self.subTest(arm_joints=count):
                with self.assertRaises(ValueError):
                    Go2_YAM(arm_joints=count)

    def test_model_paths_are_independent_of_working_directory(self):
        previous = Path.cwd()
        with TemporaryDirectory() as directory:
            try:
                os.chdir(directory)
                robot = Go2_YAM()
                self.assertEqual(robot.nv, 24)
            finally:
                os.chdir(previous)


if __name__ == "__main__":
    unittest.main()
