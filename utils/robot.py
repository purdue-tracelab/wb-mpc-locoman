from os.path import dirname, abspath

import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

from .combined_urdf import build_go2_yam_urdf
from .gait_sequence import GaitSequence


class Robot:
    def __init__(self, urdf_path, srdf_path, reference_pose, use_quaternion=True, lock_joints=None):
        urdf_dir = dirname(abspath(urdf_path))
        if use_quaternion:
            joint_model = pin.JointModelFreeFlyer()
        else:
            joint_model = pin.JointModelComposite()
            joint_model.addJoint(pin.JointModelTranslation())
            joint_model.addJoint(pin.JointModelSphericalZYX())

        self.robot = RobotWrapper.BuildFromURDF(urdf_path, [urdf_dir], joint_model)
        if lock_joints:
            self.robot = self.robot.buildReducedRobot(lock_joints)

        self.model = self.robot.model
        self.data = self.robot.data
        if srdf_path and reference_pose:
            pin.loadReferenceConfigurations(self.model, srdf_path)
            self.q0 = self.model.referenceConfigurations[reference_pose]
        else:
            self.q0 = self.robot.q0

        self.nq = self.model.nq
        self.nv = self.model.nv
        self.nj = self.nq - 7  # without base position and quaternion
        self.nf = 12  # forces at feet

        # Joint limits from URDF (exclude base indices)
        self.joint_pos_min = self.model.lowerPositionLimit[7:]
        self.joint_pos_max = self.model.upperPositionLimit[7:]
        self.joint_vel_max = self.model.velocityLimit[6:]
        self.joint_torque_max = self.model.effortLimit[6:]

        # Arm parameters
        self.arm_ee_frame = None  # end-effector frame in URDF
        self.arm_joints = 0  # number of joints to consider (the other ones are locked)

    def set_gait_sequence(self, gait_type, gait_period):
        self.gait_sequence = GaitSequence(gait_type, gait_period)
        self.foot_frames = [self.model.getFrameId(f) for f in self.gait_sequence.feet]


class B2(Robot):
    def __init__(self, reference_pose="standing"):
        urdf_path = "robots/b2_description/urdf/b2.urdf"
        srdf_path = "robots/b2_description/srdf/b2.srdf"
        super().__init__(urdf_path, srdf_path, reference_pose)
        
        # Weight distribution for reference forces (approx hardware measurement)
        self.front_force_ratio = 0.4


class B2_Z1(Robot):
    def __init__(self, reference_pose="standing_with_arm_up", arm_joints=6):
        urdf_path = "robots/b2_z1_description/urdf/b2_z1.urdf"
        srdf_path = "robots/b2_z1_description/srdf/b2_z1.srdf"
        lock_idx = 14 + arm_joints  # 14 is for the universe (0), base (1), and the 3 legs (2-13)
        lock_joints = range(lock_idx, 21)  # 20 is the last joint (the gripper)

        super().__init__(urdf_path, srdf_path, reference_pose, lock_joints=lock_joints)
        self.arm_joints = arm_joints  # init sets it to 0

        if self.arm_joints > 0:
            # External force at the arm end-effector
            self.arm_ee_frame = self.model.getFrameId("gripperCenter", type=pin.FIXED_JOINT)
            self.nf += 3

        # Weight distribution for reference forces (approx hardware measurement)
        self.front_force_ratio = 0.4


class Go2_YAM(Robot):
    def __init__(self, reference_pose="standing", arm_joints=6):
        urdf_path = build_go2_yam_urdf()
        srdf_path = "robots/go2_yam_description/srdf/go2_yam.srdf"
        joint_names = [f"joint{index}" for index in range(arm_joints + 1, 9)]
        lock_joints = [self._joint_id(urdf_path, name) for name in joint_names]

        super().__init__(urdf_path, srdf_path, reference_pose, lock_joints=lock_joints)
        self.arm_joints = arm_joints
        if self.arm_joints > 0:
            self.arm_ee_frame = self.model.getFrameId("gripper")
            self.nf += 3

        self.front_force_ratio = 0.4

    @staticmethod
    def _joint_id(urdf_path, joint_name):
        import pinocchio as pin

        model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        return model.getJointId(joint_name)
