"""
Frame transformation utilities for quadcopter_localization.

Converts between ROS standard frames (ENU/FLU) and PX4 frames (NED/FRD).
"""

import numpy as np
from geometry_msgs.msg import Quaternion, Vector3
from scipy.spatial.transform import Rotation as R


# Frame transformation matrices
# ROS ENU -> PX4 NED (position/velocity)
ENU_TO_NED = np.array([
    [0, 1, 0],   # N = E
    [1, 0, 0],   # E = N
    [0, 0, -1],  # D = -U
])

# ROS FLU -> PX4 FRD (body frame)
FLU_TO_FRD = np.array([
    [1, 0, 0],   # F = F
    [0, 0, 1],   # R = D (wait - let's be careful)
    [0, -1, 0],  # D = -R
])

# Actually, standard ROS base_link is FLU (Forward-Left-Up)
# PX4 body frame is FRD (Forward-Right-Down)
FLU_TO_FRD_BODY = np.array([
    [1, 0, 0],    # Forward -> Forward
    [0, 0, -1],   # Left -> -Down (Right)
    [0, 1, 0],    # Up -> Right
])

# Quaternion conversion: ROS (ENU) -> PX4 (NED)
# ROS quaternion: (x, y, z, w) Hamiltonian, body->world
# PX4 quaternion: (w, x, y, z) Hamiltonian, body->world (but NED world)

def enu_to_ned_position(pos_enu: np.ndarray) -> np.ndarray:
    """Convert position from ENU to NED frame."""
    return ENU_TO_NED @ pos_enu


def ned_to_enu_position(pos_ned: np.ndarray) -> np.ndarray:
    """Convert position from NED to ENU frame."""
    return ENU_TO_NED.T @ pos_ned  # ENU_TO_NED is orthogonal


def flu_to_frd_velocity(vel_flu: np.ndarray) -> np.ndarray:
    """Convert velocity from FLU to FRD body frame."""
    return FLU_TO_FRD_BODY @ vel_flu


def frd_to_flu_velocity(vel_frd: np.ndarray) -> np.ndarray:
    """Convert velocity from FRD to FLU body frame."""
    return FLU_TO_FRD_BODY.T @ vel_frd


def ros_quat_to_px4_quat(q_ros: Quaternion) -> np.ndarray:
    """
    Convert ROS quaternion (x, y, z, w) ENU frame to PX4 quaternion (w, x, y, z) NED frame.
    
    The rotation from body to world changes because world frame changes ENU->NED.
    We need to conjugate by the ENU->NED rotation.
    """
    # ROS quaternion is body->ENU (Hamiltonian, x,y,z,w)
    q_body_enu = np.array([q_ros.x, q_ros.y, q_ros.z, q_ros.w])
    
    # Rotation from ENU to NED as quaternion (Hamiltonian, w,x,y,z)
    # ENU->NED is 180deg around X, then 90deg around Z? Let's use matrix.
    R_enu_ned = ENU_TO_NED
    q_enu_ned = R.from_matrix(R_enu_ned).as_quat()  # returns [x, y, z, w]
    q_enu_ned_wxyz = np.array([q_enu_ned[3], q_enu_ned[0], q_enu_ned[1], q_enu_ned[2]])
    
    # q_body_ned = q_enu_ned * q_body_enu
    q_body_ned = quat_multiply(q_enu_ned_wxyz, q_body_enu_wxyz(q_body_enu))
    
    return q_body_ned  # Returns [w, x, y, z]


def q_body_enu_wxyz(q_xyzw: np.ndarray) -> np.ndarray:
    """Convert [x,y,z,w] to [w,x,y,z]."""
    return np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions in [w, x, y, z] format."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Conjugate of quaternion [w, x, y, z]."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def enu_quat_to_ned_quat(q_enu_xyzw: np.ndarray) -> np.ndarray:
    """
    Convert quaternion from body->ENU to body->NED.
    Input: [x, y, z, w] (ROS format)
    Output: [w, x, y, z] (PX4 format)
    """
    # ENU->NED rotation matrix
    R_enu_ned = ENU_TO_NED
    q_enu_ned = R.from_matrix(R_enu_ned).as_quat()  # [x, y, z, w]
    q_enu_ned_wxyz = np.array([q_enu_ned[3], q_enu_ned[0], q_enu_ned[1], q_enu_ned[2]])
    
    q_body_enu_wxyz = np.array([q_enu_xyzw[3], q_enu_xyzw[0], q_enu_xyzw[1], q_enu_xyzw[2]])
    
    # q_body_ned = q_enu_ned * q_body_enu
    q_body_ned_wxyz = quat_multiply(q_enu_ned_wxyz, q_body_enu_wxyz)
    
    return q_body_ned_wxyz


def transform_covariance_enu_to_ned(cov_enu: np.ndarray) -> np.ndarray:
    """
    Transform 6x6 covariance matrix from ENU to NED.
    Covariance order: [x, y, z, roll, pitch, yaw] in ENU/FLU
    """
    if cov_enu.shape != (6, 6):
        raise ValueError("Covariance must be 6x6")
    
    # Position transformation
    T_pos = ENU_TO_NED
    
    # Orientation transformation (ENU->NED for euler angles)
    # Roll (X) stays same, Pitch (Y) and Yaw (Z) swap with sign changes
    # NED: roll=X, pitch=Y, yaw=Z
    # ENU: roll=X, pitch=Y, yaw=Z but axes differ
    # Actually simpler: rotate the covariance as a tensor
    T_full = np.eye(6)
    T_full[:3, :3] = T_pos
    # For orientation, the ENU->NED rotation is the same
    T_full[3:, 3:] = T_pos
    
    cov_ned = T_full @ cov_enu @ T_full.T
    return cov_ned


def transform_covariance_flu_to_frd(cov_flu: np.ndarray) -> np.ndarray:
    """
    Transform 6x6 covariance from FLU body frame to FRD body frame.
    """
    if cov_flu.shape != (6, 6):
        raise ValueError("Covariance must be 6x6")
    
    T_pos = FLU_TO_FRD_BODY
    T_full = np.eye(6)
    T_full[:3, :3] = T_pos
    T_full[3:, 3:] = T_pos
    
    cov_frd = T_full @ cov_flu @ T_full.T
    return cov_frd


def extract_diagonal_covariance(cov: np.ndarray, indices: list) -> list:
    """Extract diagonal elements from covariance matrix."""
    return [float(cov[i, i]) for i in indices]


def build_diagonal_covariance(diag: list, size: int = 6) -> np.ndarray:
    """Build diagonal covariance matrix from list."""
    cov = np.zeros((size, size))
    for i, val in enumerate(diag):
        if i < size:
            cov[i, i] = val
    return cov


# PX4 Pose Frame Constants
class PX4PoseFrame:
    UNKNOWN = 0
    NED = 1      # North-East-Down
    FRD = 2      # Forward-Right-Down (body-fixed, constant heading offset)


class PX4VelocityFrame:
    UNKNOWN = 0
    NED = 1
    FRD = 2
    BODY_FRD = 3


def validate_px4_odometry_msg(msg) -> bool:
    """Validate that a VehicleOdometry message has reasonable values."""
    # Check for NaN
    pos = np.array(msg.position)
    vel = np.array(msg.velocity)
    q = np.array(msg.q)
    
    if np.any(np.isnan(pos)) or np.any(np.isnan(vel)) or np.any(np.isnan(q)):
        return False
    
    # Check quaternion is normalized
    q_norm = np.linalg.norm(q)
    if abs(q_norm - 1.0) > 1e-3:
        return False
    
    return True


# Convenience functions for common conversions
def convert_ros_odom_to_px4_odom(ros_odom_msg, output_frame: str = "NED") -> dict:
    """
    Convert nav_msgs/Odometry (ENU) to dict for px4_msgs/VehicleOdometry (NED/FRD).
    Returns dict with keys matching VehicleOdometry fields.
    """
    # Position ENU -> NED
    pos_enu = np.array([
        ros_odom_msg.pose.pose.position.x,
        ros_odom_msg.pose.pose.position.y,
        ros_odom_msg.pose.pose.position.z,
    ])
    pos_ned = enu_to_ned_position(pos_enu)
    
    # Orientation ROS (body->ENU) -> PX4 (body->NED)
    q_ros = ros_odom_msg.pose.pose.orientation
    q_px4 = enu_quat_to_ned_quat(np.array([q_ros.x, q_ros.y, q_ros.z, q_ros.w]))
    
    # Velocity: ROS twist is in child_frame (typically base_link FLU) -> FRD
    vel_flu = np.array([
        ros_odom_msg.twist.twist.linear.x,
        ros_odom_msg.twist.twist.linear.y,
        ros_odom_msg.twist.twist.linear.z,
    ])
    vel_frd = flu_to_frd_velocity(vel_flu)
    
    # Angular velocity: ROS is in body frame FLU -> FRD
    ang_vel_flu = np.array([
        ros_odom_msg.twist.twist.angular.x,
        ros_odom_msg.twist.twist.angular.y,
        ros_odom_msg.twist.twist.angular.z,
    ])
    ang_vel_frd = flu_to_frd_velocity(ang_vel_flu)
    
    # Covariance (6x6 pose + 6x6 twist = 36 elements each in ROS)
    # ROS: [x, y, z, rot_x, rot_y, rot_z] in ENU/FLU
    pose_cov_enu = np.array(ros_odom_msg.pose.covariance).reshape(6, 6)
    twist_cov_flu = np.array(ros_odom_msg.twist.covariance).reshape(6, 6)
    
    pose_cov_ned = transform_covariance_enu_to_ned(pose_cov_enu)
    twist_cov_frd = transform_covariance_flu_to_frd(twist_cov_flu)
    
    # PX4 expects 3x3 diagonals only
    pos_var = np.diag(pose_cov_ned[:3, :3]).tolist()
    orient_var = np.diag(pose_cov_ned[3:, 3:]).tolist()
    vel_var = np.diag(twist_cov_frd[:3, :3]).tolist()
    
    return {
        'position': pos_ned.tolist(),
        'q': q_px4.tolist(),  # [w, x, y, z]
        'velocity': vel_frd.tolist(),
        'angular_velocity': ang_vel_frd.tolist(),
        'position_variance': pos_var,
        'orientation_variance': orient_var,
        'velocity_variance': vel_var,
        'pose_frame': PX4PoseFrame.NED if output_frame == "NED" else PX4PoseFrame.FRD,
        'velocity_frame': PX4VelocityFrame.BODY_FRD,
    }