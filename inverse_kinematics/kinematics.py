#!coding:utf-8
import math
import numpy as np
# from transforms3d.euler import euler2mat
import config


def point_add(p1, p2):
    return [p1[0]+p2[0], p1[1]+p2[1], p1[2]+p2[2]]


def point_rotate_x(pt, angle):
    rad = angle/180.0*math.pi
    y = pt[1]*math.cos(rad) - pt[2]*math.sin(rad)
    z = pt[1]*math.sin(rad) + pt[2]*math.cos(rad)
    x = pt[0]
    return [x, y, z]


def point_rotate_y(pt, angle):
    rad = angle/180.0*math.pi
    x = pt[0]*math.cos(rad) - pt[2]*math.sin(rad)
    z = pt[0]*math.sin(rad) + pt[2]*math.cos(rad)
    y = pt[1]
    return [x, y, z]


def point_rotate_z(pt, angle):
    # 绕z轴逆时针旋转
    # print(pt, angle)
    rad = angle/180.0*math.pi
    x = pt[0]*math.cos(rad) - pt[1]*math.sin(rad)
    y = pt[0]*math.sin(rad) + pt[1]*math.cos(rad)
    z = pt[2]
    return [x, y, z]


def path_rotate_z(path, angle):
    rotate_path = [point_rotate_z(pt, angle) for pt in path]
    return rotate_path


def point_rotate(pt, rotate=[0, 0, 0]):
    x, y, z = rotate
    if x % 360 != 0:
        pt = point_rotate_x(pt, x)
    if y % 360 != 0:
        pt = point_rotate_y(pt, y)
    if z % 360 != 0:
        pt = point_rotate_z(pt, z)
    return pt


def to_angle(rad):
    return rad/math.pi*180


def leg_ki(angles):
    # 纵向为x轴，横向为y轴
    # 输入：三个关节的角度
    # 输出：脚尖相对mount点的坐标
    rads = [angles[0]/180.0*math.pi, angles[1] /
            180.0*math.pi, angles[2]/180.0*math.pi]
    abduction_angle, hip_angle, knee_angle = rads

    # 假设abduction_angle为0，先计算tip到mount在舵机平面的相对坐标
    R_servo_y = config.ABDUCTION_OFFSET
    R_servo_x = config.LEG_L1 * \
        math.sin(hip_angle) + config.LEG_L2*math.sin(knee_angle)
    R_servo_z = -(config.LEG_L1 * math.cos(hip_angle) +
                  config.LEG_L2*math.cos(knee_angle))
    # 再将相对坐标绕纵轴旋转abduction_angle得到身体平面的相对坐标
    R_mount = point_rotate_x([R_servo_x, R_servo_y, R_servo_z], angles[0])
    # 最后加上mount相对坐标原点的坐标就能得到真实坐标
    # R_body = point_add(R_mount,defaultMount[leg_index])
    return R_mount


def ik(r_mount_tip):
    # 纵向为x轴，横向为y轴
    # 输入：脚尖相对mount点的坐标
    # 输出：三个关节的角度
    x, y, z = r_mount_tip
    # 第一步，投影到yOz平面计算第一个夹角，由hip-knee-foot平面垂直于yOz可知，HKF投影为一条直线
    # Distance from the leg origin to the foot, projected into the y-z plane
    R_body_foot_yz = (y ** 2 + z ** 2) ** 0.5

    # R_hip_foot_yz：hip-Y长度 由body-hip与hip-knee-foot平面垂直，body-hip始终垂直于投影hip-Y即HY
    # Distance from the leg's forward/back point of rotation to the foot
    R_hip_foot_yz = (R_body_foot_yz ** 2 - config.ABDUCTION_OFFSET ** 2) ** 0.5

    # 角Y-body-hip
    # Interior angle of the right triangle formed in the y-z plane by the leg that is coincident to the ab/adduction axis
    # For feet 2 (front left) and 4 (back left), the abduction offset is positive, for the right feet, the abduction offset is negative.
    arccos_argument = config.ABDUCTION_OFFSET / R_body_foot_yz
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
    phi = np.arccos(arccos_argument)

    # 角Y-body-O
    # Angle of the y-z projection of the hip-to-foot vector, relative to the positive y-axis
    hip_foot_angle = np.arctan2(z, y)

    # abduction_angle 等于角Y-body-O与角Y-body-hip之和
    # Ab/adduction angle, relative to the positive y-axis
    abduction_angle = phi + hip_foot_angle

    # foot-Y垂直于hip-Y,在三角形hip-Y-foot中计算角Y-hip-foot与hip-foot长度
    # theta：角Y-hip-foot
    # theta: Angle between the tilted negative z-axis and the hip-to-foot vector
    theta = np.arctan2(x, R_hip_foot_yz)
    # R_hip_foot：hip-foot长度
    # Distance between the hip and foot
    R_hip_foot = (R_hip_foot_yz ** 2 + x ** 2) ** 0.5

    # trident：角foot-hip-knee，由余弦定理计算
    # Angle between the line going from hip to foot and the link L1
    arccos_argument = (config.LEG_L1 ** 2 + R_hip_foot ** 2 - config.LEG_L2 ** 2) / (
        2 * config.LEG_L1 * R_hip_foot
    )
    arccos_argument = np.clip(arccos_argument, -1, 1)
    trident = np.arccos(arccos_argument)

    # hip_angle：角Y-hip-knee 等于 角Y-hip-foot与角foot-hip-knee之和
    # Angle of the first link relative to the tilted negative z axis
    hip_angle = theta + trident

    # 过点K(knee)在HKF(hip-knee-foot)平面作一条辅助线垂直于FY（foot-Y），设交点为N
    # 所求knee_angle 即为角NKF(N-knee-foot),等于角HKF(hip-knee-foot)减去角HKN(hip-knee-N，等于180 - hip_angle)
    # beta：角hip-knee-foot
    # Angle between the leg links L1 and L2
    arccos_argument = (config.LEG_L1 ** 2 + config.LEG_L2 ** 2 - R_hip_foot ** 2) / (
        2 * config.LEG_L1 * config.LEG_L2
    )
    arccos_argument = np.clip(arccos_argument, -1, 1)
    beta = np.arccos(arccos_argument)

    # Angle of the second link relative to the tilted negative z axis
    # knee_angle = beta -  (np.pi - hip_angle)
    knee_angle = hip_angle - (np.pi - beta)

    return np.array([abduction_angle, hip_angle, knee_angle])*180.0/math.pi


def leg_explicit_inverse_kinematics(r_body_foot, leg_index, config):
    """Find the joint angles corresponding to the given body-relative foot position for a given leg and configuration

    Parameters
    ----------
    r_body_foot : [type]
        [description]
    leg_index : [type]
        [description]
    config : [type]
        [description]

    Returns
    -------
    numpy array (3)
        Array of corresponding joint angles.
    """
    # 对于讨论的腿而言，以挂载点的地面投影为原点O建立空间直角坐标系
    # 脚尖到yOz平面的投影为Y(0,y,0)
    # 脚尖到xOz平面的投影为X(x,0,0)
    # 挂载点为body: B，第一个关节为hip: H，第二个关节为knee: K，脚尖为foot: T
    # 不难看出，body-hip的运动轨迹在yOz平面上
    # 又因为body-hip始终垂直于hip-knee-foot平面，可以得出hip-knee-foot平面垂直于yOz
    # Abdaction: body-hip BH
    # L1: hip-knee HK
    # L2: knee-foot KF
    # 需要求的三个角为
    # abduction_angle：body-hip相对于y轴正方向的夹角（即body-hip与xOz的夹角-90度）
    # hip_angle hip-knee相对于x轴的夹角（即hip-knee与yOz的夹角）
    # knee_angle knee-foot相对于x轴的夹角（即knee-foot与yOz的夹角）
    # x,y,z
    (x, y, z) = r_body_foot

    # 第一步，投影到yOz平面计算第一个夹角，由hip-knee-foot平面垂直于yOz可知，HKF投影为一条直线
    # Distance from the leg origin to the foot, projected into the y-z plane
    R_body_foot_yz = (y ** 2 + z ** 2) ** 0.5

    # R_hip_foot_yz：hip-Y长度 由body-hip与hip-knee-foot平面垂直，body-hip始终垂直于投影hip-Y即HY
    # Distance from the leg's forward/back point of rotation to the foot
    R_hip_foot_yz = (R_body_foot_yz ** 2 - config.ABDUCTION_OFFSET ** 2) ** 0.5

    # 角Y-body-hip
    # Interior angle of the right triangle formed in the y-z plane by the leg that is coincident to the ab/adduction axis
    # For feet 2 (front left) and 4 (back left), the abduction offset is positive, for the right feet, the abduction offset is negative.
    arccos_argument = config.ABDUCTION_OFFSETS[leg_index] / R_body_foot_yz
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
    phi = np.arccos(arccos_argument)

    # 角Y-body-O
    # Angle of the y-z projection of the hip-to-foot vector, relative to the positive y-axis
    hip_foot_angle = np.arctan2(z, y)

    # abduction_angle 等于角Y-body-O与角Y-body-hip之和
    # Ab/adduction angle, relative to the positive y-axis
    abduction_angle = phi + hip_foot_angle

    # foot-Y垂直于hip-Y,在三角形hip-Y-foot中计算角Y-hip-foot与hip-foot长度
    # theta：角Y-hip-foot
    # theta: Angle between the tilted negative z-axis and the hip-to-foot vector
    theta = np.arctan2(-x, R_hip_foot_yz)
    # R_hip_foot：hip-foot长度
    # Distance between the hip and foot
    R_hip_foot = (R_hip_foot_yz ** 2 + x ** 2) ** 0.5

    # trident：角foot-hip-knee，由余弦定理计算
    # Angle between the line going from hip to foot and the link L1
    arccos_argument = (config.LEG_L1 ** 2 + R_hip_foot ** 2 - config.LEG_L2 ** 2) / (
        2 * config.LEG_L1 * R_hip_foot
    )
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
    trident = np.arccos(arccos_argument)

    # hip_angle：角Y-hip-knee 等于 角Y-hip-foot与角foot-hip-knee之和
    # Angle of the first link relative to the tilted negative z axis
    hip_angle = theta + trident

    # 过点K(knee)在HKF(hip-knee-foot)平面作一条辅助线垂直于FY（foot-Y），设交点为N
    # 所求knee_angle 即为角NKF(N-knee-foot),等于角HKF(hip-knee-foot)减去角HKN(hip-knee-N，等于180 - hip_angle)
    # beta：角hip-knee-foot
    # Angle between the leg links L1 and L2
    arccos_argument = (config.LEG_L1 ** 2 + config.LEG_L2 ** 2 - R_hip_foot ** 2) / (
        2 * config.LEG_L1 * config.LEG_L2
    )
    arccos_argument = np.clip(arccos_argument, -0.99, 0.99)
    beta = np.arccos(arccos_argument)

    # Angle of the second link relative to the tilted negative z axis
    # knee_angle = beta -  (np.pi - hip_angle)
    knee_angle = hip_angle - (np.pi - beta)

    return np.array([abduction_angle, hip_angle, knee_angle])


def four_legs_inverse_kinematics(r_body_foot, config):
    """Find the joint angles for all twelve DOF correspoinding to the given matrix of body-relative foot positions.

    Parameters
    ----------
    r_body_foot : numpy array (3,4)
        Matrix of the body-frame foot positions. Each column corresponds to a separate foot.
    config : Config object
        Object of robot configuration parameters.

    Returns
    -------
    numpy array (3,4)
        Matrix of corresponding joint angles.
    """
    alpha = np.zeros((3, 4))
    for i in range(4):
        body_offset = config.LEG_ORIGINS[:, i]
        alpha[:, i] = leg_explicit_inverse_kinematics(
            r_body_foot[:, i] - body_offset, i, config
        )
    return alpha


def tip_to_leg(pt):
    defaultPosition = leg_ki([0, 45, -45])
    return point_add(defaultPosition, pt)


if __name__ == '__main__':
    # pt = leg_ki([0, 60, -30])
    pt = leg_ki([0, 40, 0])
    print(pt)
    print(ik(pt))
