import numpy as np

def unitree_joint_order_to_isaac(unitree_joint_order):
    """
    Convert Unitree joint order to Isaac joint order.

    Args:
        unitree_joint_order (np.ndarray): Joint positions in Unitree order.

    Returns:
        np.ndarray: Joint positions in Isaac order.
    """
    isaac_joint_order = np.zeros(12)
    isaac_joint_order[0] = unitree_joint_order[3]  # FL_0
    isaac_joint_order[1] = unitree_joint_order[0]  # FR_0
    isaac_joint_order[2] = unitree_joint_order[9]  # RL_0
    isaac_joint_order[3] = unitree_joint_order[6]  # RR_0
    isaac_joint_order[4] = unitree_joint_order[4]  # FL_1
    isaac_joint_order[5] = unitree_joint_order[1]  # FR_1
    isaac_joint_order[6] = unitree_joint_order[10]  # RL_1
    isaac_joint_order[7] = unitree_joint_order[7]  # RR_1
    isaac_joint_order[8] = unitree_joint_order[5]  # FL_2
    isaac_joint_order[9] = unitree_joint_order[2]  # FR_2
    isaac_joint_order[10] = unitree_joint_order[11]  # RL_2
    isaac_joint_order[11] = unitree_joint_order[8]  # RR_2

    return isaac_joint_order

def isaac_joint_order_to_unitree(isaac_joint_order):
    """
    Convert Isaac joint order to Unitree joint order.

    Args:
        isaac_joint_order (np.ndarray): Joint positions in Isaac order.

    Returns:
        np.ndarray: Joint positions in Unitree order.
    """
    unitree_joint_order = np.zeros(12)
    unitree_joint_order[0] = isaac_joint_order[1]  # FR_0
    unitree_joint_order[1] = isaac_joint_order[5]  # FR_1
    unitree_joint_order[2] = isaac_joint_order[9]  # FR_2
    unitree_joint_order[3] = isaac_joint_order[0]  # FL_0
    unitree_joint_order[4] = isaac_joint_order[4]  # FL_1
    unitree_joint_order[5] = isaac_joint_order[8]  # FL_2
    unitree_joint_order[6] = isaac_joint_order[3]  # RR_0
    unitree_joint_order[7] = isaac_joint_order[7]  # RR_1
    unitree_joint_order[8] = isaac_joint_order[11]  # RR_2
    unitree_joint_order[9] = isaac_joint_order[2]  # RL_0
    unitree_joint_order[10] = isaac_joint_order[6]  # RL_1
    unitree_joint_order[11] = isaac_joint_order[10]  # RL_2

    return unitree_joint_order
