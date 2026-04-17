import math
import matplotlib.pyplot as plt

def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))

def ik_3d_3link_tip_only(l1, l2, l3, x, y, z):
    """
    3D IK with:
      - fixed psi = 90 degrees
      - fixed elbow = down

    Returns raw IK angles:
      theta0, theta1, theta2, theta3
    """
    psi_deg = 90
    psi = math.radians(psi_deg)

    # Base rotation
    theta0 = math.atan2(y, x)

    # Horizontal distance from base
    r = math.sqrt(x * x + y * y)

    # Wrist point in r-z plane
    rw = r - l3 * math.cos(psi)
    zw = z - l3 * math.sin(psi)

    d2 = rw * rw + zw * zw
    cos_theta2 = (d2 - l1 * l1 - l2 * l2) / (2 * l1 * l2)

    if cos_theta2 < -1.0 or cos_theta2 > 1.0:
        return None

    cos_theta2 = clamp(cos_theta2)

    # elbow = down
    theta2 = math.acos(cos_theta2)

    theta1 = math.atan2(zw, rw) - math.atan2(
        l2 * math.sin(theta2),
        l1 + l2 * math.cos(theta2)
    )

    theta3 = psi - theta1 - theta2

    return (
        math.degrees(theta0),
        math.degrees(theta1),
        math.degrees(theta2),
        math.degrees(theta3),
        rw,
        zw
    )

def apply_servo_mapping(theta0_raw, theta1_raw, theta2_raw, theta3_raw):
    """
    Final servo angle mapping:
      theta0 = same
      theta1 = make positive
      theta2 = 180 - theta2
      theta3 = theta3 + 45, max 95
    """
    theta0_final = theta0_raw
    theta1_final = abs(theta1_raw)
    theta2_final = 180 - theta2_raw
    theta3_final = theta3_raw + 45

    if theta3_final > 95:
        theta3_final = 95

    return theta0_final, theta1_final, theta2_final, theta3_final

def fk_3d_3link(l1, l2, l3, theta0_deg, theta1_deg, theta2_deg, theta3_deg):
    """
    Forward kinematics using RAW IK angles
    """
    t0 = math.radians(theta0_deg)
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    t3 = math.radians(theta3_deg)

    r1 = l1 * math.cos(t1)
    z1 = l1 * math.sin(t1)

    r2 = r1 + l2 * math.cos(t1 + t2)
    z2 = z1 + l2 * math.sin(t1 + t2)

    r3 = r2 + l3 * math.cos(t1 + t2 + t3)
    z3 = z2 + l3 * math.sin(t1 + t2 + t3)

    x0, y0, z0 = 0.0, 0.0, 0.0

    x1 = r1 * math.cos(t0)
    y1 = r1 * math.sin(t0)

    x2 = r2 * math.cos(t0)
    y2 = r2 * math.sin(t0)

    x3 = r3 * math.cos(t0)
    y3 = r3 * math.sin(t0)

    return (
        (x0, y0, z0),
        (x1, y1, z1),
        (x2, y2, z2),
        (x3, y3, z3)
    )

def plot_3d_arm(l1, l2, l3, x_target, y_target, z_target):
    sol = ik_3d_3link_tip_only(l1, l2, l3, x_target, y_target, z_target)

    if sol is None:
        print("Target is unreachable with these link lengths.")
        return

    theta0_raw, theta1_raw, theta2_raw, theta3_raw, rw, zw = sol

    theta0_final, theta1_final, theta2_final, theta3_final = apply_servo_mapping(
        theta0_raw, theta1_raw, theta2_raw, theta3_raw
    )

    # FK uses raw angles
    p0, p1, p2, p3 = fk_3d_3link(
        l1, l2, l3,
        theta0_raw, theta1_raw, theta2_raw, theta3_raw
    )

    # Final outputs only
    print(f"theta0 = {theta0_final:.2f}")
    print(f"theta1 = {theta1_final:.2f}")
    print(f"theta2 = {theta2_final:.2f}")
    print(f"theta3 = {theta3_final:.2f}")

    print(f"FK tip reached = ({p3[0]:.2f}, {p3[1]:.2f}, {p3[2]:.2f})")
    print(f"Target         = ({x_target:.2f}, {y_target:.2f}, {z_target:.2f})")

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Links
    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], 'r-', linewidth=3, label='L1')
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], 'g-', linewidth=3, label='L2')
    ax.plot([p2[0], p3[0]], [p2[1], p3[1]], [p2[2], p3[2]], 'b-', linewidth=3, label='L3')

    # Joints
    for p in [p0, p1, p2, p3]:
        ax.scatter(p[0], p[1], p[2], color='k', s=40)

    # Target
    ax.scatter(x_target, y_target, z_target, color='m', marker='x', s=100, label='Target')

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Robotic Arm IK (psi=90, elbow=down)")
    ax.legend()

    # Invert diagram to match your mounted arm
    ax.invert_zaxis()
    ax.invert_xaxis()

    plt.show()


# ---------------- Example run ----------------
if __name__ == "__main__":
    l1, l2, l3 = 18, 16.5, 7
    x, y, z = 14.7, 1.96, -1.5
    plot_3d_arm(l1, l2, l3, x, y, z)