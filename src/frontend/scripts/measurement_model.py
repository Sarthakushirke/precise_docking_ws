# Suppose we have:
xA, yA, thetaA = (initial guess for Hypothesis A)
xB, yB, thetaB = (initial guess for Hypothesis B)
wA, wB = 0.5, 0.5  # start equally weighted

while True:
    # 1) PREDICTION (apply motion)
    dx, dtheta = get_odometry()  # or however you measure motion
    xA, yA, thetaA = apply_motion(xA, yA, thetaA, dx, dtheta)
    xB, yB, thetaB = apply_motion(xB, yB, thetaB, dx, dtheta)

    # 2) MEASUREMENT UPDATE (with new sensor data)
    # e.g., marker detection => marker at (xm, ym)
    # compute how likely each hypothesis is to see the marker at that location
    LA = compute_marker_likelihood(xA, yA, thetaA, xm, ym)
    LB = compute_marker_likelihood(xB, yB, thetaB, xm, ym)

    wA_new = wA * LA
    wB_new = wB * LB
    alpha = wA_new + wB_new
    wA = wA_new / alpha
    wB = wB_new / alpha

    # optional: discard a hypothesis if too small
    if wA < 0.01:
        # discard Hypothesis A
        xA, yA, thetaA = xB, yB, thetaB
        wA = 0.0
        wB = 1.0
    elif wB < 0.01:
        # discard Hypothesis B
        xB, yB, thetaB = xA, yA, thetaA
        wB = 0.0
        wA = 1.0

    # now you have updated states & weights
    # you can pick whichever has the highest weight as your "best guess"

    # rinse & repeat
