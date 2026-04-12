package frc.robot;

import edu.wpi.first.math.controller.PIDController;

public final class AutoConstants {
    // Translation PID (for forward/backward movement)
    public static final double TRANSLATION_kP = 3.0;
    public static final double TRANSLATION_kI = 0.0;
    public static final double TRANSLATION_kD = 0.0;
    public static final PIDController TRANSLATION_PID =
            new PIDController(TRANSLATION_kP, TRANSLATION_kI, TRANSLATION_kD);

    // Rotation PID (for heading). Uses continuous input across -pi..pi (radians).
    public static final double ROTATION_kP = 3.0;
    public static final double ROTATION_kI = 0.0;
    public static final double ROTATION_kD = 0.0;
    public static final PIDController ROTATION_PID =
            rotationController();

    // Cross-track PID (to correct lateral / cross-track error)
    public static final double CROSSTRACK_kP = 2.0;
    public static final double CROSSTRACK_kI = 0.0;
    public static final double CROSSTRACK_kD = 0.0;
    public static final PIDController CROSSTRACK_PID =
            new PIDController(CROSSTRACK_kP, CROSSTRACK_kI, CROSSTRACK_kD);

    private static PIDController rotationController() {
        PIDController c = new PIDController(ROTATION_kP, ROTATION_kI, ROTATION_kD);
        // Expect heading errors in radians; enable wrap-around so +179 deg -> -181 deg is handled.
        c.enableContinuousInput(-Math.PI, Math.PI);
        return c;
    }
}
