package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class LaunchCalculator {
    private static LaunchCalculator instance;

    public static LaunchCalculator getInstance() {
        if (instance == null)
            instance = new LaunchCalculator();
        return instance;
    }

    private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap flywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap = new InterpolatingDoubleTreeMap();

    static {
        // hoodAngleMap.put(0.96, Rotation2d.fromDegrees(10.0));

        // flywheelSpeedMap.put(0.96, 150.0);

        // timeOfFlightMap.put(5.68, 1.16);

        // passingHoodAngleMap.put(5.46, Rotation2d.fromDegrees(38.0));

        // passingFlywheelSpeedMap.put(5.46, 160.0);

        // passingTimeOfFlightMap.put(5.46, 1.27);
    }
}
