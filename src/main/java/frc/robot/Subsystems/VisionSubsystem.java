// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PositionConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.DegreesPerSecond;

public class VisionSubsystem extends SubsystemBase {
  // Limelights for AprilTags
  Limelight aprilTagsA; // LL4
  Limelight aprilTagsB; // LL3G

  // Limelight for object detection
  Limelight objectDetection; // LL4

  // Swerve drive to interface with
  SwerveSubsystem swerveSubsystem;

  // Pose estimators for MT2, corresponds to Limelights
  LimelightPoseEstimator poseEstimatorMT2A;
  LimelightPoseEstimator poseEstimatorMT2B;

  // Pose estimators for MT1, corresponds to Limelights (disabled mode only)
  LimelightPoseEstimator poseEstimatorMT1A;
  LimelightPoseEstimator poseEstimatorMT1B;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    aprilTagsA = new Limelight(VisionConstants.APRILTAGS_LL4_NAME);
    aprilTagsB = new Limelight(VisionConstants.APRILTAGS_LL3G_NAME);

    poseEstimatorMT2A = aprilTagsA.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG2);
    poseEstimatorMT2B = aprilTagsB.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG2);

    poseEstimatorMT1A = aprilTagsA.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG1);
    poseEstimatorMT1B = aprilTagsB.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG1);

    objectDetection = new Limelight(VisionConstants.OBJECT_DETECTION_LL4_NAME);
    swerveSubsystem = RobotContainer.swerveSubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Gets the distance in meters from your alliance hub.
   * @return The distance between the robot and the alliance hub in meters
   */
  public double getDistanceFromAllianceHub() {
    Translation2d robotPose = swerveSubsystem.getPose().getTranslation();
    if (isRedAlliance()) {
      return robotPose.getDistance(PositionConstants.RED_HUB_T_2D);
    } else {
      return robotPose.getDistance(PositionConstants.BLUE_HUB_T_2D);
    }
  }

  /**
   * Localizes the robot by taking vision feeds from Limelights A and B, then feeding them into the swerve drive's vision estimator. Uses the MT2 algorithm. To be used always in operation.
   */
  public void localizeMT2(Limelight limelight) {
    if (swerveSubsystem.getSwerveDrive().getGyroRotation3d().getMeasureX().abs(Units.Degree) > VisionConstants.ROLL_PITCH_MT2_DISABLE_THRESHOLD  ||
        swerveSubsystem.getSwerveDrive().getGyroRotation3d().getMeasureY().abs(Units.Degree) > VisionConstants.ROLL_PITCH_MT2_DISABLE_THRESHOLD) {
      return;
    }

    // Provide LLs with gyro data
    limelight.getSettings()
            .withRobotOrientation(
                    new Orientation3d(
                            swerveSubsystem.getSwerveDrive().getGyroRotation3d(),
                            new AngularVelocity3d(
                                    DegreesPerSecond.of(0), // Cannot seem to get the pitch velocity
                                    DegreesPerSecond.of(0), // Cannot seem to get the roll velocity
                                    swerveSubsystem.getSwerveDrive().getGyro().getYawAngularVelocity().copy()
                            )
                    )
            )
            .save();

    // Get pose estimators
    Optional<PoseEstimate> visionEstimate = limelight == aprilTagsA ? poseEstimatorMT2A.getPoseEstimate() : poseEstimatorMT2B.getPoseEstimate();

    // Add vision estimates to the swerve drive if the measurements are present
    visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
      swerveSubsystem.getSwerveDrive().addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
    });
  }

  /**
   * Localizes the robot by taking vision feeds from Limelights A and B, then feeding them into the swerve drive's vision estimator. Uses the MT1 algorithm. Use <b>only</b> in disabled mode.
   */
  public void localizeMT1(Limelight limelight) {
    // Get pose estimators
    Optional<PoseEstimate> visionEstimate = limelight == aprilTagsA ? poseEstimatorMT1A.getPoseEstimate() : poseEstimatorMT1B.getPoseEstimate();

    // Add vision estimates to the swerve drive if the measurements are present
    visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
      swerveSubsystem.getSwerveDrive().addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
    });
  }

  /**
   * Checks if the Driverstation alliance is red, true means red, false means blue
   * @return whether your alliance is red or not
   */
  public boolean isRedAlliance() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Determine the delta of the robot to the  hub, where the position of the robot is subtracted by that of the hub.
   * @return the delta of the hub from the robot
   */
  private Translation2d robotHubDeltaTranslation2d() {
    Translation2d robotPosition = swerveSubsystem.getPose().getTranslation();
    return (isRedAlliance() ? PositionConstants.RED_HUB_T_2D : PositionConstants.BLUE_HUB_T_2D).minus(robotPosition);
  }

  /**
   * Find the angle at which the robot must point at in order to aim directly at the hub
   * @return the angle in radians (using WPILib's coordinate system)
   */
  private double findAngleToHubRadians() {
    Translation2d robotHubDeltaTranslation2d = robotHubDeltaTranslation2d().plus(velocityDeltaCompensator());
    return Math.atan2(robotHubDeltaTranslation2d.getY(),robotHubDeltaTranslation2d.getX()) - (Math.PI/2);
  }

  private Translation2d velocityDeltaCompensator() {
    Translation2d velocityTranslation2d = new Translation2d(swerveSubsystem.getSwerveDrive().getRobotVelocity().vxMetersPerSecond, swerveSubsystem.getSwerveDrive().getRobotVelocity().vyMetersPerSecond);
    Translation2d deltaCompensator = velocityTranslation2d.times(VisionConstants.VELOCITY_COMPENSATOR_COEFFICIENT);
    return deltaCompensator;
  }

  /**
   * Double supplier of the x value to be plugged into the direct angle swerve controller for auto-aim
   * @return DoubleSupplier of x
   */
  public DoubleSupplier xSupplier() {
    return () ->  {
      return Math.cos(findAngleToHubRadians());
    };
  }

  /**
   * Double supplier of the y value to be plugged into the direct angle swerve controller for auto-aim
   * @return DoubleSupplier of y
   */
  public DoubleSupplier ySupplier() {
    return () ->  {
      return Math.sin(findAngleToHubRadians());
    };
  }
}
