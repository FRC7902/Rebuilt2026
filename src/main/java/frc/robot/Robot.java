// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.util.StatusLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata(
            "GitDirty",
            switch (BuildConstants.DIRTY){
                case 0 -> "All changes committed";
                case 1 -> "Uncommitted changes";
                default -> "Unknown";
            }
        );
        switch (Constants.AdvantageKitConstants.currentMode){
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM:
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }
        StatusLogger.disableAutoLogging();
        Logger.start();
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_robotContainer.updateLocalization();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    private void teleopAndAutonomousInit() {
        // Check the linear intake position and set the encoder position accordingly
        m_robotContainer.calibrateLinearIntakePosition();

        CommandScheduler.getInstance().schedule(m_robotContainer.stopAllSubsystems());

        // Start the flywheel at the default RPM when teleop starts
        CommandScheduler.getInstance().schedule(m_robotContainer.m_shooterSubsystem.startFlywheelDefaultRPM());
    }

    @Override
    public void autonomousInit() {
        teleopAndAutonomousInit();

        // Zero gyro (shooter must face away from driver, towards opponent wall)
        m_robotContainer.zeroGyroWithAlliance();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        m_robotContainer.driveAngularVelocity.driveToPoseEnabled(false);
    }

    @Override
    public void teleopInit() {
        teleopAndAutonomousInit();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        m_robotContainer.driveAngularVelocity.driveToPoseEnabled(false);

        // Extend the intake to lower the hopper enough to go underneath the trench
        CommandScheduler.getInstance().schedule(m_robotContainer.m_linearIntakeSubsystem.midpoint());

        try {
            m_robotContainer.getDashboardSubsystem().setInactiveFirst(DriverStation.getGameSpecificMessage().charAt(0));
        } catch (IndexOutOfBoundsException e) {
            SmartDashboard.putString("Inactive first", "Blue");
            String inactiveFirst = SmartDashboard.getString("Inactive first", "None");
            if (inactiveFirst.equalsIgnoreCase("Red")) {
                m_robotContainer.getDashboardSubsystem().setInactiveFirst('R');
            } else if (inactiveFirst.equalsIgnoreCase("Blue")) {
                m_robotContainer.getDashboardSubsystem().setInactiveFirst('B');
            }
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
