// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class FeederTwoSubsystem extends SubsystemBase {
  private static final TalonFX feederMotor = new TalonFX(FeederConstants.FEEDER_ID);

  private SmartMotorControllerConfig feederControllerConfig;

  private SmartMotorController talonfxSmartMotorController;

  private final FlyWheelConfig feederConfig;

  private FlyWheel feeder;

  private final DigitalInput BeamBreakBottomFeeder;
	private final DigitalInput BeamBreakTop;
  /** Creates a new FeederTwoSubsystem. */
  public FeederTwoSubsystem() {
     feederControllerConfig = new SmartMotorControllerConfig(this)
     .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback Constants (PID Constants)
      .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
      // Feedforward Constants
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      // Telemetry name and verbosity level
      .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
      // Gearing from the motor rotor to final shaft.
      // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
      // You could also use .withGearing(12) which does the same thing.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      // Motor properties to prevent over currenting.
      .withMotorInverted(false)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));
    talonfxSmartMotorController = new TalonFXWrapper(feederMotor, DCMotor.getKrakenX60Foc(1), feederControllerConfig);
    feederConfig = new FlyWheelConfig(talonfxSmartMotorController)
      // Diameter of the flywheel.
      .withDiameter(Inches.of(4))
      // Mass of the flywheel.
      .withMass(Pounds.of(1))
      // Maximum speed of the feeder.
      .withUpperSoftLimit(RPM.of(1000))
      // Telemetry name and verbosity for the arm.
      .withTelemetry("feederMech", TelemetryVerbosity.HIGH);
    feeder = new FlyWheel(feederConfig);

    // beam brakes
    BeamBreakBottomFeeder = new DigitalInput(FeederConstants.BEAM_BREAK_BOTTOM_ID);
    BeamBreakTop = new DigitalInput(FeederConstants.BEAM_BREAK_TOP_ID);
  }

  /**
   * Gets the current velocity of the feeder.
   *
   * @return feeder velocity.
   */
  public AngularVelocity getVelocity() {return feeder.getSpeed();}

  /**
   * Set the feeder velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {return feeder.run(speed);}
  
  /**
   * Set the feeder velocity setpoint.
   *
   * @param speed Speed to set
   */
  public void setVelocitySetpoint(AngularVelocity speed) {feeder.setMechanismVelocitySetpoint(speed);}

  /**
   * Set the dutycycle of the feeder.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {return feeder.set(dutyCycle);}

  public Command stop() {
    return runOnce(() -> {
      talonfxSmartMotorController.stopClosedLoopController();
      talonfxSmartMotorController.setDutyCycle(0);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    feeder.updateTelemetry();
  }
  @Override
  public void simulationPeriodic(){
    feeder.simIterate();
  }
}
