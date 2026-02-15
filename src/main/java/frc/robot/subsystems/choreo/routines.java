package frc.robot.subsystems.choreo;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class routines {
  // All routines for autonomous period are made here

  public static final AutoFactory autoFactory = RobotContainer.autoFactory;

  // Performs two cycles then does an L3 climb on driver-relative right side
  public static AutoRoutine rightCycleClimb() {
    AutoRoutine routine = autoFactory.newRoutine("rightCycleClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("Right_Trench_Neutral1"),
      routine.trajectory("Right_Trench_Shoot1"),
      routine.trajectory("Right_Trench_Neutral2"),
      routine.trajectory("Right_Trench_Shoot2"),
      routine.trajectory("Climb_Right_Front")
    };

    routine.active().onTrue(
      Commands.sequence(
        trajs[0].resetOdometry(),
        trajs[0].cmd()
      )
    );
    trajs[0].done().onTrue(
      trajs[1].cmd()
    );
    trajs[1].done().onTrue(
      trajs[2].cmd()
    );
    trajs[2].done().onTrue(
      trajs[3].cmd()
    );
    trajs[3].done().onTrue(
      trajs[4].cmd()
    );

    return routine;
  }

  // Performs two cycles then climbs L3 on driver-relative left side
  public static AutoRoutine leftCycleClimb() {
    AutoRoutine routine = autoFactory.newRoutine("leftCycleClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("Left_Trench_Neutral1"),
      routine.trajectory("Left_Trench_Shoot1"),
      routine.trajectory("Left_Trench_Neutral2"),
      routine.trajectory("Left_Trench_Shoot2"),
      routine.trajectory("Climb_Left_Front")
    };

    routine.active().onTrue(
      Commands.sequence(
        trajs[0].resetOdometry(),
        trajs[0].cmd()
      )
    );
    trajs[0].done().onTrue(
      trajs[1].cmd()
    );
    trajs[1].done().onTrue(
      trajs[2].cmd()
    );
    trajs[2].done().onTrue(
      trajs[3].cmd()
    );
    trajs[3].done().onTrue(
      trajs[4].cmd()
    );

    return routine;
  }

  // Performs two cycles then goes to outpost on driver-relative right side
  public static AutoRoutine rightCycleOutpost() {
    AutoRoutine routine = autoFactory.newRoutine("rightCycleOutpost");

    AutoTrajectory[] trajs = {
      routine.trajectory("Right_Trench_Neutral1"),
      routine.trajectory("Right_Trench_Shoot1"),
      routine.trajectory("Right_Trench_Neutral2"),
      routine.trajectory("Right_Trench_Outpost"),
      routine.trajectory("Right_Outpost")
    };

    routine.active().onTrue(
      Commands.sequence(
        trajs[0].resetOdometry(),
        trajs[0].cmd()
      )
    );
    trajs[0].done().onTrue(
      trajs[1].cmd()
    );
    trajs[1].done().onTrue(
      trajs[2].cmd()
    );
    trajs[2].done().onTrue(
      trajs[3].cmd()
    );
    trajs[3].done().onTrue(
      trajs[4].cmd()
    );

    return routine;
  }

  // Performs two cycles then goes to depot on driver-relative left side
  public static AutoRoutine leftCycleDepot() {
    AutoRoutine routine = autoFactory.newRoutine("leftCycleDepot");

    AutoTrajectory[] trajs = {
      routine.trajectory("Left_Trench_Neutral1"),
      routine.trajectory("Left_Trench_Shoot1"),
      routine.trajectory("Left_Trench_Neutral2"),
      routine.trajectory("Left_Trench_Depot"),
      routine.trajectory("Left_Depot")
    };

    routine.active().onTrue(
      Commands.sequence(
        trajs[0].resetOdometry(),
        trajs[0].cmd()
      )
    );
    trajs[0].done().onTrue(
      trajs[1].cmd()
    );
    trajs[1].done().onTrue(
      trajs[2].cmd()
    );
    trajs[2].done().onTrue(
      trajs[3].cmd()
    );
    trajs[3].done().onTrue(
      trajs[4].cmd()
    );

    return routine;
  }

  // Goes to depot from mid
  public static AutoRoutine midDepot() {
    AutoRoutine routine = autoFactory.newRoutine("midDepot");

    return routine;
  }

  // Goes to outpost from mid
  public static AutoRoutine midOutpost() {
    AutoRoutine routine = autoFactory.newRoutine("midOutpost");

    return routine;
  }
}
