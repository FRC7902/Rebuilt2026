package frc.robot.subsystems.choreo;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class Routines {
  public static final AutoFactory autoFactory = RobotContainer.autoFactory;

  // Utility method to run multiple trajectories
  // TODO: Check if the for loop is okay to use
  public static void runTrajectories(AutoRoutine routine, AutoTrajectory[] trajs) {
    routine.active().onTrue(
      Commands.sequence(
        trajs[0].resetOdometry(),
        trajs[0].cmd()
      )
    );

    for (int i = 1; i < trajs.length; i++) {
      trajs[i - 1].chain(trajs[i]);
    }
  }

  // All routines for autonomous period are made here

  public static AutoRoutine twoCycleRightClimb() {
    AutoRoutine routine = autoFactory.newRoutine("twoCycleRightClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("RightTrenchNeutral1"),
      routine.trajectory("RightTrenchShoot1"),
      routine.trajectory("RightTrenchNeutral2"),
      routine.trajectory("RightTrenchShoot2"),
      routine.trajectory("ClimbRightSide1")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  public static AutoRoutine twoCycleLeftClimb() {
    AutoRoutine routine = autoFactory.newRoutine("twoCycleLeftClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("LeftTrenchNeutral1"),
      routine.trajectory("LeftTrenchShoot1"),
      routine.trajectory("LeftTrenchNeutral2"),
      routine.trajectory("LeftTrenchShoot2"),
      routine.trajectory("ClimbLeftSide1")
    };

    runTrajectories(routine, trajs);

    return routine;
  }

  public static AutoRoutine twoCycleRight() {
    AutoRoutine routine = autoFactory.newRoutine("twoCycleRight");

    AutoTrajectory[] trajs = {
      routine.trajectory("RightTrenchNeutral1"),
      routine.trajectory("RightTrenchShoot1"),
      routine.trajectory("RightTrenchNeutral2"),
      routine.trajectory("RightTrenchShoot2"),
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  public static AutoRoutine twoCycleLeft() {
    AutoRoutine routine = autoFactory.newRoutine("twoCycleLeft");

    AutoTrajectory[] trajs = {
      routine.trajectory("LeftTrenchNeutral1"),
      routine.trajectory("LeftTrenchShoot1"),
      routine.trajectory("LeftTrenchNeutral2"),
      routine.trajectory("LeftTrenchShoot2"),
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  public static AutoRoutine oneCycleRightClimb() {
    AutoRoutine routine = autoFactory.newRoutine("oneCycleRightClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("RightTrenchNeutral1"),
      routine.trajectory("RightTrenchShoot1"),
      routine.trajectory("ClimbRightSide2")
    };

    runTrajectories(routine, trajs);

    return routine;
  }

  public static AutoRoutine oneCycleLeftClimb() {
    AutoRoutine routine = autoFactory.newRoutine("oneCycleLeftClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("LeftTrenchNeutral1"),
      routine.trajectory("LeftTrenchShoot1"),
      routine.trajectory("ClimbLeftSide2")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  public static AutoRoutine twoDepotCycleLeftClimb() {
    AutoRoutine routine = autoFactory.newRoutine("twoDepotCycleLeftClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle"),
      routine.trajectory("DepotCycle"),
      routine.trajectory("ClimbLeftSide3"),
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  public static AutoRoutine twoDepotCycleRightClimb() {
    AutoRoutine routine = autoFactory.newRoutine("twoDepotCycleRightClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle"),
      routine.trajectory("DepotCycle"),
      routine.trajectory("ClimbRightSide4"),
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  public static AutoRoutine twoDepotCycle() {
    AutoRoutine routine = autoFactory.newRoutine("twoDepotCycle");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle"),
      routine.trajectory("DepotCycle")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  public static AutoRoutine twoDepotOneOutpostCycleRightClimb() {
    AutoRoutine routine = autoFactory.newRoutine("twoDepotOneOutpostCycleRightClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle"),
      routine.trajectory("DepotCycle"),
      routine.trajectory("DepotSwitchOutpost"),
      routine.trajectory("OutpostCycle"),
      routine.trajectory("ClimbRightSide3")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  public static AutoRoutine twoDepotOneOutpostCycleLeftClimb() {
    AutoRoutine routine = autoFactory.newRoutine("twoDepotOneOutpostCycleLeftClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle"),
      routine.trajectory("DepotCycle"),
      routine.trajectory("DepotSwitchOutpost"),
      routine.trajectory("OutpostCycle"),
      routine.trajectory("ClimbLeftSide4")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  public static AutoRoutine twoDepotOneOutpostCycle() {
    AutoRoutine routine = autoFactory.newRoutine("twoDepotOneOutpostCycle");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle"),
      routine.trajectory("DepotCycle"),
      routine.trajectory("DepotSwitchOutpost"),
      routine.trajectory("OutpostCycle")
    };

    runTrajectories(routine, trajs);

    return routine;
  }

  public static AutoRoutine oneDepotOutpostCycleRightClimb() {
    AutoRoutine routine = autoFactory.newRoutine("oneDepotOutpostCycleRightClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle"),
      routine.trajectory("DepotSwitchOutpost"),
      routine.trajectory("OutpostCycle"),
      routine.trajectory("ClimbRightSide3")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  public static AutoRoutine oneDepotOutpostCycleLeftClimb() {
    AutoRoutine routine = autoFactory.newRoutine("oneDepotOutpostCycleLeftClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle"),
      routine.trajectory("DepotSwitchOutpost"),
      routine.trajectory("OutpostCycle"),
      routine.trajectory("ClimbLeftSide4")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  public static AutoRoutine oneDepotOutpostCycle() {
    AutoRoutine routine = autoFactory.newRoutine("oneDepotOutpostCycle");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle"),
      routine.trajectory("DepotSwitchOutpost"),
      routine.trajectory("OutpostCycle")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  public static AutoRoutine preloadOnly() {
    AutoRoutine routine = autoFactory.newRoutine("preloadOnly");

    AutoTrajectory traj = routine.trajectory("MidNothing");

    routine.active().onTrue(
      Commands.sequence(
        traj.resetOdometry(),
        traj.cmd()
      )
    );

    return routine;
  }
}
