package frc.robot.subsystems.choreo;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class Routines {
  public static final AutoFactory autoFactory = RobotContainer.autoFactory;

  // Utility method to run multiple trajectories
  // TODO: Remove later to accomodate for commands or check if bind method works in sim/real life
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


  // ~9.5s
  public static AutoRoutine twoTrenchCycleRightClimb() {
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

  // ~9.7s
  public static AutoRoutine twoTrenchCycleLeftClimb() {
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


  // ~8.0s
  public static AutoRoutine twoTrenchCycleRight() {
    AutoRoutine routine = autoFactory.newRoutine("twoCycleRight");

    AutoTrajectory[] trajs = {
      routine.trajectory("RightTrenchNeutral1"),
      routine.trajectory("RightTrenchShoot1"),
      routine.trajectory("RightTrenchNeutral2"),
      routine.trajectory("RightTrenchShootAlt2"),
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~7.8s
  public static AutoRoutine twoTrenchCycleLeft() {
    AutoRoutine routine = autoFactory.newRoutine("twoCycleLeft");

    AutoTrajectory[] trajs = {
      routine.trajectory("LeftTrenchNeutral1"),
      routine.trajectory("LeftTrenchShoot1"),
      routine.trajectory("LeftTrenchNeutral2"),
      routine.trajectory("LeftTrenchShootAlt2"),
    };

    runTrajectories(routine, trajs);

    return routine;
  }

  // ~4.9s
  public static AutoRoutine oneTrenchCycleRightClimb() {
    AutoRoutine routine = autoFactory.newRoutine("oneCycleRightClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("RightTrenchNeutral1"),
      routine.trajectory("RightTrenchShoot1"),
      routine.trajectory("ClimbRightSide2")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~4.7s
  public static AutoRoutine oneTrenchCycleLeftClimb() {
    AutoRoutine routine = autoFactory.newRoutine("oneCycleLeftClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("LeftTrenchNeutral1"),
      routine.trajectory("LeftTrenchShoot1"),
      routine.trajectory("ClimbLeftSide2")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~4.4s
  public static AutoRoutine twoDepotCycleLeftClimb() {
    AutoRoutine routine = autoFactory.newRoutine("twoDepotCycleLeftClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle1"),
      routine.trajectory("DepotCycle1"),
      routine.trajectory("ClimbLeftSide3"),
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~4.5s
  public static AutoRoutine twoDepotCycleRightClimb() {
    AutoRoutine routine = autoFactory.newRoutine("twoDepotCycleRightClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle1"),
      routine.trajectory("DepotCycle1"),
      routine.trajectory("ClimbRightSide4"),
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~3.4s
  public static AutoRoutine twoDepotCycle() {
    AutoRoutine routine = autoFactory.newRoutine("twoDepotCycle");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle1"),
      routine.trajectory("DepotCycle1")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~7.5s
  public static AutoRoutine twoDepotOneOutpostCycleRightClimb() {
    AutoRoutine routine = autoFactory.newRoutine("twoDepotOneOutpostCycleRightClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle1"),
      routine.trajectory("DepotCycle1"),
      routine.trajectory("DepotSwitchOutpost"),
      routine.trajectory("OutpostCycle"),
      routine.trajectory("ClimbRightSide3")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~7.6s
  public static AutoRoutine twoDepotOneOutpostCycleLeftClimb() {
    AutoRoutine routine = autoFactory.newRoutine("twoDepotOneOutpostCycleLeftClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle1"),
      routine.trajectory("DepotCycle1"),
      routine.trajectory("DepotSwitchOutpost"),
      routine.trajectory("OutpostCycle"),
      routine.trajectory("ClimbLeftSide4")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~6.3s
  public static AutoRoutine twoDepotOneOutpostCycle() {
    AutoRoutine routine = autoFactory.newRoutine("twoDepotOneOutpostCycle");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle1"),
      routine.trajectory("DepotCycle1"),
      routine.trajectory("DepotSwitchOutpost"),
      routine.trajectory("OutpostCycle")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~6.4s
  public static AutoRoutine oneDepotOutpostCycleRightClimb() {
    AutoRoutine routine = autoFactory.newRoutine("oneDepotOutpostCycleRightClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle1"),
      routine.trajectory("DepotSwitchOutpost"),
      routine.trajectory("OutpostCycle"),
      routine.trajectory("ClimbRightSide3")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~6.5s
  public static AutoRoutine oneDepotOutpostCycleLeftClimb() {
    AutoRoutine routine = autoFactory.newRoutine("oneDepotOutpostCycleLeftClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle1"),
      routine.trajectory("DepotSwitchOutpost"),
      routine.trajectory("OutpostCycle"),
      routine.trajectory("ClimbLeftSide4")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~5.2s
  public static AutoRoutine oneDepotOutpostCycle() {
    AutoRoutine routine = autoFactory.newRoutine("oneDepotOutpostCycle");

    AutoTrajectory[] trajs = {
      routine.trajectory("MidDepot"),
      routine.trajectory("DepotCycle1"),
      routine.trajectory("DepotSwitchOutpost"),
      routine.trajectory("OutpostCycle")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~1.7s
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


  // ~7.0s
  public static AutoRoutine twoTrenchCycleOutpostCycle() {
    AutoRoutine routine = autoFactory.newRoutine("twoCycleOutpostCycle");

    AutoTrajectory[] trajs = {
      routine.trajectory("RightTrenchNeutral1"),
      routine.trajectory("RightTrenchShoot1"),
      routine.trajectory("RightTrenchNeutral2"),
      routine.trajectory("RightTrenchOutpost"),
      routine.trajectory("OutpostCycle")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~9.0s
  public static AutoRoutine twoTrenchCycleDepotCycle() {
    AutoRoutine routine = autoFactory.newRoutine("twoTrenchCycleDepotCycle");

    AutoTrajectory[] trajs = {
      routine.trajectory("LeftTrenchNeutral1"),
      routine.trajectory("LeftTrenchShoot1"),
      routine.trajectory("LeftTrenchNeutral2"),
      routine.trajectory("LeftTrenchDepot"),
      routine.trajectory("DepotCycle2")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~8.9s
  public static AutoRoutine twoBumpCycleLeftClimb() {
    AutoRoutine routine = autoFactory.newRoutine("twoBumpCycleLeftClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("LeftTrenchNeutral1"),
      routine.trajectory("LeftTrenchShoot1"),
      routine.trajectory("LeftTrenchNeutral2"),
      routine.trajectory("LeftBumpShoot1"),
      routine.trajectory("ClimbLeftSide1")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~8.8s
  public static AutoRoutine twoBumpCycleRightClimb() {
    AutoRoutine routine = autoFactory.newRoutine("twoBumpCycleRightClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("RightTrenchNeutral1"),
      routine.trajectory("RightTrenchShoot1"),
      routine.trajectory("RightTrenchNeutral2"),
      routine.trajectory("RightBumpShoot1"),
      routine.trajectory("ClimbRightSide1")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~7.7s
  public static AutoRoutine twoBumpCycleLeft() {
    AutoRoutine routine = autoFactory.newRoutine("twoBumpCycleLeft");

    AutoTrajectory[] trajs = {
      routine.trajectory("LeftTrenchNeutral1"),
      routine.trajectory("LeftTrenchShoot1"),
      routine.trajectory("LeftTrenchNeutral2"),
      routine.trajectory("LeftBumpShootAlt1")
    };

    runTrajectories(routine, trajs);
    
    return routine;
  }


  // ~7.8s
  public static AutoRoutine twoBumpCycleRight() {
    AutoRoutine routine = autoFactory.newRoutine("twoBumpCycleRight");

    AutoTrajectory[] trajs = {
      routine.trajectory("RightTrenchNeutral1"),
      routine.trajectory("RightTrenchShoot1"),
      routine.trajectory("RightTrenchNeutral2"),
      routine.trajectory("RightBumpShootAlt1")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~9.8s
  public static AutoRoutine twoCycleRightSweepClimb() {
    AutoRoutine routine = autoFactory.newRoutine("twoCycleRightSweepClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("RightTrenchSweep"),
      routine.trajectory("RightSweepShoot"),
      routine.trajectory("RightPickupSwept"),
      routine.trajectory("RightShootSwept"),
      routine.trajectory("ClimbRightSide1")
    };

    runTrajectories(routine, trajs);

    return routine;
  }


  // ~10.0s
  public static AutoRoutine twoCycleLeftSweepClimb() {
    AutoRoutine routine = autoFactory.newRoutine("twoCycleLeftSweepClimb");

    AutoTrajectory[] trajs = {
      routine.trajectory("LeftTrenchSweep"),
      routine.trajectory("LeftSweepShoot"),
      routine.trajectory("LeftPickupSwept"),
      routine.trajectory("LeftShootSwept"),
      routine.trajectory("ClimbLeftSide1")
    };

    runTrajectories(routine, trajs);

    return routine;
  }
}
