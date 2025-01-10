package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory factory;

    public AutoRoutines(AutoFactory factory) {
        this.factory = factory;
    }

    public AutoRoutine branching2024Auto() {
        AutoRoutine routine = autoFactory.newRoutine("branching2024Auto");

        // This routine uses segments between pre-defined handoff points.
        // Expand the tooltip for information about their naming convention ->

        // Load the routine's trajectories
        AutoTrajectory startToC2 = routine.trajectory("startToC2");
        AutoTrajectory C2toM1 = routine.trajectory("C2toM1");
        AutoTrajectory M1toM2 = routine.trajectory("M1toM2");
        AutoTrajectory M2toM3 = routine.trajectory("M2toM3");
        AutoTrajectory M1toScore = routine.trajectory("M1toScore");
        AutoTrajectory M2toScore = routine.trajectory("M2toScore");
        AutoTrajectory M3toScore = routine.trajectory("M3toScore");
        AutoTrajectory scoreToM2 = routine.trajectory("scoreToM2");
        AutoTrajectory scoreToM3 = routine.trajectory("scoreToM3");

        // When the routine starts, reset odometry, shoot the first gamepiece, then go to the "C2" location
        routine.active().onTrue(
                Commands.sequence(
                        startToC2.resetOdometry(),
                        shooterSubsystem.shoot(),
                        startToC2.cmd()
                )
        );

        // Pick up and shoot the gamepiece at the "C2" location, then go to the "M1" location
        startToC2.active().whileTrue(intakeSubsystem.intake());
        startToC2.done().onTrue(shooterSubsystem.shoot().andThen(C2toM1.cmd()));

        // Run the intake when we are approaching a gamepiece
        routine.anyActive(C2toM1, scoreToM2, scoreToM3, M1toM2, M2toM3) //
                .whileTrue(intakeSubsystem.intake());

        // If we picked up the gamepiece, go score, then go to the next midline location
        // If we didn't pick up the gamepiece, go directly to the next midline location

        // M1
        Trigger atM1 = C2toM1.done();
        atM1.and(shooterSubsystem::noGamepiece).onTrue(M1toM2.cmd());
        atM1.and(shooterSubsystem::hasGamepiece).onTrue(M1toScore.cmd());
        M1toScore.done().onTrue(shooterSubsystem.shoot().andThen(scoreToM2.cmd()));

        // M2
        Trigger atM2 = routine.anyDone(scoreToM2, M1toM2); //
        atM2.and(shooterSubsystem::noGamepiece).onTrue(M2toM3.cmd());
        atM2.and(shooterSubsystem::hasGamepiece).onTrue(M2toScore.cmd());
        M2toScore.done().onTrue(shooterSubsystem.shoot().andThen(scoreToM3.cmd()));

        // M3
        Trigger atM3 = routine.anyDone(scoreToM3, M2toM3);
        atM3.and(shooterSubsystem::hasGamepiece).onTrue(M3toScore.cmd());
        M3toScore.done().onTrue(shooterSubsystem.shoot());

        return routine;
    }
}