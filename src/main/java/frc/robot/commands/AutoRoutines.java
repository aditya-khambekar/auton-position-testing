package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory factory;

    public AutoRoutines(AutoFactory factory) {
        this.factory = factory;
    }

    public AutoRoutine followPathRoutine(String name) {
        final AutoRoutine routine = factory.newRoutine(name);
        final AutoTrajectory simplePath = routine.trajectory(name);

        routine.active().onTrue(
                simplePath.resetOdometry()
                        .andThen(simplePath.cmd())
        );
        return routine;
    }
}