package frc.robot.subsystems.drivetrain;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.DriverStationHelpers;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

// TODO: copy implementations from old code; use CTRE Swerve Generator

public abstract class DrivetrainSubsystem extends SubsystemBase {

    public abstract void stop();

    protected abstract void setControl(SwerveRequest request);

    public abstract Rotation2d getOrientation();
    public abstract void resetOrientation();

    public abstract Pose2d getPose();
    public abstract void resetPoseTo(Pose2d pose);

    protected final Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> setControl(requestSupplier.get()));
    }

    public abstract Command pathfindToCommand(Pose2d targetPose);
    public abstract Command driveFieldCentricCommand();

    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     *
     * @param trajectory    The Choreo trajectory to follow.
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(ChoreoTrajectory trajectory, boolean resetPosition) {
        List<Command> commands = new ArrayList<>();

        if (resetPosition) {
            commands.add(runOnce(() -> resetPoseTo(
                switch (DriverStationHelpers.getAlliance()) {
                    case Red -> trajectory.getFlippedInitialPose();
                    case Blue -> trajectory.getInitialPose();
                }
            )));
        }

        commands.add(choreoSwerveCommand(trajectory));
        return Commands.sequence(commands.toArray(Command[]::new));
    }


    /**
     * Returns a command that makes the robot follow a Choreo path using the ChoreoLib library.
     *
     * @param pathName      The name of a path located in the "deploy/choreo" directory
     * @param resetPosition If the robot's position should be reset to the starting position of the path
     * @return A command that makes the robot follow the path
     */
    public Command followChoreoPath(String pathName, boolean resetPosition) {
        return followChoreoPath(Choreo.getTrajectory(pathName), resetPosition);
    }

    private Command choreoSwerveCommand(ChoreoTrajectory trajectory) {
        return Choreo.choreoSwerveCommand(
            trajectory,
            this::getPose,
            null, // TODO
            null, // TODO
            null, // TODO
            (ChassisSpeeds speeds) -> setControl(
                new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds)
            ),
            () -> DriverStationHelpers.getAlliance() == DriverStation.Alliance.Red,
            this
        );
    }
}
