package frc.robot.subsystems.coralIntake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.subsystems.coralIntake.constants.CoralIntakeConstants;

public class ConcreteCoralIntakeSubsystem extends CoralIntakeSubsystem {
    private final TalonFX intakePivot;
    private final SparkFlex intakeMotor;

    private final ProfiledPIDController intakePIDController;

    public ConcreteCoralIntakeSubsystem() {
        intakePivot = new TalonFX(CoralIntakeConstants.IDs.pivotID);
        intakeMotor = new SparkFlex(
                CoralIntakeConstants.IDs.intakeID,
                SparkLowLevel.MotorType.kBrushless
        );
    }
}
