package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.oi.OI;

import static edu.wpi.first.units.Units.Percent;
import java.util.Objects;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

public abstract class AlgaeIntakeSubsystem extends SubsystemBase {
    private static AlgaeIntakeSubsystem instance;

    public static AlgaeIntakeSubsystem getInstance() {
        return instance = Objects.requireNonNullElseGet(instance, ConcreteAlgaeIntakeSubsystem::new);
    }

    protected abstract void setPivotSpeed(double percent);

    protected abstract void setIntakeSpeed(double percent);

    protected abstract SparkFlex getPivotMotor();

    protected abstract SparkMax getIntakeMotor();

    public Command test(DoubleSupplier pivot_speed_percent, DoubleSupplier intake_speed_percent) {
        return run(() -> {
            setPivotSpeed(pivot_speed_percent.getAsDouble());
            setIntakeSpeed(intake_speed_percent.getAsDouble());
        }).finallyDo(() -> {
            setPivotSpeed(0);
            setIntakeSpeed(0);
        });
    }

    public Command intake(DoubleSupplier intakeSpeedPercent){
        return run(
            () -> setIntakeSpeed(-intakeSpeedPercent.getAsDouble())
        ).finallyDo(
            () -> setIntakeSpeed(0)
        );
    }
    
    public Command pivot(DoubleSupplier pivotSpeedPercent){
        return run(
            () -> {
                setPivotSpeed(pivotSpeedPercent.getAsDouble());
                SmartDashboard.putNumber("Pivot Motor Position", getPivotMotor().getEncoder().getPosition());
            }
        ).finallyDo(() -> setPivotSpeed(0));
    }
}
