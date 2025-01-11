package frc.robot.constants;

import frc.lib.oi.OI;

import java.util.function.DoubleSupplier;

public class Controls {
    public static class Driver {
        public static final DoubleSupplier SwerveForwardAxis = OI.getInstance().driverController()::leftStickY;
        public static final DoubleSupplier SwerveStrafeAxis = OI.getInstance().driverController()::leftStickX;
        public static final DoubleSupplier SwerveRotationAxis = OI.getInstance().driverController()::rightStickX;
    }
}
