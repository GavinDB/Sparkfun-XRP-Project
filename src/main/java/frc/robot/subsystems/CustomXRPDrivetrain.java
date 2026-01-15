package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

// This Drivetrain class was made completely by me (gavin) unlike how the
// "BuiltInXRPDrivetrain" came along with the project template, this version is
// intended to be more concise and include some of my personal notes as well as drive straight lol.
public class CustomXRPDrivetrain extends SubsystemBase {
    private static final double gearRatio =
            (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
    private static final double countsPerMotorShaftRev = 12.0;
    private static final double countsPerRevolution = countsPerMotorShaftRev * gearRatio; // 585.0
    private static final double wheelDiameterInch = 2.3622; // 60 mm

    private final XRPMotor leftMotor = new XRPMotor(0);
    private final XRPMotor rightMotor = new XRPMotor(1);

    private final Encoder leftEncoder = new Encoder(4, 5);
    private final Encoder rightEncoder = new Encoder(6, 7);

    private final PIDController leftPID = new PIDController(4.6, 0, 0.0);
    private final PIDController rightPID = new PIDController(4.6, 0, 0.0);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.07366);

//    private final DifferentialDrive m_diffDrive =
//            new DifferentialDrive(leftMotor::set, rightMotor::set);

    public CustomXRPDrivetrain() {
        // Use inches as unit for encoder distances
        leftEncoder.setDistancePerPulse((Math.PI * wheelDiameterInch) / countsPerRevolution);
        rightEncoder.setDistancePerPulse((Math.PI * wheelDiameterInch) / countsPerRevolution);
        resetEncoders();

        leftPID.reset();
        rightPID.reset();

        // Invert right side since motor is flipped
        rightMotor.setInverted(true);
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void xrpVelocityDrive(double forward, double rotation) {
        double leftTarget = -feedforward.calculate(forward + rotation);
        double rightTarget = -feedforward.calculate(forward - rotation);

        // PID calculations
        double leftOut = leftPID.calculate(leftEncoder.getRate(), leftTarget);
        double rightOut = rightPID.calculate(rightEncoder.getRate(), rightTarget);

        leftMotor.set(leftOut);
        rightMotor.set(rightOut);
    }

    public Command xrpVelocityDriveCommand(DoubleSupplier forward, DoubleSupplier rotation) {
        return run(() -> {
            xrpVelocityDrive(forward.getAsDouble(), rotation.getAsDouble());
        });
    }

    public Command autoXRPTimeTurn(boolean isRightTurn, double sec) {
        return run(() -> {
            if (isRightTurn) {
                xrpVelocityDrive(0.0, 1.0);
            } else {
                xrpVelocityDrive(0.0, -1.0);
            }
        }).withTimeout(sec);
    }

    public Command runAtFullVoltage() {
        return run(() -> {
            rightMotor.setVoltage(5.0);
            System.out.println(rightEncoder.getRate());
        }).repeatedly();
    }
}
