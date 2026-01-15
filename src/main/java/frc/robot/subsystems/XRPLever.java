package frc.robot.subsystems;

import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class XRPLever extends SubsystemBase {
    private final XRPServo leverServo = new XRPServo(4);

    public void setLeverAngle(double angle) {
        leverServo.setAngle(angle);
    }

    public Command setLeverAngleCommand(double angle) {
        return runOnce(() -> {
            setLeverAngle(angle);
        });
    }
}
