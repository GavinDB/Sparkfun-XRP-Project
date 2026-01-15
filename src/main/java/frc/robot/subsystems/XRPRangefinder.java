package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class XRPRangefinder extends SubsystemBase {
    private final AnalogInput rangefinder = new AnalogInput(2);
    // the rangefinder returns a voltage from 0.0V to 5.0V based on the respective distance scale of
    // 2cm to 4 meters

    public double getRangeVoltage() {
        return rangefinder.getVoltage();
    }

    public double getRangeMeters() {
        return (rangefinder.getVoltage() / 5.0) * 4.0;
    }
}
