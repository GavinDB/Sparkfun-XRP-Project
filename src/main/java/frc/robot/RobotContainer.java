// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.BuiltInXRPDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CustomXRPDrivetrain;
import frc.robot.subsystems.XRPLever;
import frc.robot.subsystems.XRPRangefinder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CustomXRPDrivetrain xrpDrivetrain = new CustomXRPDrivetrain();
  private final XRPLever xrpLever = new XRPLever();
  private final XRPRangefinder xrpRangefinder = new XRPRangefinder();

  private final CommandXboxController xboxController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //xrpDrivetrain.setDefaultCommand(xrpDrivetrain.xrpVelocityDriveCommand(xboxController::getLeftY, xboxController::getLeftX));
    xboxController.a().onTrue(xrpLever.setLeverAngleCommand(90.0)).onFalse(xrpLever.setLeverAngleCommand(0.0));


    SmartDashboard.putData("SetLeverAngleTo90", xrpLever.setLeverAngleCommand(90.0));
    SmartDashboard.putData("SetLeverAngleTo0", xrpLever.setLeverAngleCommand(0.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
//    return printKv();
    return Commands.run(() -> xrpDrivetrain.xrpVelocityDrive(1.0, 0.0))
            .until(() -> xrpRangefinder.getRangeMeters() <= 0.3)
            .andThen(xrpDrivetrain.autoXRPTimeTurn(false, 0.5))
            .repeatedly();
  }

  public Command printKv() {
    return xrpDrivetrain.runAtFullVoltage();
  }
}
