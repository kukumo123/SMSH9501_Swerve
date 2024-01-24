package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final XboxController stick = new XboxController(Constants.OIConstants.kDriverControllerPort);
  
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();


  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
      () -> stick.getLeftY(),
      () -> stick.getLeftX(),
      () -> stick.getRightX(),
      () -> !stick.getAButton()));
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(stick, Button.kX.value).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading(),swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
  }
    /*return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new SwerveJoystickCmd(swerveSubsystem, () -> 0d, () -> -0.5/1.25, () -> 0d, () -> false).withTimeout(1)
      ),
      new ParallelCommandGroup(
        new SwerveJoystickCmd(swerveSubsystem, () -> 0d, () -> 1/1.25, () -> 0d, () -> false).withTimeout(1)
      ),
      new SwerveJoystickCmd(swerveSubsystem, () -> 0d, () -> 0.5, () -> 0d, () -> false).withTimeout(1),
      new SwerveJoystickCmd(swerveSubsystem, () -> 0d, () -> 0d, () -> -0.3, () -> false).withTimeout(0.3),
      new ParallelCommandGroup(
        new SwerveJoystickCmd(swerveSubsystem, () -> 0d, () -> -0.5/1.25, () -> 0d, () -> false).withTimeout(2.5)
      ),
      new ParallelCommandGroup(
        new SwerveJoystickCmd(swerveSubsystem, () -> 0d, () -> 1/1.25, () -> 0d, () -> false).withTimeout(1.5)
      ),
      new SwerveJoystickCmd(swerveSubsystem, () -> 0d, () -> 0d, () -> 0.3, () -> false).withTimeout(0.3),
      new SwerveJoystickCmd(swerveSubsystem, () -> 0d, () -> 1/1.25, () -> 0d, () -> false).withTimeout(0.5)
      );
  }*/
}