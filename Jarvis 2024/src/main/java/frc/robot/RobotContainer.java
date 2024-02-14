// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.HomeCommand;
import frc.robot.Commands.IntakeShooterControl;
import frc.robot.Commands.JoystickDrive;
import frc.robot.Commands.PivotHold;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Shooter;
import frc.robot.Util.PIDDisplay;

public class RobotContainer {
  /**PS4-ish Controller */
  private static final Joystick controller = new Joystick(0);

  private static final Drivetrain drivetrain = new Drivetrain();
  private static final Pivot pivot = new Pivot();
  private static final Shooter shooter = new Shooter();
  private static final Intake intake = new Intake();


  private static final JoystickDrive joystickDrive = new JoystickDrive(drivetrain, controller);
  private static final HomeCommand homeCommand = new HomeCommand(drivetrain);
  private static final PivotHold pivotHoldCommand = new PivotHold(pivot);
  private static final IntakeShooterControl intakeShooterControl = new IntakeShooterControl(intake, shooter, controller);
  private static final PIDDisplay pid = new PIDDisplay();

  public RobotContainer() {
    configureBindings();
    drivetrain.setDefaultCommand(joystickDrive);
    pivot.setDefaultCommand(pivotHoldCommand);
    shooter.setDefaultCommand(intakeShooterControl);
  }

  private void configureBindings() {
    new JoystickButton(controller, 3).whileTrue(homeCommand);

    new JoystickButton(controller, 1).whileTrue(new RunCommand(() -> pivot.setPivotAngle(new Rotation2d(), false)));
    new JoystickButton(controller, 4).whileTrue(new RunCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(30), false)));

    new JoystickButton(controller, 2).onTrue(new InstantCommand(() -> pivot.resetAngleOffset()));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
