// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.ArmControl;
import frc.robot.Commands.HomeCommand;
import frc.robot.Commands.JoystickDrive;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Util.PIDDisplay;

public class RobotContainer {
  /**PS4-ish Controller */
  private static final Joystick controller = new Joystick(0);

  private static final Drivetrain drivetrain = new Drivetrain();
  private static final Arm arm = new Arm();
  private static final JoystickDrive joystickDrive = new JoystickDrive(drivetrain, controller);
  private static final HomeCommand homeCommand = new HomeCommand(drivetrain);
  private static final ArmControl armControl = new ArmControl(arm, controller);
  private static final PIDDisplay pid = new PIDDisplay();

  public RobotContainer() {
    configureBindings();
    drivetrain.setDefaultCommand(joystickDrive);
    arm.setDefaultCommand(armControl);
  }

  private void configureBindings() {
    new JoystickButton(controller, 3).whileTrue(homeCommand);

    // new JoystickButton(controller, 5).whileTrue(new InstantCommand( () -> arm.setIntakeSpeed(1)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
