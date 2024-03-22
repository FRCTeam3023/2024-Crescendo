// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Commands.AimRobotDrive;
import frc.robot.Commands.AmpOrient;
import frc.robot.Commands.Autonomous;
import frc.robot.Commands.CommandList;
import frc.robot.Commands.CommandList.*;
import frc.robot.Commands.HomeCommand;
import frc.robot.Commands.JoystickDrive;
import frc.robot.Commands.PivotHold;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LED;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.VisionSystem;
import frc.robot.Subsystems.Pivot.PivotState;
import frc.robot.Util.PIDDisplay;

public class RobotContainer {
  /**PS4-ish Controller */
  private static final Joystick controller = new Joystick(0);
  private static final Joystick controller2 = new Joystick(1);

  private static final Drivetrain drivetrain = new Drivetrain();
  private static final VisionSystem visionSystem = new VisionSystem();
  private static final Pivot pivot = new Pivot();
  private static final Shooter shooter = new Shooter(controller);
  private static final Intake intake = new Intake();
  private static Autonomous autonomous;
  private static final LED led = new LED();

   private static final PIDDisplay pid = new PIDDisplay();

  private static final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
  private static final GenericEntry angleSetpoint = armTab.add("Angle Setpoint", 110).withPosition(4, 2).getEntry();

  public RobotContainer() {
    CommandList.setSubsystemRequirements(drivetrain, pivot, shooter, led, intake);
    configureBindings();
    drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, controller, controller2));
    pivot.setDefaultCommand(new PivotHold(pivot, controller2));
    // shooter.setDefaultCommand(intakeShooterControl);
    drivetrain.calibrateGyro();

    autonomous =  new Autonomous(pivot,shooter,intake, drivetrain);
  }

  private void configureBindings() {
//#region Bindings
    /*
     * Button layout on controller 1:
     * 1 - A - Calibrate gyro
     * 2 - B - Amp orient
     * 3 - X - Toggle vision system
     * 4 - Y - Toggle field relative drive
     * 5 - leftBumper - Intake
     * 6 - rightBumper - Shoot
     * 7 - reset - Set heading to 0
     * 8 - Start - Home swerve
     * 9 - Left Joystick Pushbutton
     * 10 - Right Joystick Pushbutton
     *
     * Button layout on controller 2:
     * 1 - A - Set pivot to local 0
     * 2 - B - Set pivot to fire when flush at speaker
     * 3 - X - Auto aim
     * 4 - Y - Aim at amp
     * 5 - leftBumper
     * 6 - rightBumper
     * 7 - reset
     * 8 - Start
     * 9 - Left Joystick Pushbutton
     * 10 - Right Joystick Pushbutton
     */
//#endregion
    
    new JoystickButton(controller, 8).whileTrue(new HomeCommand(drivetrain));
    new JoystickButton(controller, 6)
      .whileTrue(new ShootSequenceCommand())
      .onFalse(new ParallelCommandGroup(
        new IntakeStopCommand(),
        new ShooterStopCommand()
      ));
    new JoystickButton(controller, 5).whileTrue(new IntakeCommand()).whileFalse(new IntakeStopCommand());
    new JoystickButton(controller, 7).onTrue(new InstantCommand(
        () -> drivetrain.setPose(new Pose2d(drivetrain.getPose().getX(),drivetrain.getPose().getY(),new Rotation2d()))
      ));

    new JoystickButton(controller, 1).onTrue(new InstantCommand(() -> drivetrain.calibrateGyro()));
    new JoystickButton(controller, 2).whileTrue(new AmpOrient(drivetrain));
    new JoystickButton(controller, 3).onTrue(new InstantCommand(() -> VisionSystem.disabled = !VisionSystem.disabled));
    new JoystickButton(controller, 4).onTrue(new InstantCommand(() -> JoystickDrive.fieldRelativeDrive = !JoystickDrive.fieldRelativeDrive));

    new JoystickButton(controller2, 1).onTrue(new SequentialCommandGroup(
      new ShooterStopCommand(),
      new ParallelRaceGroup(
        new WaitUntilCommand(() -> !Intake.noteLoaded),
        new IntakeCommand(),
        new WaitCommand(0.5)
      ),
      new IntakeStopCommand(),
      new SetPivotStateCommand(PivotState.PICKUP)
    ));
    
    new JoystickButton(controller2, 2).onTrue(new SequentialCommandGroup(
      new SetPivotStateCommand(PivotState.SPEAKER),
      new PrimeShootSequenceCommand()
    ));

    new JoystickButton(controller2, 3).onTrue(new SequentialCommandGroup(
      new SetPivotStateCommand(PivotState.SPEAKER),
      new InstantCommand(() -> drivetrain.resetTurnController()),
      new PrimeShootSequenceCommand()
    )).whileTrue(new AimRobotDrive(drivetrain,controller));

    new JoystickButton(controller2, 4).onTrue(new SequentialCommandGroup(
      new SetPivotStateCommand(PivotState.AMP),
      new PrimeShootSequenceCommand()
    ));
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new HomeCommand(drivetrain),
      new ParallelCommandGroup(
        new RunCommand(() -> pivot.approachCurrentState()),
        autonomous.getSelectedAuto()
      )
    );
  }
}
