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
import frc.robot.Commands.AimPivot;
import frc.robot.Commands.Autonomous;
import frc.robot.Commands.HomeCommand;
import frc.robot.Commands.JoystickDrive;
import frc.robot.Commands.PivotHold;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LED;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.VisionSystem;
import frc.robot.Util.PIDDisplay;

public class RobotContainer {
  /**PS4-ish Controller */
  private static final Joystick controller = new Joystick(0);
  private static final Joystick controller2 = new Joystick(1);

  private static final Drivetrain drivetrain = new Drivetrain();
  private static final VisionSystem visionSystem = new VisionSystem();
  private static final Pivot pivot = new Pivot();
  private static final Shooter shooter = new Shooter();
  private static final Intake intake = new Intake();
  private static final Autonomous autonomous = new Autonomous(pivot,shooter,intake);
  private static final LED led = new LED();


  private static final JoystickDrive joystickDrive = new JoystickDrive(drivetrain, controller);
  private static final HomeCommand homeCommand = new HomeCommand(drivetrain);
  private static final PivotHold pivotHoldCommand = new PivotHold(pivot, controller);
  private static final AimPivot aimPivot = new AimPivot(pivot, drivetrain);
  private static final PIDDisplay pid = new PIDDisplay();

  private static final Command intakeCommand = new RunCommand(() -> intake.intakeTillSensed(1), intake);
  private static final Command intakeNoSensorCommand = new InstantCommand(() -> intake.setIntakeSpeed(1));
  private static final Command intakeStopCommand = new InstantCommand(() -> intake.setIntakeSpeed(0));
  private static final Command prepShooterCommand = new SequentialCommandGroup(
            new ParallelRaceGroup(
                new RunCommand(() -> intake.setIntakeSpeed(-.25)),
                new WaitUntilCommand(() -> !intake.senseNote())
            ),
            new InstantCommand(()->intake.setIntakeSpeed(0))
        );
  private static final Command shootCommand = new InstantCommand(() -> shooter.setShooterDutyCycle(1), shooter);
  private static final Command shootStopCommand = new InstantCommand(() -> shooter.setShooterDutyCycle(0), shooter);

  private static final Command shootSequenceCommand = 
      new SequentialCommandGroup(
          prepShooterCommand,
          shootCommand,
          new WaitCommand(2),
          intakeNoSensorCommand,
          new InstantCommand(() -> Intake.noteLoaded = false)
    );
        


  private static final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
  private static final GenericEntry angleSetpoint = armTab.add("Angle Setpoint", 110).withPosition(3, 1).getEntry();

  public RobotContainer() {
    configureBindings();
    drivetrain.setDefaultCommand(joystickDrive);
    pivot.setDefaultCommand(pivotHoldCommand);
    // shooter.setDefaultCommand(intakeShooterControl);
    drivetrain.calibrateGyro();
  }

  private void configureBindings() {
    new JoystickButton(controller, 8).whileTrue(homeCommand);
    // new JoystickButton(controller, 7).onTrue(new InstantCommand(() -> pivot.resetAngleOffset(), pivot));

    new JoystickButton(controller2, 1).onTrue(new InstantCommand(() -> {Pivot.holdPosition = new Rotation2d(); Pivot.climbMode = false;}, pivot));
    new JoystickButton(controller2, 2).onTrue(new InstantCommand(() -> {Pivot.holdPosition = Rotation2d.fromDegrees(13); Pivot.climbMode = false;}, pivot));
    new JoystickButton(controller2, 4).onTrue(new InstantCommand(() -> {Pivot.holdPosition = Rotation2d.fromDegrees(angleSetpoint.getDouble(110)); Pivot.climbMode = false;}, pivot));
    new JoystickButton(controller2, 3).whileTrue(aimPivot);

    new JoystickButton(controller, 6).whileTrue(shootSequenceCommand).onFalse(new InstantCommand(() -> {intake.setIntakeSpeed(0); shooter.setShooterDutyCycle(0);}));


      //   new StartEndCommand(() -> shooter.setShooterDutyCycle(1), () -> shooter.setShooterDutyCycle(0), shooter)
      // ).onTrue(new SequentialCommandGroup(
      //   new ParallelRaceGroup(
      //     new WaitCommand(.05),
      //     new RunCommand(() -> intake.setIntakeSpeed(-.25))
      //   ),
      //   new InstantCommand(()->intake.setIntakeSpeed(0))
      // ));

    

    new JoystickButton(controller, 5).whileTrue(intakeCommand).onFalse(intakeStopCommand);

    new JoystickButton(controller, 7).onTrue(new InstantCommand(
        () -> drivetrain.setPose(new Pose2d(drivetrain.getPose().getX(),drivetrain.getPose().getY(),new Rotation2d()))
      ));

  }

  public Command getAutonomousCommand() {
    // return new HomeCommand(drivetrain);
    return new SequentialCommandGroup(
      // new InstantCommand(() -> drivetrain.calibrateGyro()),
      homeCommand,
      autonomous.getSelectedAuto()
    );
  }
}
