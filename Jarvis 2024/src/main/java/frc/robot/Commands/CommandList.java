// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LED;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Shooter;

/** Add your docs here. */
public class CommandList {
    private static Drivetrain drivetrain;
    private static Pivot pivot;
    private static Shooter shooter;
    private static LED led;
    private static Intake intake;

    public static void setSubsystemRequirements(Drivetrain _drivetrain, Pivot _pivot, Shooter _shooter, LED _led, Intake _intake) {
        drivetrain = _drivetrain;
        pivot = _pivot;
        shooter = _shooter;
        led = _led;
        intake = _intake;
    }

    public static final class IntakeCommand extends FunctionalCommand {
        public IntakeCommand() {
            super(() -> {}, () -> intake.intakeTillSensed(1), interrupted -> {}, () -> false, intake);
        }
    }
    public static final class IntakeNoSensorCommand extends FunctionalCommand {
        public IntakeNoSensorCommand() {
            super(() -> {}, () -> intake.setIntakeSpeed(1), interrupted -> {}, () -> true, intake);
        }
    }
    public static final class IntakeStopCommand extends FunctionalCommand {
        public IntakeStopCommand() {
            super(() -> {}, () -> intake.setIntakeSpeed(0), interrupted -> {}, () -> true, intake);
        }
    }
    public static final class PrepShooterCommand extends SequentialCommandGroup {
        public PrepShooterCommand() {
            addCommands(
                new ParallelRaceGroup(
                    new RunCommand(() -> intake.setIntakeSpeed(-.9)),
                    new WaitUntilCommand(() -> !intake.senseNote()),
                    new WaitCommand(1)
                ),
                new InstantCommand(()->intake.setIntakeSpeed(0))
            );
        }
    }
    public static final class ShootCommand extends FunctionalCommand {
        public ShootCommand() {
            super(() -> {}, () -> shooter.setShooterVoltage(11.5), interrupted -> {}, () -> true, intake);
        }
    }
    public static final class ShootStopCommand extends FunctionalCommand {
        public ShootStopCommand() {
            super(() -> {}, () -> shooter.setShooterVoltage(0), interrupted -> {}, () -> true, intake);
        }
    }
    public static final class ShootSequenceCommand extends SequentialCommandGroup {
        public ShootSequenceCommand() {
            addCommands(
                new PrepShooterCommand(),
                new ShootCommand(),
                new WaitCommand(1),
                new IntakeNoSensorCommand(),
                new InstantCommand(() -> Intake.noteLoaded = false)
            );
        }
    }
}
