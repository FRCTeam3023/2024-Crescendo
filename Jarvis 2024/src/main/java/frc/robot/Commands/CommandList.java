// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LED;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Pivot.PivotState;

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
            super(() -> {}, () -> intake.intakeTillSensed(1), interrupted -> {}, intake::senseNote, intake);
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
    public static final class SpinShooterCommand extends FunctionalCommand {
        public SpinShooterCommand() {
            super(() -> {shooter.setShooterRPM(Constants.ArmConstants.SHOOTER_RPM);}, () -> {}, interrupted -> {}, () -> shooter.isFlywheelReady(), shooter);
        }
    }
    public static final class ShooterStopCommand extends FunctionalCommand {
        public ShooterStopCommand() {
            super(() -> {}, () -> shooter.setShooterRPM(0), interrupted -> {}, () -> shooter.isFlywheelReady(), shooter);
        }
    }
    public static final class ShootSequenceCommand extends SequentialCommandGroup {
        public ShootSequenceCommand() {
            addCommands(
                new PrepShooterCommand(),
                new ParallelRaceGroup(
                    new SpinShooterCommand(),
                    new WaitCommand(2)
                ),
                new InstantCommand(() -> Intake.noteLoaded = false),
                new IntakeNoSensorCommand(),
                new WaitCommand(1),
                new IntakeStopCommand(),
                new ShooterStopCommand()
            );
        }
    }
   


    public static final class SetPivotStateCommand extends FunctionalCommand {
        public SetPivotStateCommand(PivotState state){
            super(() -> {}, () -> {Pivot.setPivotState(state);}, interrupted -> {}, () -> true);
        }
    }

    public static final class SetDrivetrainAimStateCommand extends FunctionalCommand{
        public SetDrivetrainAimStateCommand(boolean isAutoAim){
            super(() -> drivetrain.resetTurnController(), () -> {Drivetrain.autoAimDrivetrain = isAutoAim;}, interrupted -> {}, () -> true);
        }
    }

    public static final class FaceSpeakerDrivetrainCommand extends FunctionalCommand{
        public FaceSpeakerDrivetrainCommand(){
            super(() -> drivetrain.resetTurnController(), () -> drivetrain.driveFacingTarget(new ChassisSpeeds(), false), interrupted -> {}, () -> drivetrain.atHeadingTarget());
        }
    }

    public static final class FaceSpeakerStationaryCommand extends ParallelCommandGroup {
        public FaceSpeakerStationaryCommand() {
            addCommands(
                new SetPivotStateCommand(PivotState.AUTOAIM),
                new FaceSpeakerDrivetrainCommand()
            );
        }
    }
    public static final class FaceSpeakerMovingCommand extends ParallelCommandGroup{
        public FaceSpeakerMovingCommand(){
            addCommands(
                new SetPivotStateCommand(PivotState.AUTOAIM),
                new SetDrivetrainAimStateCommand(true)
            );

        }
    }

    public static final class PrimeShootSequenceCommand extends ParallelRaceGroup {
        public PrimeShootSequenceCommand() {
            addCommands(
                new WaitUntilCommand(() -> !Intake.noteLoaded),
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> pivot.noteClearsGround()),
                    new PrepShooterCommand(),
                    new SpinShooterCommand()
                )
            );
        }
    }
}
