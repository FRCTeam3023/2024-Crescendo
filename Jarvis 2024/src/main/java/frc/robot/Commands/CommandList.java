// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import java.util.Timer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
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
        /**Intake command until intake sensor triggered */
        public IntakeCommand() {
            super(() -> {}, () -> intake.intakeTillSensed(1), interrupted -> {intake.setIntakeSpeed(0);}, intake::senseNote, intake);
        }
    }
    public static final class IntakeNoSensorCommand extends FunctionalCommand {
        /**Intake note but without stopping on sensor trigger */
        public IntakeNoSensorCommand() {
            super(() -> {}, () -> intake.setIntakeSpeed(1), interrupted -> {}, () -> true, intake);
        }
    }
    public static final class IntakeStopCommand extends FunctionalCommand {
        /**Set intake speed to 0 */
        public IntakeStopCommand() {
            super(() -> {}, () -> intake.setIntakeSpeed(0), interrupted -> {}, () -> true, intake);
        }
    }

    public static final class PrepShooterCommand extends SequentialCommandGroup {
        /**Prep intake for shooting, bypassed if the note is not sensed by the sensor */
        public PrepShooterCommand() {
            addCommands(
                new ParallelRaceGroup(
                    new WaitUntilCommand(() -> !Intake.noteSensed),
                    new RunCommand(() -> intake.setIntakeSpeed(-.5),intake),
                    new WaitCommand(1)
                ),
                new InstantCommand(()->intake.setIntakeSpeed(0),intake)
            );
        }
    }

    public static final class SpinShooterCommand extends FunctionalCommand {
        /**Spins shooter up to target RPM - Changes if at AMP position */
        public SpinShooterCommand() {
            super(() -> {
                    if(Pivot.getPivotState() == PivotState.AMP){
                        shooter.setShooterRPM(Constants.ArmConstants.SHOOTER_RPM_AMP);
                    }else{
                        shooter.setShooterRPM(Constants.ArmConstants.SHOOTER_RPM);
                    }
                    // shooter.setShooterRPM(Constants.ArmConstants.SHOOTER_RPM);
                }, () -> {}, interrupted -> {}, () -> shooter.isFlywheelReady(), shooter);
        }
    }

    public static final class ShooterStopCommand extends FunctionalCommand {
        /**Set shooter rpm to 0 */
        public ShooterStopCommand() {
            super(() -> {}, () -> shooter.setShooterRPM(0), interrupted -> {}, () -> shooter.isFlywheelReady(), shooter);
        }
    }

    public static final class ShooterStopAutoCommand extends FunctionalCommand {
        /**Set shooter rpm to 0 */
        public ShooterStopAutoCommand() {
            super(() -> {}, () -> shooter.setShooterRPM(0), interrupted -> {}, () -> true, shooter);
        }
    }

    public static final class ShootSequenceCommand extends SequentialCommandGroup {
        /**Prep shooter and shoot sequence, will skip to shooting if previously intaked and spun up */
        public ShootSequenceCommand() {
            addCommands(
                new PrepShooterCommand(),
                new ParallelRaceGroup(
                    new SpinShooterCommand(),
                    new WaitCommand(2)
                ),
                new InstantCommand(() -> Intake.noteLoaded = false),
                new IntakeNoSensorCommand(),
                new WaitCommand(0.5),
                new IntakeStopCommand(),
                new ShooterStopCommand()
            );
        }
    }

    public static final class ShootSequenceAutoCommand extends SequentialCommandGroup {
        /**Prep shooter and shoot sequence, will skip to shooting if previously intaked and spun up */
        public ShootSequenceAutoCommand() {
            addCommands(
                new WaitUntilCommand(() -> Intake.noteLoaded),
                new PrepShooterCommand(),
                new ParallelRaceGroup(
                    new SpinShooterCommand(),
                    new WaitCommand(2)
                ),
                new WaitUntilCommand(() -> {if(Pivot.getPivotState() == PivotState.AUTOAIM){return drivetrain.atHeadingTarget();}else{return true;}}),
                new InstantCommand(() -> Intake.noteLoaded = false),
                new IntakeNoSensorCommand(),
                new WaitCommand(0.4),
                new IntakeStopCommand(),
                new ShooterStopAutoCommand()
            );
        }
    }
   

    public static final class SetPivotStateCommand extends FunctionalCommand {
        /**Set the current target state for the pivot assembly 
         * @param state pivot state
        */
        public SetPivotStateCommand(PivotState state){
            super(() -> {}, () -> {Pivot.setPivotState(state); Pivot.climbMode = false;}, interrupted -> {}, () -> true);
        }
    }

    public static final class SetDrivetrainAimStateCommand extends FunctionalCommand{
        /**set the drivetrain state for auto aim during autonomous path following
         * @param isAutoAim auto aim state - if true the drivetrain will override the target heading in pathplanner and aim towards target
         */
        public SetDrivetrainAimStateCommand(boolean isAutoAim){
            super(() -> drivetrain.resetTurnController(), () -> {Drivetrain.autoAimDrivetrain = isAutoAim;}, interrupted -> {}, () -> true);
        }
    }

    public static final class FaceSpeakerDrivetrainCommand extends FunctionalCommand{
        /**Command to rotate the drivetrain towards the target, use setDrivetrainAimStateCommand() for heading auto aim during autonomous*/
        public FaceSpeakerDrivetrainCommand(){
            super(() -> {drivetrain.resetTurnController();}, () -> drivetrain.driveFacingTarget(new ChassisSpeeds(), false), interrupted -> {drivetrain.stop();}, () -> drivetrain.atHeadingTarget(), drivetrain);
        }
    }

    public static final class FaceSpeakerStationaryCommand extends ParallelCommandGroup {
        /**Combined command for auto aim on drivetrain and pivot for when stationary in autonomous */
        public FaceSpeakerStationaryCommand() {
            addCommands(
                new SetPivotStateCommand(PivotState.AUTOAIM),
                new FaceSpeakerDrivetrainCommand()
            );
        }
    }
    public static final class FaceSpeakerMovingCommand extends ParallelCommandGroup{
        /**Combined command for auto aim on drivetrain and pivot for when moving along path */
        public FaceSpeakerMovingCommand(){
            addCommands(
                new SetPivotStateCommand(PivotState.AUTOAIM),
                new SetDrivetrainAimStateCommand(true)
            );
        }
    }

    public static final class StopAimCommand extends ParallelCommandGroup{
        /**Set Pivot state to hold and turn off drivetrain auto aim */
        public StopAimCommand(){
            addCommands(
                new SetPivotStateCommand(PivotState.HOLD),
                new SetDrivetrainAimStateCommand(false)
            );
        }
    }

    public static final class PrimeShootSequenceCommand extends ParallelRaceGroup {
        /**Prep the shooter and then spool up shooter - only preps it when the note clears the ground */
        public PrimeShootSequenceCommand() {
            addCommands(
                new WaitUntilCommand(() -> !Intake.noteLoaded),
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> pivot.noteClearsGround()),
                    new PrepShooterCommand(),
                    // new SpinShooterCommand()
                    new InstantCommand(() -> shooter.setShooterRPM((Pivot.getPivotState() == PivotState.AMP) ? ArmConstants.SHOOTER_RPM_AMP : ArmConstants.SHOOTER_RPM))
                )
            );
        }
    }
}
