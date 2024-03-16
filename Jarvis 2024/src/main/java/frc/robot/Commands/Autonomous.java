// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Shooter;
import frc.robot.Commands.CommandList.*;

/** Add your docs here. */
public class Autonomous {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    SendableChooser<Command> autoChooser;

    public Autonomous(Pivot pivot, Shooter shooter, Intake intake, Drivetrain drivetrain){
        // Command intakeCommand = new RunCommand(() -> intake.setIntakeSpeed(0.65), intake);
        // Command intakeStopCommand = new InstantCommand(() -> intake.setIntakeSpeed(0));
        // Command prepShooterCommand = new SequentialCommandGroup(
        //     new ParallelRaceGroup(
        //         new WaitCommand(.15),
        //         new RunCommand(() -> intake.setIntakeSpeed(-.25))
        //     ),
        //     new InstantCommand(()->intake.setIntakeSpeed(0))
        // );
        // Command shootCommand = new RunCommand(() -> shooter.setShooterDutyCycle(1), shooter);
        // Command shootStopCommand = new InstantCommand(() -> shooter.setShooterDutyCycle(0), shooter);

        // Command shootSequenceCommand = new ParallelRaceGroup(
        //     new SequentialCommandGroup(
        //         prepShooterCommand,
        //         shootCommand
        //     ),
        //     new SequentialCommandGroup(
        //         new WaitCommand(2),
        //         intakeCommand
        //     ),
        //     new WaitCommand(3.5)
        // );
      
        // public static final Command shootCommand = new InstantCommand(() -> shooter.setShooterDutyCycle(1), shooter);

        // Command shootAmpSequenceCommand = 
        //     new SequentialCommandGroup(
        //         new InstantCommand(() -> intake.setIntakeSpeed(-.25)),
        //         new WaitUntilCommand(() -> !intake.senseNote()),
        //         new InstantCommand(()->intake.setIntakeSpeed(0))
        //         // prepShooterCommand
        //         // shootCommand,
        //         // new WaitCommand(0.5),
        //         // intakeNoSensorCommand,
        //         // new WaitCommand(1),
        //         // intakeStopCommand,
        //         // shootStopCommand,
        //         // new InstantCommand(()-> Intake.noteLoaded = false)
        //     );


        


        NamedCommands.registerCommand("Intake", new IntakeCommand());
        NamedCommands.registerCommand("Intake Stop", new IntakeStopCommand());
        NamedCommands.registerCommand("Prep Shooter", new PrepShooterCommand());
        NamedCommands.registerCommand("Shoot", new ShootCommand());
        NamedCommands.registerCommand("Shoot Sequence", new ShootSequenceCommand());
        // NamedCommands.registerCommand("Amp Shoot Sequence", shootAmpSequenceCommand);
        NamedCommands.registerCommand("Shooter Stop", new ShootStopCommand());

        NamedCommands.registerCommand("Pivot Pickup", new RunCommand(() -> new SetPivotHoldCommand(new Rotation2d())));
        NamedCommands.registerCommand("Pivot Speaker", new RunCommand(() -> new SetPivotHoldCommand(Rotation2d.fromDegrees(12))));
        NamedCommands.registerCommand("Pivot Amp", new RunCommand(() -> new SetPivotHoldCommand(Rotation2d.fromDegrees(115))));
        NamedCommands.registerCommand("Aim Pivot", new AimPivot(pivot, drivetrain));

        new PathPlannerAuto("Test Auto");
        new PathPlannerAuto("2 Note Center");
        new PathPlannerAuto("3 Note Top");
        new PathPlannerAuto("3 Note Bottom");
        new PathPlannerAuto("Leave");
        new PathPlannerAuto("Amp");
        new PathPlannerAuto("2 Note Top");
        new PathPlannerAuto("2 Note Bottom");
        new PathPlannerAuto("2 Note Amp");
        new PathPlannerAuto("Amp Long");
        new PathPlannerAuto("2 Note Amp Modified");
        new PathPlannerAuto("Amp Hide");
        new PathPlannerAuto("Leave Speaker");

        autoChooser = AutoBuilder.buildAutoChooser();
        autoTab.add(autoChooser).withPosition(0, 0);
    }

    public Command getSelectedAuto(){
        return autoChooser.getSelected();
    }
}
