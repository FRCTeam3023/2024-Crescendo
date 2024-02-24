// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Shooter;

/** Add your docs here. */
public class Autonomous {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    SendableChooser<Command> autoChooser;


    Command exampleAuto; 
    Command twoNoteAuto;
    

    public Autonomous(Pivot pivot, Shooter shooter, Intake intake){
        Command intakeCommand = new RunCommand(() -> intake.setIntakeSpeed(0.65), intake);
        Command intakeStopCommand = new InstantCommand(() -> intake.setIntakeSpeed(0));
        Command prepShooterCommand = new SequentialCommandGroup(
            new ParallelRaceGroup(
                new WaitCommand(.15),
                new RunCommand(() -> intake.setIntakeSpeed(-.25))
            ),
            new InstantCommand(()->intake.setIntakeSpeed(0))
        );
        Command shootCommand = new RunCommand(() -> shooter.setShooterDutyCycle(1), shooter);
        Command shootStopCommand = new InstantCommand(() -> shooter.setShooterDutyCycle(0), shooter);

        Command shootSequenceCommand = new ParallelRaceGroup(
            new SequentialCommandGroup(
                prepShooterCommand,
                shootCommand
            ),
            new SequentialCommandGroup(
                new WaitCommand(2),
                intakeCommand
            ),
            new WaitCommand(3.5)
        );
        


        NamedCommands.registerCommand("Intake", intakeCommand);
        NamedCommands.registerCommand("Intake Stop", intakeStopCommand);
        NamedCommands.registerCommand("Prep Shooter", prepShooterCommand);
        NamedCommands.registerCommand("Shoot", shootCommand);
        NamedCommands.registerCommand("Shoot Sequence", shootSequenceCommand);
        NamedCommands.registerCommand("Shooter Stop", shootStopCommand);

        NamedCommands.registerCommand("Pivot Pickup", new RunCommand(() -> pivot.setPivotAngle(new Rotation2d(), false), pivot));
        NamedCommands.registerCommand("Pivot Speaker", new RunCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(12), false), pivot));
        NamedCommands.registerCommand("Pivot Amp", new RunCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(115), false), pivot));

        exampleAuto = new PathPlannerAuto("Test Auto");
        twoNoteAuto = new PathPlannerAuto("2 Note Center");
        autoChooser = AutoBuilder.buildAutoChooser();
        autoTab.add(autoChooser).withPosition(0, 0);
    }

    public Command getSelectedAuto(){
        return autoChooser.getSelected();
    }

}
