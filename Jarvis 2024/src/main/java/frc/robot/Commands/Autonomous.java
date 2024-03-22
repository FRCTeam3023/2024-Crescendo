// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Pivot.PivotState;
import frc.robot.Commands.CommandList.*;
import frc.robot.Constants.ArmConstants;

/** Add your docs here. */
public class Autonomous {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    SendableChooser<Command> autoChooser;

    public Autonomous(Pivot pivot, Shooter shooter, Intake intake, Drivetrain drivetrain){
        NamedCommands.registerCommand("Intake", new IntakeCommand());
        NamedCommands.registerCommand("Intake Stop", new IntakeStopCommand());
        NamedCommands.registerCommand("Prep Shooter", new PrepShooterCommand());
        NamedCommands.registerCommand("Shoot", new SpinShooterCommand());
        NamedCommands.registerCommand("Shoot Sequence", new ShootSequenceCommand());
        // NamedCommands.registerCommand("Amp Shoot Sequence", shootAmpSequenceCommand);
        NamedCommands.registerCommand("Shooter Stop", new ShooterStopCommand());

        NamedCommands.registerCommand("Pivot Pickup", new SetPivotStateCommand(PivotState.PICKUP));
        NamedCommands.registerCommand("Pivot Speaker", new SetPivotStateCommand(PivotState.SPEAKER));
        NamedCommands.registerCommand("Pivot Amp", new SetPivotStateCommand(PivotState.AMP));
        NamedCommands.registerCommand("Pivot Aim", new SetPivotStateCommand(PivotState.AUTOAIM));
        NamedCommands.registerCommand("Aim and Shoot", new SequentialCommandGroup(
            new FaceSpeakerStationaryCommand(),
            new ShootSequenceCommand()
        ));

        new PathPlannerAuto("Test Auto");
        new PathPlannerAuto("2 Note Center");
        new PathPlannerAuto("3 Note Top");
        new PathPlannerAuto("3 Note Bottom");
        new PathPlannerAuto("Leave");
        new PathPlannerAuto("Amp");
        new PathPlannerAuto("2 Note Top");
        new PathPlannerAuto("2 Note Bottom");
        new PathPlannerAuto("2 Note Amp");
        new PathPlannerAuto("Amp Hide");
        new PathPlannerAuto("Leave Speaker");
        new PathPlannerAuto("Inner Speaker 2 Note");
        new PathPlannerAuto("Nothing");

        autoChooser = AutoBuilder.buildAutoChooser();
        autoTab.add("Auto Chooser", autoChooser).withPosition(0, 0).withSize(5, 1);
    }

    public Command getSelectedAuto(){
        return AutoBuilder.buildAuto(autoChooser.getSelected().getName());
    }
}
