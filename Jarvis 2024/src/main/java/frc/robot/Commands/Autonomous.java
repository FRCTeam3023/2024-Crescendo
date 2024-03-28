// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import javax.swing.GroupLayout.ParallelGroup;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Pivot;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Pivot.PivotState;
import frc.robot.Commands.CommandList.*;

/** Add your docs here. */
public class Autonomous {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    SendableChooser<Command> autoChooser;

    public Autonomous(Pivot pivot, Shooter shooter, Intake intake, Drivetrain drivetrain){
        NamedCommands.registerCommand("Intake", new IntakeCommand());
        NamedCommands.registerCommand("Intake Stop", new IntakeStopCommand());
        NamedCommands.registerCommand("Prime Shoot Sequence", new PrimeShootSequenceCommand());
        NamedCommands.registerCommand("Shoot", new SpinShooterCommand());
        NamedCommands.registerCommand("Shoot Sequence", new ShootSequenceAutoCommand());
        NamedCommands.registerCommand("Shooter Stop", new ShooterStopAutoCommand());

        NamedCommands.registerCommand("Pivot Pickup", new SetPivotStateCommand(PivotState.PICKUP));
        NamedCommands.registerCommand("Pivot Speaker", new SetPivotStateCommand(PivotState.SPEAKER));
        NamedCommands.registerCommand("Pivot Amp", new SetPivotStateCommand(PivotState.AMP));
        NamedCommands.registerCommand("Pivot Aim", new SetPivotStateCommand(PivotState.AUTOAIM));
        NamedCommands.registerCommand("Aim and Shoot", new ParallelCommandGroup(
            new FaceSpeakerStationaryCommand(),
            new ShootSequenceAutoCommand()
        ));
        NamedCommands.registerCommand("Start Aim", new FaceSpeakerMovingCommand());
        NamedCommands.registerCommand("Stop Aim", new StopAimCommand());

        NamedCommands.registerCommand("Check Intake", new WaitUntilCommand(() -> Intake.noteSensed));

        // new PathPlannerAuto("2 Note Center");
        // new PathPlannerAuto("3 Note Top");
        // new PathPlannerAuto("3 Note Bottom");
        // new PathPlannerAuto("Leave");
        // new PathPlannerAuto("Amp");
        // new PathPlannerAuto("2 Note Top");
        // new PathPlannerAuto("2 Note Bottom");
        // new PathPlannerAuto("2 Note Amp");
        // new PathPlannerAuto("Amp Hide");
        // new PathPlannerAuto("Leave Speaker");
        // new PathPlannerAuto("Inner Speaker 2 Note");
        // new PathPlannerAuto("Nothing");
        // new PathPlannerAuto("Top Speaker Outer");
        // new PathPlannerAuto("Top Speaker Long");

        // new PathPlannerAuto("2_N2");
        // new PathPlannerAuto("5_N3");
        // new PathPlannerAuto("6_N1");
        // new PathPlannerAuto("5_N3_N4");
        // new PathPlannerAuto("5_N3_N4_RS");
        // new PathPlannerAuto("Nothing");
        // new PathPlannerAuto("2_N2_N5_RS");
        // new PathPlannerAuto("5_N3_N5_RS");
        // new PathPlannerAuto("2 Note Amp");

        autoChooser = AutoBuilder.buildAutoChooser();
        autoTab.add("Auto Chooser", autoChooser).withPosition(0, 0).withSize(5, 1);
    }

    public Command getSelectedAuto(){
        return AutoBuilder.buildAuto(autoChooser.getSelected().getName());
    }
}
