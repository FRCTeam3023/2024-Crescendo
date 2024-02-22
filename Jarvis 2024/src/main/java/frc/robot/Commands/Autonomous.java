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

    public Autonomous(Pivot pivot, Shooter shooter, Intake intake){
        NamedCommands.registerCommand("Intake", new RunCommand(() -> intake.setIntakeSpeed(0.65), intake));
        NamedCommands.registerCommand("Intake Stop", new InstantCommand(() -> intake.setIntakeSpeed(0)));
        NamedCommands.registerCommand("Prep Shooter", new SequentialCommandGroup(
            new ParallelRaceGroup(
                new WaitCommand(.15),
                new RunCommand(() -> intake.setIntakeSpeed(-.25))
            ),
            new InstantCommand(()->intake.setIntakeSpeed(0))
        ));
        NamedCommands.registerCommand("Shoot",new RunCommand(() -> shooter.setShooterDutyCycle(1), shooter));
        NamedCommands.registerCommand("Pivot Pickup", new RunCommand(() -> pivot.setPivotAngle(new Rotation2d(), false), pivot));
        NamedCommands.registerCommand("Pivot Speaker", new RunCommand(() -> pivot.setPivotAngle(Rotation2d.fromDegrees(12), false), pivot));

        exampleAuto = new PathPlannerAuto("Test Auto");
        autoChooser = AutoBuilder.buildAutoChooser();
        autoTab.add(autoChooser).withPosition(0, 0);
    }

    public Command getSelectedAuto(){
        return autoChooser.getSelected();
    }

}
