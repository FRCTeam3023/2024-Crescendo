// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  AnalogOutput signalPin = new AnalogOutput(0);
  COLORS currentColor = null;
  boolean isRed = DriverStation.getAlliance().get() == Alliance.Red;

  int iterations = 0;

  public LED() {

  }

  @Override
  public void periodic() {
    if (iterations > 30) {
      System.out.println("Color: " + currentColor.name());
      iterations = 0;
    }
    iterations++;
    isRed = DriverStation.getAlliance().get() == Alliance.Red;
    
    setLEDColor(getColorState());
  }

  //Descending priority
  public COLORS getColorState() {
    if (Pivot.climbMode) return COLORS.YELLOW;
    if (Intake.noteLoaded) return isRed ? COLORS.GREEN : COLORS.ORANGE;
    return isRed ? COLORS.RED : COLORS.BLUE;
  }

  public void setLEDColor(COLORS color){
    if (color == currentColor) return;
    currentColor = color;

    signalPin.setVoltage((double)color.ordinal() / (double)COLORS.values().length * 5);
  }

  public enum COLORS{
    RED,
    BLUE,
    GREEN,
    ORANGE,
    YELLOW
  }
}