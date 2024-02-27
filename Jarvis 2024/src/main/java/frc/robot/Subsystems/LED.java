// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  AnalogOutput signalPin = new AnalogOutput(1);
  SerialPort serialPort = new SerialPort(9600, Port.kOnboard);
  colors currentColor = null;
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
  public colors getColorState() {
    if (Pivot.climbMode) return colors.YELLOW;
    if (Intake.noteLoaded) return isRed ? colors.GREEN : colors.ORANGE;
    return isRed ? colors.RED : colors.BLUE;
  }

  public void setLEDColor(colors color){
    if (color == currentColor) return;
    currentColor = color;

    serialPort.writeString(color.name());
  }

  public enum colors{
    RED,
    BLUE,
    GREEN,
    ORANGE,
    YELLOW
  }
}