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
  SerialPort serialPort = new SerialPort(9600, Port.kMXP);
  colors currentColor = null;
  boolean isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

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
    isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    
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

    //Construct
    byte[] data = new byte[3];
    data[0] = color.r;
    data[1] = color.g;
    data[2] = color.b;
    if (serialPort.write(data, data.length) < data.length) System.out.println("Failed to send (" + data.length + ") bytes over serial");
  }

  public enum colors{
    RED(0xFF, 0x00, 0x00),
    BLUE(0x00, 0x00, 0xFF),
    GREEN(0x00, 0xFF, 0x00),
    ORANGE(0xF0, 0xF0, 0x00),
    YELLOW(0xF5, 0xF5, 0xF5);

    public byte r;
    public byte g;
    public byte b;
    private colors(int _r, int _g, int _b) {
      r = (byte)_r;
      g = (byte)_g;
      b = (byte)_b;
    }
  }
}