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
  SerialPort serialPort = new SerialPort(115200, Port.kMXP);
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
    isRed = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    iterations++;
    
    setLEDColor(0, 100, getColorState());
  }

  //Descending priority
  public colors getColorState() {
    if (Pivot.climbMode) return colors.YELLOW;
    if (Intake.noteLoaded) return isRed ? colors.GREEN : colors.ORANGE;
    return isRed ? colors.RED : colors.BLUE;
  }

  public void setLEDColor(int start, int end, colors color){
    if (color == currentColor) return;
    currentColor = color;

    //Construct
    byte[] data = new byte[7];
    data[0] = (byte)(start);
    data[1] = (byte)(start >> 8);
    data[2] = (byte)(end);
    data[3] = (byte)(end >> 8);
    data[4] = color.r;
    data[5] = color.g;
    data[6] = color.b;
    if (serialPort.write(data, data.length) < data.length) System.out.println("Failed to send (" + data.length + ") bytes over serial");
  }

  public enum colors{
    RED(255, 0, 0),
    BLUE(0, 0, 255),
    GREEN(0, 0xFF, 0x00),
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