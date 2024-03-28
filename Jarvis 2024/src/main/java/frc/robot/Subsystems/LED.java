// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class LED extends SubsystemBase {
  /** Creates a new LED. */

  private static SerialPort serialPort = new SerialPort(115200, Port.kMXP);
  private static COLORS currentColor = null;
  private static boolean interupted = false;

  public LED() {
    setLEDColor(0, Constants.LED_LENGTH, COLORS.OFF);
  }

  @Override
  public void periodic() {
    if (!interupted)
      setLEDColor(0, Constants.LED_LENGTH, getColorState());
  }

  //Descending priority
  public COLORS getColorState() {
    if (Pivot.climbMode) return COLORS.YELLOW;
    if (Intake.noteSensed) return COLORS.GREEN;
    return Robot.alliance == Alliance.Red ? COLORS.RED : COLORS.BLUE;
  }

  public static void interuptSignal(Command sequence) {
    interupted = true;
    sequence.andThen(new InstantCommand(() -> interupted = false));
    //sequence.schedule();
  }

  public static void setLEDColor(int start, int end, COLORS color){

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

  public enum COLORS{
    RED(255, 0, 0),
    BLUE(0, 0, 255),
    GREEN(0, 255, 0),
    ORANGE(255, 20, 0),
    YELLOW(255, 80, 0),
    PURPLE(60,0,255),
    AQUA(0,255,255),
    HOTPINK(255,0,10),
    OFF(0,0,0);

    public byte r;
    public byte g;
    public byte b;
    private COLORS(int _r, int _g, int _b) {
      r = (byte)_r;
      g = (byte)_g;
      b = (byte)_b;
    }
  }
}