// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AnalogOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  AnalogOutput signalPin = new AnalogOutput(1);
  public LED() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void setLEDColor(colors color){

  }
}

enum colors{
  RED,
  BLUE,
  GREEN
}
