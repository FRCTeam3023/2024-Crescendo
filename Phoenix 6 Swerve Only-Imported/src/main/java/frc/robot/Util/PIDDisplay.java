// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */

public class PIDDisplay extends SubsystemBase{

    //shuffeboard stuff

    public static final SendableChooser<PIDSetter> PIDList = new SendableChooser<>();
    public static final WPILibSetter defaultPID = new WPILibSetter(List.of(new PIDController(0, 0, 0)));

    private static ShuffleboardTab PIDTab = Shuffleboard.getTab("PID");

    private static GenericEntry PEntry = PIDTab.add("P", 0).withPosition(0, 1).getEntry();
    private static GenericEntry IEntry = PIDTab.add("I", 0).withPosition(1, 1).getEntry();
    private static GenericEntry DEntry = PIDTab.add("D", 0).withPosition(2, 1).getEntry();
    private static GenericEntry SEntry = PIDTab.add("S", 0).withPosition(3, 1).getEntry();
    private static GenericEntry VEntry = PIDTab.add("V", 0).withPosition(4, 1).getEntry();



    PIDSetter selectedPID;
    PIDSetter lastSelected;

    double PValue;
    double IValue;
    double DValue;
    double SValue;
    double VValue;

    public static void Init() {
        PIDTab.add(PIDList).withPosition(0, 0).withSize(4, 1);
        PIDList.setDefaultOption("Default PID", defaultPID);
    }

    @Override
    public void periodic(){



        //Check if the selected PID config is different. If so update the Dashboard with new numbers
        selectedPID = PIDList.getSelected();
        double[] codeCurrentPID = selectedPID.getPID();
        if(selectedPID != lastSelected){
            PEntry.setDouble(codeCurrentPID[0]);
            IEntry.setDouble(codeCurrentPID[1]);
            DEntry.setDouble(codeCurrentPID[2]);
            SEntry.setDouble(codeCurrentPID[3]);
            VEntry.setDouble(codeCurrentPID[4]);
        }
        lastSelected = selectedPID;


        
        PValue = PEntry.getDouble(codeCurrentPID[0]);
        IValue = IEntry.getDouble(codeCurrentPID[1]);
        DValue = DEntry.getDouble(codeCurrentPID[2]);
        SValue = SEntry.getDouble(codeCurrentPID[3]);
        VValue = VEntry.getDouble(codeCurrentPID[4]);


        //Check if any of the dashboard values are different than the PID values in code, write updated values
        if(PValue != codeCurrentPID[0] || IValue != codeCurrentPID[1] || DValue != codeCurrentPID[2] || SValue != codeCurrentPID[3] || VValue != codeCurrentPID[4]){
            selectedPID.setPID(PValue, IValue, DValue, SValue, VValue);
        }



    }


    public interface PIDSetter {

        public void setPID(double p, double i, double d, double s, double v);

        public double[] getPID();
        
    }

    
}
