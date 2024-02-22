// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Util.Gains;

/** Add your docs here. */
public class SwerveModule {
    private TalonFX driveMotor;
    private CANSparkMax turnMotor;
    private double moduleOffset;
    private DigitalInput hallEffectSensor;
    public int moduleID;
    private RelativeEncoder turnEncoder;
    private SparkPIDController turnPIDController;

    private Gains turnGains = new Gains(.6, 1);

    private Gains driveGains = new Gains(5, 0, 0.15, 2.65, 12);


    // private Gains driveGains = new Gains(0 /*0.05*/,0,0,0.05,0,1);

    public boolean homeStatus = false;
    private boolean lastState;

    private VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private static List<TalonFXConfigurator> driveConfigurators = new ArrayList<>();
    private static TalonFXConfiguration talonFXConfig; //Reflects the latest constructor call
    private static List<SparkPIDController> turnPIDControllers = new ArrayList<SparkPIDController>();

     /** Shuffelboard tab to display telemetry such as heading, homing status, gyro drift, etc*/
    private static final ShuffleboardTab telemTab = Shuffleboard.getTab("Telemetry");

    private TalonFXConfiguration driveConfiguration;

    GenericEntry desiredSpeedEntry;
    GenericEntry desiredAngleEntry;
    GenericEntry actualSpeedEntry;
    GenericEntry actualAngleEntry;
    GenericEntry turnOutputEntry;
    GenericEntry driveOutputEntry;

    /**
     * A single swerve module object
     * @param moduleID Module number
     * @param driveID CAN ID number for the drive Falcon500 
     * @param turnID CAN ID number for the turning CANSparkMax
     * @param moduleOffset radian offset for the zero position in relation to the hall effect sensor
     * @param isInverted set true if the drive motor should be inverted. (Preference depending on set module offset)
     * @param sensorID DIO pin number of the hall effect sensor
     */

    public SwerveModule(int moduleID, int driveID, int turnID, double moduleOffset, InvertedValue inverted, int sensorID){

        this.moduleOffset = moduleOffset;
        this.moduleID = moduleID;

        hallEffectSensor = new DigitalInput(sensorID);

        /*--------------------------------------------------------------------------------------------*/
        //Configure Drive Motor

        driveMotor = new TalonFX(driveID);

        driveConfiguration = new TalonFXConfiguration();
        driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfiguration.MotorOutput.Inverted = inverted;
        driveConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

        driveConfiguration.Slot0.kP = driveGains.P;
        driveConfiguration.Slot0.kI = driveGains.I;
        driveConfiguration.Slot0.kD = driveGains.D;
        driveConfiguration.Slot0.kS = driveGains.S;
        driveConfiguration.Slot0.kV = driveGains.V;

        driveConfiguration.Voltage.PeakForwardVoltage = driveGains.peakOutput;
        driveConfiguration.Voltage.PeakReverseVoltage = -driveGains.peakOutput;

        driveConfiguration.Feedback.SensorToMechanismRatio = ModuleConstants.DRIVE_GEARING / Units.inchesToMeters(ModuleConstants.WHEEL_DIA * Math.PI);
        driveConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        driveConfiguration.Audio.BeepOnBoot = false;
        driveConfiguration.Audio.BeepOnConfig = false;

        driveMotor.getConfigurator().apply(driveConfiguration);

        /*-------------------------------------------------------------------------------------------*/



        //main turn motor
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

        //reset and create PID
        turnMotor.restoreFactoryDefaults();
        turnMotor.setInverted(true);
        turnPIDController = turnMotor.getPIDController();
        
        //reduce shocking turn speeds, for testing and smoother driving
        turnMotor.setClosedLoopRampRate(0.3);

        //set up encoder on the NEO
        turnEncoder = turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(2 * Math.PI * (1/ModuleConstants.TURN_GEARING)); //changes rotations to Radians
        turnEncoder.setPosition(0);
        
        turnPIDController.setPositionPIDWrappingEnabled(true);
        turnPIDController.setPositionPIDWrappingMaxInput(Math.PI);
        turnPIDController.setPositionPIDWrappingMinInput(-Math.PI);

        //set up PID gains for the turning motor
        turnPIDController.setP(turnGains.P);
        turnPIDController.setOutputRange(-turnGains.peakOutput, turnGains.peakOutput);



        turnMotor.enableVoltageCompensation(12);

        /*--------------------------------------------------------------------------------- */


        driveConfigurators.add(driveMotor.getConfigurator());
        talonFXConfig = driveConfiguration;
        turnPIDControllers.add(turnPIDController);

        velocityRequest.UpdateFreqHz = 100;








        desiredSpeedEntry = telemTab.add("Desired Speed " + moduleID, 0).withPosition(0, moduleID - 1).getEntry();
        desiredAngleEntry = telemTab.add("Desired Angle " + moduleID, 0).withPosition(3, moduleID - 1).getEntry();

        actualSpeedEntry = telemTab.add("Actual Speed " + moduleID, 0).withPosition(1, moduleID - 1).getEntry();
        actualAngleEntry = telemTab.add("Actual Angle " + moduleID, 0).withPosition(4, moduleID - 1).getEntry();

        driveOutputEntry = telemTab.add("Drive Output " + moduleID, 0).withPosition(6, moduleID - 1).getEntry();
        turnOutputEntry = telemTab.add("Turn Output " + moduleID, 0).withPosition(7, moduleID - 1).getEntry();

    }

    

    /**
     * Set the desired state for the module. Drive speeds of 0 will result in no movement of module. 
     * This prevents recentering of module after no joystick input
     * @param desiredState the desired state
     */
    public void setDesiredState(SwerveModuleState desiredState){

        desiredSpeedEntry.setDouble(desiredState.speedMetersPerSecond);
        desiredAngleEntry.setDouble(desiredState.angle.getRadians());

        //prevents returning back to foreward state when not moving
        if(Math.abs(desiredState.speedMetersPerSecond) < .001){
            stop();
            return;
        }

        //optimize the state for the current angle, prevents more than 90 degrees of motion at once
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getConstrainedAngle());

        velocityRequest.Velocity = state.speedMetersPerSecond;
        driveMotor.setControl(velocityRequest);


        //set reference angle to this new adjustment
        turnPIDController.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        desiredSpeedEntry.setDouble(velocityRequest.Velocity);
        actualSpeedEntry.setDouble(driveMotor.getVelocity().getValue());
        actualAngleEntry.setDouble(turnEncoder.getPosition());

        turnOutputEntry.setDouble(turnMotor.get());
        driveOutputEntry.setDouble(driveMotor.get());


    }


    /**
     * Homes the swerve module, to reset the encoder data
     */
    public void home(){
        //only triggers on rising edge of switch, set new home state and target location
        if(getSwitch() != lastState  && getSwitch()){
            turnEncoder.setPosition(moduleOffset);
            homeStatus = true;
        }

        //if it is triggered, set to target
        if(homeStatus){
            turnPIDController.setReference(moduleOffset, CANSparkMax.ControlType.kPosition);
        } else {
            //or keep rotating
            turnMotor.set(0.25);
        }

        lastState = getSwitch();
    }


    /**
     * @return state of the hall effect sensor
     */
    public boolean getSwitch(){
        return !hallEffectSensor.get();
    }

    /**
     * @return speed of drive motor in meters/second
     */
    public double getSpeed(){
        return driveMotor.getVelocity().getValue();
    }

    /**
     * Angle of module. This is the unbound version raw from encoder
     * @return angle of module 
     */
    public Rotation2d getAngle(){
        return new Rotation2d(turnEncoder.getPosition());
    }

    /**
     * Angle of the swerve module constrained to (-PI, PI)
     * @return
     */
    public Rotation2d getConstrainedAngle(){
        return new Rotation2d(MathUtil.angleModulus(getAngle().getRadians()));
    }

    /**
     * Set the home state. Called before homing module
     */
    public void setHomeStatus(boolean state){
        homeStatus = state;
        lastState = getSwitch();
    }

    /**
     * Direct control of motor output percent
     * @param drive drive motor speed
     * @param turn turn motor speed
     */
    public void setMotorSpeeds(double drive, double turn){
        turnMotor.set(turn);
        driveMotor.set(drive);
    }

    /**
     * @return state of module 
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(getSpeed(), getAngle());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveMotor.getPosition().getValue(), getAngle());
    }

    /**
     * stop module.
     * Sets speeds to 0
     */
    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    /**
     * Resets the encoder position for both motors
     */
    public void zeroEncoders() {
        driveMotor.setPosition(0);
        turnEncoder.setPosition(0);
    }

    
    
    /**
     * test code for manual movement of steering angle 
     * @param angle target angle in radians
     */
    public void setSteerAngle(double angle){
        turnPIDController.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Returns a list of all swerve drive motors
     */
    public static List<TalonFXConfigurator> getDriveConfigurators() {
        return driveConfigurators;
    }

    /**
     * Returns a list of all swerve turn motor PIDs
     */
    public static List<SparkPIDController> getTurnPIDControllers() {
        return turnPIDControllers;
    }

    public static TalonFXConfiguration getTalonFXConfig() {
        return talonFXConfig;
    }
}
