// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {  
  private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.pivotMotorID, MotorType.kBrushless);//9
  private final CANSparkMax pivotMotor = new CANSparkMax(IntakeConstants.intakeMotorID, MotorType.kBrushless);//10

  private final DigitalInput infraredSensor =  new DigitalInput(9);

  private final CANcoder intakeCANCoder = new CANcoder(IntakeConstants.pivotCANcoderID, "Drivetrain");
  private final PIDController pivotPID = new PIDController(IntakeConstants.Intake_HighkP, IntakeConstants.Intake_kI, IntakeConstants.Intake_kD);
  private Timer timerForIntaking = new Timer();
  private Timer timerForLimelightBlink = new Timer();

  ShuffleboardTab IntakeTab = Shuffleboard.getTab("Intake");

  private double goal; 
  private double PIDvalue;
  private String goalPosition;
  private double intakeAngle;
  
  GenericEntry pivotmotortempEntry = IntakeTab.add("PivotNeoMotorTempertaure", pivotMotor.getMotorTemperature()).withPosition(3, 0).withSize(3, 1).getEntry();
  GenericEntry neopositionencoderEntry = IntakeTab.add("NeoPositionEncoderIntakePivot", pivotMotor.getEncoder().getPosition()).withPosition(0, 0).withSize(3, 1).getEntry();
  GenericEntry intakeminineotempEntry = IntakeTab.add("IntakeMiniNeoTemp", intakeMotor.getMotorTemperature()).withPosition(8, 0).withSize(3, 1).getEntry();
  GenericEntry intakeAngleEntry = IntakeTab.add("Intake Angle", getIntakeEncoderPosition()).withPosition(8, 1).withSize(3, 1).getEntry();
  GenericEntry pivotMotorValueEntry = IntakeTab.add("Pivot Motor Value", pivotMotor.getAppliedOutput()).withPosition(8, 2).withSize(3, 1).getEntry();
  GenericEntry PIDvalueIntakeEntry = IntakeTab.add("PID Value Pivot Value", PIDvalue).withPosition(8, 4).withSize(3, 1).getEntry();
  GenericEntry goalPositionEntry = IntakeTab.add("Position goal", 0).withPosition(8, 3).withSize(3, 1).getEntry();
  GenericEntry kPEntry = IntakeTab.add("KP value", pivotPID.getP()).withPosition(8, 5).withSize(3, 1).getEntry();
  GenericEntry infraredSensorEntry = IntakeTab.add("Note presence", getInfraredSensorValue()).withPosition(6, 0).withSize(2, 1).getEntry();

  public Intake() {
      intakeMotor.setIdleMode(IdleMode.kCoast);
      intakeMotor.setInverted(true);
      pivotMotor.setInverted(true);
  }

  public void pivotMotorCoast() {
    pivotMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void pivotMotorBreak() {
    pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public boolean getInfraredSensorValue(){
    return !infraredSensor.get();
  }

  //Security method to avoid the intake from lowering more than expected
  public double getIntakeEncoderPosition() {
    intakeAngle = intakeCANCoder.getAbsolutePosition().getValue()*360 + IntakeConstants.intakeOffset; 
    if (intakeAngle >= 310 && intakeAngle <= 360){
      intakeAngle = 0;
    }
    return intakeAngle; 
  }

  public void intakePivot(double velocidad) {
    pivotMotor.set(velocidad);
  }

  public double desaturatePIDValue(double s_PIDvalue) {
    if (s_PIDvalue > IntakeConstants.pivotMotorMaxOutput) {
      s_PIDvalue = IntakeConstants.pivotMotorMaxOutput;
    }
    else if (s_PIDvalue < -IntakeConstants.pivotMotorMaxOutput) {
      s_PIDvalue = -IntakeConstants.pivotMotorMaxOutput;
    }
    return s_PIDvalue;
  }

  public void setIntakePosition(int position) {
    switch (position) {
      case 1:
        goal = IntakeConstants.floorGoalPosition;
        goalPosition = "Floor";

        pivotPID.setP(0.005);
        PIDvalue = pivotPID.calculate(getIntakeEncoderPosition(), goal);
        PIDvalue = desaturatePIDValue(PIDvalue);
        pivotMotor.set(PIDvalue);
        break;
      case 2:
        goal = IntakeConstants.ampGoalPosition;
        goalPosition = "Amp";

        pivotPID.setP(0.005);
        PIDvalue = pivotPID.calculate(getIntakeEncoderPosition(), goal);
        PIDvalue = desaturatePIDValue(PIDvalue);
        pivotMotor.set(PIDvalue);
        break;
      case 3:
        goal = IntakeConstants.shooterGoalPosition;
        goalPosition = "Shooter";

        pivotPID.setP(0.005);
        PIDvalue = pivotPID.calculate(getIntakeEncoderPosition(), goal);
        PIDvalue = desaturatePIDValue(PIDvalue);
        pivotMotor.set(PIDvalue); 
        break;
    }

    if (Math.abs(goal-getIntakeEncoderPosition()) <= 65) {
      //Settings for security method on the PID value (When the error lowers to a determined number, the PID changes to a lower value)
      switch (position) {
        case 1:
          pivotPID.setP(0.003);
          break;
        case 2:
          pivotPID.setP(0.0003);
          break;
        case 3:
          pivotPID.setP(IntakeConstants.Intake_LowkP);
          break;
      }
    }

    //When the pivot angle encoder reads certain position, the intake rollers toggle
    if (getIntakeEncoderPosition() >= 11 && getIntakeEncoderPosition() <= 35 ){
      intakeRollers(IntakeConstants.intakeMotorVelocitySuck);
    }
    else {
      intakeRollers(0);
    }   
  }

  public void intakeRollers(double velocity) {
    //Method to make the limelight blik when the intake has a note
    if (getInfraredSensorValue() == true && getIntakeEncoderPosition() <= 40){
      timerForLimelightBlink.start();
    }
    if(timerForLimelightBlink.get() <= 1.5 && timerForLimelightBlink.get() >= 0.2){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
    }
    else if (timerForLimelightBlink.get() >= 1.5){
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
      timerForLimelightBlink.stop();
      timerForLimelightBlink.reset();
    }
    
    //Method to prevent the intake motor from being forced
    if (velocity<-0.4 && getInfraredSensorValue() == true){
      timerForIntaking.start();
    }
    if (timerForIntaking.get() >= 0.5){  
      intakeMotor.set(0);
      timerForIntaking.stop();
      timerForIntaking.reset();
    }
    else{
      intakeMotor.set(velocity);
    }
  }
  
  @Override
  public void periodic() {
    Double pivotmotortemperature = pivotMotor.getMotorTemperature();
    pivotmotortempEntry.setDouble(pivotmotortemperature);
    Double noepositionencoder = pivotMotor.getEncoder().getPosition();
    neopositionencoderEntry.setDouble(noepositionencoder);
    Double intakeminiNeo = pivotMotor.getMotorTemperature();
    intakeminineotempEntry.setDouble(intakeminiNeo);
    Double intakeAngle = getIntakeEncoderPosition();
    intakeAngleEntry.setDouble(intakeAngle);
    Double pivotMotorValue = pivotMotor.getAppliedOutput();
    pivotMotorValueEntry.setDouble(pivotMotorValue);
    Double PIDvalueIntake = PIDvalue;
    PIDvalueIntakeEntry.setDouble(PIDvalueIntake);
    Double kPValue = pivotPID.getP();
    kPEntry.setDouble(kPValue);
    boolean infraredSensor = getInfraredSensorValue();
    infraredSensorEntry.setBoolean(infraredSensor);
  }
}