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
import frc.robot.Constants.ConstantesIntake;

public class Intake extends SubsystemBase {  
  private final CANSparkMax intakeMotor = new CANSparkMax(ConstantesIntake.pivotMotorID, MotorType.kBrushless);//9
  private final CANSparkMax pivotMotor = new CANSparkMax(ConstantesIntake.intakeMotorID, MotorType.kBrushless);//10
  // private final DigitalInput limitSwitchArriba = new DigitalInput(ConstantesIntake.limitSwitchArribaID);
  // private final DigitalInput limitSwitchAbajo = new DigitalInput(ConstantesIntake.limitSwitchAbajoID);

  private final DigitalInput infraredSensor =  new DigitalInput(9);
  // private final I2C.Port i2cPort = I2C.Port.kOnboard;
  // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private final CANcoder intakeCANCoder = new CANcoder(ConstantesIntake.pivotCANcoderID, "Drivetrain");
  private final PIDController pivotPID = new PIDController(ConstantesIntake.Intake_HighkP, ConstantesIntake.Intake_kI, ConstantesIntake.Intake_kD);
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
  //GenericEntry limitswitchabajoStateEntry = IntakeTab.add("Limit Switch Abajo", getLimitSwitchAbajo()).withPosition(5, 0).withSize(1, 1).getEntry();
  //GenericEntry limitswitcharribaStateEntry = IntakeTab.add("Limit Switch Arriba", getLimitSwitchArriba()).withPosition(7, 0).withSize(1, 1).getEntry();
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

  // public boolean getLimitSwitchArriba() {
  //   return limitSwitchArriba.get();
  // }

  // public boolean getLimitSwitchAbajo() {
  //   return limitSwitchAbajo.get();
  // }

  public double getIntakeEncoderPosition() {
    intakeAngle = intakeCANCoder.getAbsolutePosition().getValue()*360 + ConstantesIntake.intakeOffset; 
    if (intakeAngle >= 310 && intakeAngle <= 360){
      intakeAngle = 0;
    }
    return intakeAngle; 
  }

  public void pivotear(double velocidad) {
      pivotMotor.set(velocidad);
  }


  public double desaturatePIDValue(double s_PIDvalue) {
    if (s_PIDvalue > ConstantesIntake.pivotMotorMaxOutput) {
      s_PIDvalue = ConstantesIntake.pivotMotorMaxOutput;
    } else if (s_PIDvalue < -ConstantesIntake.pivotMotorMaxOutput) {
      s_PIDvalue = -ConstantesIntake.pivotMotorMaxOutput;
    }
    return s_PIDvalue;
  }

  public void SetIntakePosition(int position) {
    if (position == 1) {
      goal = ConstantesIntake.floorGoalPosition;
      goalPosition = "Floor";
    }
    else if (position == 2) {
      goal = ConstantesIntake.ampGoalPosition;
      goalPosition = "Amp";
    }
    else if (position == 3) {
      goal = ConstantesIntake.shooterGoalPosition;
      goalPosition = "Shooter";
    }
    else if (position == 4){
      goal = 125;
      goalPosition = "4";
      }


    if (Math.abs(goal-getIntakeEncoderPosition()) <= 65) {
      if (position==1){
        pivotPID.setP(0.008);
      }
      else if (position==2){
        pivotPID.setP(0.0005);
      }
      
      else {
      pivotPID.setP(ConstantesIntake.Intake_LowkP);
      }
    } 
    else {
    pivotPID.setP(ConstantesIntake.Intake_HighkP);
    }

    // 1
    if (position == 1) {
      pivotPID.setP(0.01);
      PIDvalue = pivotPID.calculate(getIntakeEncoderPosition(), goal);
      PIDvalue = desaturatePIDValue(PIDvalue);
      pivotMotor.set(PIDvalue);
    }

    //2
      else if (position == 2){
      pivotPID.setP(0.01);
      PIDvalue = pivotPID.calculate(getIntakeEncoderPosition(), goal);
      PIDvalue = desaturatePIDValue(PIDvalue);
      pivotMotor.set(PIDvalue);
    }

    //3
    else if (position == 3) {
        pivotPID.setP(0.01);
        PIDvalue = pivotPID.calculate(getIntakeEncoderPosition(), goal);
        PIDvalue = desaturatePIDValue(PIDvalue);
        pivotMotor.set(PIDvalue); 
    }
    
    //4
    else if (position == 4) {
        pivotPID.setP(0.015);
        PIDvalue = pivotPID.calculate(getIntakeEncoderPosition(), goal);
        PIDvalue = desaturatePIDValue(PIDvalue);
        pivotMotor.set(PIDvalue);
    }
    /*
     * 
     * 
     */
    if (getIntakeEncoderPosition() >= 11 && getIntakeEncoderPosition() <= 35 ){
      intake(ConstantesIntake.velocidadIntakeNeoChupar);
    }
    else {
      intake(0);
    }   
  }

  public void intake(double velocidad) {
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
    

    if (velocidad<-0.4 && getInfraredSensorValue() == true){
      timerForIntaking.start();

      if (timerForIntaking.get() >= 0.5){  
      intakeMotor.set(0);
      }
    }
    else{
      intakeMotor.set(velocidad);
      timerForIntaking.reset();
    }
  }
  
  @Override
  public void periodic() {
    Double pivotmotortempTemp = pivotMotor.getMotorTemperature();
    pivotmotortempEntry.setDouble(pivotmotortempTemp);
    Double noepositionencoderTemp = pivotMotor.getEncoder().getPosition();
    neopositionencoderEntry.setDouble(noepositionencoderTemp);
    Double intakeminiNeoTemp = pivotMotor.getMotorTemperature();
    intakeminineotempEntry.setDouble(intakeminiNeoTemp);
    //boolean limitswitchabajoTemp = getLimitSwitchAbajo();
    //limitswitchabajoStateEntry.setBoolean(limitswitchabajoTemp);
    //boolean limitswitcharribaTemp = getLimitSwitchArriba();
    //limitswitcharribaStateEntry.setBoolean(limitswitcharribaTemp);
    Double intakeAngleTemp = getIntakeEncoderPosition();
    intakeAngleEntry.setDouble(intakeAngleTemp);
    Double pivotMotorValueTemp = pivotMotor.getAppliedOutput();
    pivotMotorValueEntry.setDouble(pivotMotorValueTemp);
    Double PIDvalueIntakeTemp = PIDvalue;
    PIDvalueIntakeEntry.setDouble(PIDvalueIntakeTemp);
    Double kPValueTemp = pivotPID.getP();
    kPEntry.setDouble(kPValueTemp);
    Double goalPositionTemp = 0.0;
    boolean infraredSensor = getInfraredSensorValue();
    infraredSensorEntry.setBoolean(infraredSensor);

    if (goalPosition == "Floor") {
      goalPositionTemp = 1.0;
    }
    else if (goalPosition == "Amp") {
      goalPositionTemp = 2.0;
    }
    else if (goalPosition == "Shooter") {
      goalPositionTemp = 3.0;
    }
    else if(goalPosition == "4") {
      goalPositionTemp = 4.0;
    }
    goalPositionEntry.setDouble(goalPositionTemp);

  }
}