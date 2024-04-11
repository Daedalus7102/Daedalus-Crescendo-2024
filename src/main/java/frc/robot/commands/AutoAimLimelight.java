// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ConstantesIntake;
import frc.robot.Constants.ConstantsShooter;
/* Subsistema correspondiente al comando */
import frc.robot.Constants.VisionConstants;

public class AutoAimLimelight extends Command {

  /* Variables a declarar dentro del comando */
  private final Chasis s_chasis;
  private final Shooter s_shooter;
  private final Intake s_intake;
  Supplier<Double> xSpeed;
  private Timer temporizadorShooter = new Timer();
  boolean readyForShoot = false;


  /* Constructor del comando y sus atributos */
  public AutoAimLimelight(Chasis s_chasis, Shooter s_shooter, Intake s_intake, Supplier<Double> xSpeed) {
    this.s_chasis = s_chasis;
    this.s_shooter = s_shooter;
    this.s_intake = s_intake;
    this.xSpeed = xSpeed;
    addRequirements(s_chasis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public Double calibrateX(double tagPos){
    boolean needsCalibration = tagPos <= VisionConstants.targetTX - VisionConstants.xThreshold || tagPos >= VisionConstants.targetTX + VisionConstants.xThreshold;
    // System.out.println(s_chasis.getDriveMotorOutput());
    // System.out.println(this.xSpeed.get());

    if(needsCalibration){
        // double speed = VisionConstants.xMaxSpeed * (VisionConstants.targetTX - tagPos) / VisionConstants.targetTX; 

        double xSpeed = this.xSpeed.get();

        if (xSpeed <= 0.1){
          xSpeed = 0;
        }
        double offset = (s_chasis.getDriveMotorOutput() * 0.03) * (Math.signum(xSpeed));
        // System.out.println(offset);

        if (offset <= 0.01){
          offset = 0;
        }

        double error = (VisionConstants.targetTX /*+ offset*/) - tagPos; // Calcula el error
        double speed = VisionConstants.kProt * VisionConstants.xMaxSpeed * error / VisionConstants.targetTX; // Ajusta la velocidad

        if(speed > VisionConstants.xMaxSpeed){
          speed = VisionConstants.xMaxSpeed;
        }
        return speed;
    }
    return 0.0;
  }

  public Double calibrateY(double tagPos){
    boolean needsCalibration = tagPos <= VisionConstants.targetTY - VisionConstants.yThreshold || tagPos >= VisionConstants.targetTY + VisionConstants.yThreshold;

    if(needsCalibration){
        // double speed = VisionConstants.yMaxSpeed * (VisionConstants.targetTY - tagPos) / VisionConstants.targetTY;

        double error = VisionConstants.targetTY - tagPos; // Calcula el error
        double speed = VisionConstants.kPdriveY * VisionConstants.yMaxSpeed * error / VisionConstants.targetTY; // Ajusta la velocidad
        
        if(speed > VisionConstants.yMaxSpeed){
          speed = VisionConstants.yMaxSpeed;
        }
        return speed;
    }
    return 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Utiliza el método creado en el subsistema */
    temporizadorShooter.start();
    s_shooter.shooter(-ConstantsShooter.velocidadNeoShooter);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
        
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);

    NetworkTableEntry ledMode =  table.getEntry("limelight/ledMode");
    ledMode.setNumber(2);

    System.out.println(ledMode);

    double found = table.getEntry("tv").getDouble(0);
    if(found == 1){


        double xSpeed = calibrateX(x);
        double ySpeed = calibrateY(y);
        s_chasis.setFieldOrientedSpeed(-ySpeed, 0/*-this.xSpeed.get()*/, xSpeed);

        //NO ESTA TOMANDO EN CUENTA EL TRESHOLD
        if(Math.abs(x - VisionConstants.targetTX) <= 5 && Math.abs(y - VisionConstants.targetTY) <= 0.8 && temporizadorShooter.get() >= 0.4){
          s_intake.intake(ConstantesIntake.velocidadIntakeNeoEscupirParaShooter);
        }

        SmartDashboard.putBoolean("Ready for shoot", readyForShoot);
        SmartDashboard.putNumber("Required distance X", x - VisionConstants.targetTX + ((s_chasis.getDriveMotorOutput() * 0.03) * (Math.signum(this.xSpeed.get()))));
        SmartDashboard.putNumber("Required distance Y", y - VisionConstants.targetTY);
    }
    else
    {
        s_chasis.setFieldOrientedSpeed(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /* Utiliza el método creado en el subsistema */
    s_chasis.setFieldOrientedSpeed(0, 0, 0);
    s_shooter.shooter(0);
    s_intake.intake(0);
    temporizadorShooter.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
