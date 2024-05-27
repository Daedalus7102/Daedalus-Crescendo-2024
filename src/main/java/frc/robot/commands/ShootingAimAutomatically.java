// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive.Chassi;
import frc.robot.Constants.ConstantsShooter;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.VisionConstants;

public class ShootingAimAutomatically extends Command {

  /* Variables a declarar dentro del comando */
  private final Chassi s_chasis;
  private final Shooter s_shooter;
  private final Intake s_intake;
  private Timer shooterTimmer = new Timer();
  boolean readyForShoot = false;


  /* Constructor del comando y sus atributos */
  public ShootingAimAutomatically(Chassi s_chasis, Shooter s_shooter, Intake s_intake) {
    this.s_chasis = s_chasis;
    this.s_shooter = s_shooter;
    this.s_intake = s_intake;
    addRequirements(s_chasis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public Double calibrateX(double tagPos){
    boolean needsCalibration = tagPos <= VisionConstants.targetTXAutoAimSpeaker - VisionConstants.xThreshold || tagPos >= VisionConstants.targetTXAutoAimSpeaker + VisionConstants.xThreshold;

    if(needsCalibration){
        double error = (VisionConstants.targetTXAutoAimSpeaker) - tagPos; // Calculate error
        double speed = VisionConstants.kProt * VisionConstants.xMaxSpeed * error / VisionConstants.targetTXAutoAimSpeaker; // Adjust velocity

        if(speed > VisionConstants.xMaxSpeed){
          speed = VisionConstants.xMaxSpeed;
        }
        return speed;
    }
    return 0.0;
  }

  public Double calibrateY(double tagPos){
    boolean needsCalibration = tagPos <= VisionConstants.targetTYAutoAimSpeaker - VisionConstants.yThreshold || tagPos >= VisionConstants.targetTYAutoAimSpeaker + VisionConstants.yThreshold;

    if(needsCalibration){
        double error = VisionConstants.targetTYAutoAimSpeaker - tagPos; // Calculate error
        double speed = VisionConstants.kPdriveY * VisionConstants.yMaxSpeed * error / VisionConstants.targetTYAutoAimSpeaker; // Adjust velocity
        
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
    shooterTimmer.start();
    s_shooter.shooter(-ConstantsShooter.shooterMotorVelocity);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);

    double found = table.getEntry("tv").getDouble(0);
    if(found == 1){
        double xSpeed = calibrateX(x);
        double ySpeed = calibrateY(y);
        s_chasis.setFieldOrientedSpeed(-ySpeed, 0, xSpeed);

        if(Math.abs(x - VisionConstants.targetTXAutoAimSpeaker) <= 5
        && Math.abs(y - VisionConstants.targetTYAutoAimSpeaker) <= 1 
        && shooterTimmer.get() >= 0.4){
          s_intake.intakeRollers(IntakeConstants.intakeMotorVelocityThrowForShooter);
        }
        SmartDashboard.putBoolean("Ready for shoot", readyForShoot);
        SmartDashboard.putNumber("Required distance X", x - VisionConstants.targetTXAutoAimSpeaker);
        SmartDashboard.putNumber("Required distance Y", y - VisionConstants.targetTYAutoAimSpeaker);
    }
    else
    {
        s_chasis.setFieldOrientedSpeed(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /* Uses the method created in the subsystem */
    s_chasis.setFieldOrientedSpeed(0, 0, 0);
    s_shooter.shooter(0);
    s_intake.intakeRollers(0);
    shooterTimmer.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
