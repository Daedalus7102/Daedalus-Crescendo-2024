// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootNoteAutomatically extends Command {
  private final Shooter s_shooter;
  private final Intake s_intake;
  private final double shooter_velocity;
  private final double intake_velocity;
  private Timer temporizadorShooter = new Timer();

  public ShootNoteAutomatically(Shooter s_shooter, Intake s_intake, double shooter_velocity, double intake_velocity) {
    this.s_shooter = s_shooter;
    this.s_intake = s_intake;
    this.shooter_velocity = shooter_velocity;
    this.intake_velocity = intake_velocity;
    addRequirements(s_shooter);
  }

  private double getTemporizador(){
    return temporizadorShooter.get();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (getTemporizador() == 0){
      temporizadorShooter.start();
    }
    else if (getTemporizador() <= 0.7){
      s_shooter.shooter(shooter_velocity);
    }
    if (getTemporizador() >= 0.4 & getTemporizador() <= 0.6){
      s_intake.intakeRollers(intake_velocity);
    }
    else if (getTemporizador() >= 0.7){
      s_intake.intakeRollers(0);
      s_shooter.shooter(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_shooter.shooter(0);
    s_intake.intakeRollers(0);
    temporizadorShooter.stop();
    temporizadorShooter.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
