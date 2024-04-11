// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
/* Subsistema correspondiente al comando */
import frc.robot.subsystems.Shooter;

public class MoverShooterAutomatico extends Command {

  /* Variables a declarar dentro del comando */
  private final Shooter s_shooter;
  private final Intake s_intake;
  private final double shooter_velocidad;
  private final double intake_velocidad;
  private Timer temporizadorShooter = new Timer();

  /* Constructor del comando y sus atributos */
  public MoverShooterAutomatico(Shooter s_shooter, Intake s_intake, double shooter_velocidad, double intake_velocidad) {
    this.s_shooter = s_shooter;
    this.s_intake = s_intake;
    this.shooter_velocidad = shooter_velocidad;
    this.intake_velocidad = intake_velocidad;
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
    if (getTemporizador() <= 0.7){
    s_shooter.shooter(shooter_velocidad);
    }
    if (getTemporizador() >= 0.4 & getTemporizador()<=0.7){
        s_intake.intake(intake_velocidad);
    }
    if (getTemporizador() >= 0.7){
      s_intake.intake(0);
      s_shooter.shooter(0);
    }
    /* Utiliza el método creado en el subsistema */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /* Utiliza el método creado en el subsistema */
    s_shooter.shooter(0);
    s_intake.intake(0);
    temporizadorShooter.stop();
    temporizadorShooter.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
