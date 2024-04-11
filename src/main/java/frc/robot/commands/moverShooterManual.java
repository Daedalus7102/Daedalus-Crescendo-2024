// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

/* Subsistema correspondiente al comando */
import frc.robot.subsystems.Shooter;

public class moverShooterManual extends Command {

  /* Variables a declarar dentro del comando */
  private final Shooter s_shooter;
  private final double shooter_velocidad;

  /* Constructor del comando y sus atributos */
  public moverShooterManual(Shooter s_shooter, double shooter_velocidad) {
    this.s_shooter = s_shooter;
    this.shooter_velocidad = shooter_velocidad;
    addRequirements(s_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Utiliza el método creado en el subsistema */
    s_shooter.shooter(shooter_velocidad);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /* Utiliza el método creado en el subsistema */
    s_shooter.shooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
