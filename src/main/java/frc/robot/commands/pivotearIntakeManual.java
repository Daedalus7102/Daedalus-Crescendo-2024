// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
/* Subsistema correspondiente al comando */
import frc.robot.subsystems.Intake;

public class pivotearIntakeManual extends Command {

  /* Variables a declarar dentro del comando */
  private final Intake s_intake;
  private final double s_velocidad;
  private final SlewRateLimiter filter = new SlewRateLimiter(0.5);

  /* Constructor del comando y sus atributos */
  public pivotearIntakeManual(Intake s_intake, double s_velocidad) {
    this.s_intake = s_intake;
    this.s_velocidad = s_velocidad;
    addRequirements(s_intake);

    filter.calculate(s_velocidad);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Utiliza el método creado en el subsistema */
    s_intake.pivotear(s_velocidad);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /* Utiliza el método creado en el subsistema */
    s_intake.pivotear(0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
