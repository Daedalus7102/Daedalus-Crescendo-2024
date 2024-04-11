// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* Subsistema correspondiente al comando */
import frc.robot.subsystems.Intake;

public class IntakeNoteAutomatico extends Command {

  /* Variables a declarar dentro del comando */
  private final Intake s_intake;
  /*
  private final pivotearIntakeManual s_pivot;
  private final PivotearIntakeAutomatico s_automaticIntake;
  */
  private final double s_velocidad;
  private final double s_pivotVelocidad;
  private Timer temporizadorIntakeNoteAutomatico;


  /* Constructor del comando y sus atributos */
  public IntakeNoteAutomatico(Intake s_intake/*, pivotearIntakeManual s_pivot, PivotearIntakeAutomatico s_automaticIntake*/, double s_velocidad, double s_pivotVelocidad) {
    this.s_intake = s_intake;
    /*
    this.s_pivot = s_pivot;
    this.s_automaticIntake = s_automaticIntake;
    */
    this.s_velocidad = s_velocidad;
    this.s_pivotVelocidad = s_pivotVelocidad;
    addRequirements(s_intake);
  }

  private double getTemporizador(){
    return temporizadorIntakeNoteAutomatico.get();
  }

  /*
  private void stopResetTemporizador(){
    temporizadorIntakeNoteAutomatico.stop();
    temporizadorIntakeNoteAutomatico.reset();
  }
  */

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Utiliza el método creado en el subsistema */
    s_intake.SetIntakePosition(1);
    temporizadorIntakeNoteAutomatico.start();
    if (getTemporizador() >=  2) {
      s_intake.intake(s_velocidad);
    }

    s_intake.intake(s_pivotVelocidad);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /* Utiliza el método creado en el subsistema */
    s_intake.intake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
