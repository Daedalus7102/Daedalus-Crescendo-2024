// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
/* Subsistema correspondiente al comando */
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PivotearIntakeAutomatico extends Command {

  /* Variables a declarar dentro del comando */
  private final Intake s_intake;
  private final Shooter s_shooter;
  private final int s_position;

  /* Constructor del comando y sus atributos */
  public PivotearIntakeAutomatico(Intake s_intake, Shooter s_shooter, int s_position) {
    this.s_intake = s_intake;
    this.s_shooter = s_shooter;
    this.s_position = s_position;
    addRequirements(s_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Utiliza el método creado en el subsistema */
    s_intake.SetIntakePosition(s_position);
    if (s_position == 3 && s_intake.getIntakeEncoderPosition() >= 160) {
      //s_shooter.shooter(-ConstantsShooter.velocidadNeoShooter);
    } else {
      s_shooter.shooter(0);
    }
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
