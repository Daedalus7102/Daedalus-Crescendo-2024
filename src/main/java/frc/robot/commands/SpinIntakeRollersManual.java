// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;

public class SpinIntakeRollersManual extends Command {

  private final Intake s_intake;
  private final double velocity;

  public SpinIntakeRollersManual(Intake s_intake, double velocity) {
    this.s_intake = s_intake;
    this.velocity = velocity;
    addRequirements(s_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_intake.intakeRollers(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_intake.intakeRollers(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
