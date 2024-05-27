// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class MoveClimber extends Command {
  
  private final Climber s_climber;
  private final DoubleSolenoid.Value movement;

  public MoveClimber(Climber s_climber, DoubleSolenoid.Value movement) {
    this.s_climber = s_climber;
    this.movement = movement;
    addRequirements(s_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    s_climber.moveClimber(movement);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_climber.stopClimber(DoubleSolenoid.Value.kOff);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
