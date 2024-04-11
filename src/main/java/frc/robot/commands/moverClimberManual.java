// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class moverClimberManual extends Command {
  
  private final Climber s_climber;
  private final DoubleSolenoid.Value movimiento;

  /** Creates a new Gripper. */
  public moverClimberManual(Climber s_climber, DoubleSolenoid.Value movimiento) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_climber = s_climber;
    this.movimiento = movimiento;
    addRequirements(s_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    s_climber.moverClimber(movimiento);
  }

  @Override
  public void end(boolean interrupted) {
    s_climber.detenerClimber(DoubleSolenoid.Value.kOff);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
