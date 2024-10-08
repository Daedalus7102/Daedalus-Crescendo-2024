  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.

  package frc.robot.commands;
  import edu.wpi.first.wpilibj2.command.Command;
  import frc.robot.subsystems.Intake;

  public class PivotIntakeAutomatically extends Command {
    private final Intake s_intake;
    private final int s_position;

    public PivotIntakeAutomatically(Intake s_intake, int s_position) {
      this.s_intake = s_intake;
      this.s_position = s_position;
      addRequirements(s_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      s_intake.setIntakePosition(s_position);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      s_intake.intakePivot(0);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }
