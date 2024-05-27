package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class APMScoreAutomatically extends Command {

  private final Shooter s_shooter;
  private final Intake s_intake;
  private Timer AMPScoreTimer = new Timer();

  /* Constructor del comando y sus atributos */
  public APMScoreAutomatically(Shooter s_shooter, Intake s_intake) {
    this.s_shooter = s_shooter;
    this.s_intake = s_intake;
    addRequirements(s_shooter);
  }

  private double getTemporizador(){
    return AMPScoreTimer.get();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    AMPScoreTimer.start();
    if (getTemporizador() <= 1){
      s_shooter.shooterMotor1(-0.1);
      s_shooter.shooterMotor2(-0.8);
    }
    else{
      s_shooter.shooter(0);
    }

    if (getTemporizador() >= 0.4 & getTemporizador() <= 0.6){
      s_intake.intakeRollers(IntakeConstants.intakeMotorVelocityThrowForShooter);
    }
    else{
      s_intake.intakeRollers(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    s_shooter.shooter(0);
    AMPScoreTimer.stop();
    AMPScoreTimer.reset();
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
