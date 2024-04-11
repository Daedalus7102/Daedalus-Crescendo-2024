package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConstantesIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoAMPScore extends Command {

  private final Shooter s_shooter;
  private final Intake s_intake;
  private Timer temporizadorAMPScore = new Timer();

  /* Constructor del comando y sus atributos */
  public AutoAMPScore(Shooter s_shooter, Intake s_intake) {
    this.s_shooter = s_shooter;
    this.s_intake = s_intake;
    addRequirements(s_shooter);
  }

  private double getTemporizador(){
    return temporizadorAMPScore.get();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    temporizadorAMPScore.start();

    if (getTemporizador() <= 1){
      s_shooter.shooterMotor1(-0.1);
      s_shooter.shooterMotor2(-0.8);
    }
    else{
      s_shooter.shooter(0);
    }

    if (getTemporizador() >= 0.4 & getTemporizador() <= 0.6){
      s_intake.intake(ConstantesIntake.velocidadIntakeNeoEscupirParaShooter);
    }
    else{
      s_intake.intake(0);
    }

    // if (s_intake.getIntakeEncoderPosition() <= 120 && getTemporizador() >= 3){
    //   s_intake.SetIntakePosition(3);
    // }

    /*
    if (getTemporizador()<= 2){
        s_intake.SetIntakePosition(4);
    }
    
    if(getTemporizador() >= 1.2 && getTemporizador() <= 2){
        s_intake.intake(ConstantesIntake.velocidadIntakeNeoEscupir);
    }
    else {
        s_intake.intake(0);
    }

    if (s_intake.getIntakeEncoderPosition() >= 119){
        s_intake.SetIntakePosition(3);
    }
    */

    /*
    if (goal == ConstantesIntake.ampGoalPosition){
      goal = 120;
    }
       
    PIDvalue = pivotPID.calculate(getIntakeEncoderPosition(), goal);
    PIDvalue = desaturatePIDValue(PIDvalue);
    pivotMotor.set(PIDvalue);
    // pivotPID.setP(0.02);

    if (Math.abs(goal-getIntakeEncoderPosition()) <= 15) {
        pivotPID.setP(0.4);
    }

    else {
        pivotPID.setP(0.6);
    }

    if (getIntakeEncoderPosition() <= 125){
      intakeMotor.set(ConstantesIntake.velocidadIntakeNeoEscupir);
    }

    if (getIntakeEncoderPosition() <= 120){
      intakeMotor.set(0);
      goal = ConstantesIntake.shooterGoalPosition;
      PIDvalue = pivotPID.calculate(getIntakeEncoderPosition(), goal);
      PIDvalue = desaturatePIDValue(PIDvalue);
      pivotMotor.set(PIDvalue);
    }
      if (Math.abs(goal-getIntakeEncoderPosition()) <= 15) {
        pivotPID.setP(ConstantesIntake.Intake_LowkP);
      }
      else {
        pivotPID.setP(ConstantesIntake.Intake_HighkP);
      }*/
    }

    @Override
    public void end(boolean interrupted) {
      /* Utiliza el método creado en el subsistema */
      s_shooter.shooter(0);
      temporizadorAMPScore.stop();
      temporizadorAMPScore.reset();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  
}
