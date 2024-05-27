package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
  private final CANSparkMax shooterMotor1 = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax shooterMotor2 = new CANSparkMax(12, MotorType.kBrushless);

  public Shooter() {
      shooterMotor1.setInverted(true);
      shooterMotor2.setInverted(true);
  }

  @Override
  public void periodic() {}

  public void shooter(double velocity) {
    shooterMotor1.set(velocity);
    shooterMotor2.set(velocity);
  }

  public void shooterMotor1(double velocity) {
    shooterMotor1.set(velocity);
  }

  public void shooterMotor2(double velocity) {
    shooterMotor2.set(velocity);
  }

}