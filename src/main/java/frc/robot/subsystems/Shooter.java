package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
  private final CANSparkMax shooterMotor1 = new CANSparkMax(11, MotorType.kBrushless);//9
  private final CANSparkMax shooterMotor2 = new CANSparkMax(12, MotorType.kBrushless);//10

  public Shooter() {
      shooterMotor1.setInverted(true);
      shooterMotor2.setInverted(true);
  }

  @Override
  public void periodic() {}

  public void shooter(double velocidad) {
    shooterMotor1.set(velocidad);
    shooterMotor2.set(velocidad);
  }

  public void shooterMotor1(double velocidad) {
    shooterMotor1.set(velocidad);
  }

  public void shooterMotor2(double velocidad) {
      shooterMotor2.set(velocidad);
  }

}