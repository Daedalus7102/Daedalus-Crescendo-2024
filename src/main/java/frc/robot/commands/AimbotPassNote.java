package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConstantesIntake;
import frc.robot.Constants.ConstantsShooter;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Chasis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;

public class AimbotPassNote extends Command{
  /* Variables a declarar dentro del comando */
  private final Chasis s_chasis;
  private final Shooter s_shooter;
  private final Intake s_intake;
  private final Leds s_leds;
  private Timer temporizadorShooter = new Timer();
  boolean readyForShoot = false;
  private double desiredPoseX;
  private double desiredPoseY;

  /* Constructor del comando y sus atributos */
  public AimbotPassNote(Chasis s_chasis, Shooter s_shooter, Intake s_intake, Leds s_leds) {
    this.s_chasis = s_chasis;
    this.s_shooter = s_shooter;
    this.s_intake = s_intake;
    this.s_leds = s_leds;
    addRequirements(s_chasis);
  }
  @Override
  public void initialize() {}

  public boolean getAllianceAsBoolean(){

    //Hace que si detecta la alianza azul, el booleano se aplica a true
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Blue;
    }
    return false;
  }

  public void changeDesiredPose(){
    if(getAllianceAsBoolean() == true){
        desiredPoseX = VisionConstants.targetTXAimbotPassNoteToBLUE;
        desiredPoseY = VisionConstants.targetTYAimbotPassNoteToBLUE;
    }

    else if (getAllianceAsBoolean() == false){
        desiredPoseX = VisionConstants.targetTXAimbotPassNoteToRED;
        desiredPoseY = VisionConstants.targetTYAimbotPassNoteToRED;
    }
  }

  public Double calibrateX(double tagPos){
    //if(DriverStation.getAlliance(); == )

    boolean needsCalibration = tagPos <= desiredPoseX - VisionConstants.xThreshold || tagPos >= desiredPoseX + VisionConstants.xThreshold;
    // System.out.println(s_chasis.getDriveMotorOutput());
    // System.out.println(this.xSpeed.get());

    if(needsCalibration){
        // double speed = VisionConstants.xMaxSpeed * (VisionConstants.targetTX - tagPos) / VisionConstants.targetTX; 

        double error = (desiredPoseX /*+ offset*/) - tagPos; // Calcula el error
        double speed = VisionConstants.kProt * VisionConstants.xMaxSpeed * error / desiredPoseX; // Ajusta la velocidad

        if(speed > VisionConstants.xMaxSpeed){
          speed = VisionConstants.xMaxSpeed;
        }
        return speed;
    }
    return 0.0;
  }

  public Double calibrateY(double tagPos){
    boolean needsCalibration = tagPos <= desiredPoseY - VisionConstants.yThreshold || tagPos >= desiredPoseY + VisionConstants.yThreshold;

    if(needsCalibration){
        // double speed = VisionConstants.yMaxSpeed * (VisionConstants.targetTY - tagPos) / VisionConstants.targetTY;

        double error = desiredPoseY - tagPos; // Calcula el error
        double speed = VisionConstants.kPdriveY * VisionConstants.yMaxSpeed * error / desiredPoseY; // Ajusta la velocidad
        
        if(speed > VisionConstants.yMaxSpeed){
          speed = VisionConstants.yMaxSpeed;
        }
        return speed;
    }
    return 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Utiliza el método creado en el subsistema */
    s_leds.setEffect(1, 0, true);
    
    temporizadorShooter.start();
    s_shooter.shooter(-ConstantsShooter.velocidadNeoShooter);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
        
    NetworkTableEntry ta = table.getEntry("ta");

    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);

    double found = table.getEntry("tv").getDouble(0);
    if(found == 1){

        s_leds.setEffect(3, 0, false);
        double xSpeed = calibrateX(x);
        double ySpeed = calibrateY(y);
        s_chasis.setFieldOrientedSpeed(-ySpeed, 0/*-this.xSpeed.get()*/, xSpeed);

        //NO ESTA TOMANDO EN CUENTA EL TRESHOLD
        if(Math.abs(x - desiredPoseX) <= 5 && Math.abs(y - desiredPoseY) <= 0.8 && temporizadorShooter.get() >= 0.4){
          s_intake.intake(ConstantesIntake.velocidadIntakeNeoEscupirParaShooter);
        }

        SmartDashboard.putBoolean("Ready for shoot", readyForShoot);
        SmartDashboard.putNumber("Required distance X", x - desiredPoseX);
        SmartDashboard.putNumber("Required distance Y", y - desiredPoseY);
    }
    else
    {
        s_chasis.setFieldOrientedSpeed(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /* Utiliza el método creado en el subsistema */
    s_chasis.setFieldOrientedSpeed(0, 0, 0);
    s_shooter.shooter(0);
    s_intake.intake(0);
    temporizadorShooter.reset();
    s_leds.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
