package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ConstantesShuffleboard;

public class Chasis extends SubsystemBase {

    private final Field2d field;
        //Valores puntuales y fijos que tendrá cada módulo como el ID que tienen sus motores, su valor de PID, etc. Los datos estan almacenados en la
        //clase "Constants" y se estan accediendo a partir de la nomenclatura Constants.'nombre de variable'
        private Module frenteIzquierda = new Module(Constants.driveMotorIDfrenteIzquierda, 
                                    Constants.turnMotorIDfrenteIzquierda, 
                                    Constants.cancoderIDfrenteIzquierda, 
                                    Constants.genericModulekP, 
                                    Constants.genericModulekI, 
                                    Constants.genericModulekD, 
                                    Constants.FRONT_LEFT_MODULE_STEER_OFFSET, "Frente Izquierda");

        private Module frenteDerecha = new Module(Constants.driveMotorIDfrenteDerecha, 
                                Constants.turnMotorIDfrenteDerecha, 
                                Constants.cancoderIDfrenteDerecha, 
                                Constants.genericModulekP, 
                                Constants.genericModulekI, 
                                Constants.genericModulekD,
                                Constants.FRONT_RIGHT_MODULE_STEER_OFFSET, "Frente Derecha");

        private Module atrasIzquierda = new Module(Constants.driveMotorIDatrasIzquierda, 
                                    Constants.turnMotorIDatrasIzquierda, 
                                    Constants.cancoderIDatrasIzquierda, 
                                    Constants.genericModulekP, 
                                    Constants.genericModulekI, 
                                    Constants.genericModulekD,
                                    Constants.BACK_LEFT_MODULE_STEER_OFFSET, "Atras Izquierda");

        private Module atrasDerecha = new Module(Constants.driveMotorIDatrasDerecha, 
                                Constants.turnMotorIDatrasDerecha, 
                                Constants.cancoderIDatrasDerecha, 
                                Constants.genericModulekP, 
                                Constants.genericModulekI, 
                                Constants.genericModulekD,
                                Constants.BACK_RIGHT_MODULE_STEER_OFFSET, "Atras Derecha");
    
    
    //Valores de "x" y "y" que representan la ubicación de cada módulo en el chasís
    Translation2d frenteIzqTranslation = new Translation2d(Math.toRadians(-11.5), Math.toRadians(11.5)); //Units in Meters
    Translation2d frenteDerTranslation = new Translation2d(Math.toRadians(11.5), Math.toRadians(11.5)); //Units in Meters
    Translation2d atrasIzqTranslation = new Translation2d(Math.toRadians(-11.5), Math.toRadians(-11.5)); //Units in Meters
    Translation2d atrasDerTranslation = new Translation2d(Math.toRadians(11.5), Math.toRadians(-11.5)); //Units in Meters
    //Declarar giroscopio Pigeon 2
    final Pigeon2 gyro = new Pigeon2(0, "Drivetrain");

    public boolean isFieldOriented = true;

    public boolean changeDriveMode(){
        return !isFieldOriented;
    }



    final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frenteIzqTranslation, atrasIzqTranslation, frenteDerTranslation, atrasDerTranslation);

    SwerveModulePosition[] positions = {frenteIzquierda.getPosition(), atrasIzquierda.getPosition(), 
        frenteDerecha.getPosition(), atrasDerecha.getPosition()};

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));

    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));

    //Método que sería usado si no quisieramos que el robot tuviera orientación a cancha (esta manera no implementa el giroscopio) 
    
    public void setChassisSpeeds(double xSpeed, double ySpeed, double zSpeed){
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
    
        setModuleStates(states);
    }

    public void setFieldOrientedSpeed(double xSpeed, double ySpeed, double zSpeed){
        ChassisSpeeds chassisSpeedsFieldOriented = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation2d());    
    
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeedsFieldOriented);
        
        //Método que da un límite a todos los módulos de manera proporcional para mantener el comportamiento deseado al mover el robot
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DriveConstants.maxSpeed);

        setModuleStates(states);
    }

    public void setChassisToBreak(){
        frenteIzquierda.SwerveMotorToBreak();
        frenteDerecha.SwerveMotorToBreak();
        atrasIzquierda.SwerveMotorToBreak();
        atrasDerecha.SwerveMotorToBreak();
    }

    public void setChassisToCoast(){
        frenteIzquierda.swerveMotorsToCoast();
        frenteDerecha.swerveMotorsToCoast();
        atrasIzquierda.swerveMotorsToCoast();
        atrasDerecha.swerveMotorsToCoast();

    }

    public void setModuleStates(SwerveModuleState[] states){
        /*Importante asignar los valores del array en este orden porque así fue como se
        declaró cada estado por separado y tienen que ser asignados al indicado*/
        frenteIzquierda.setDesiredState(states[0], "Frente Izquierda");
        frenteDerecha.setDesiredState(states[1], "Frente Derecha");
        atrasIzquierda.setDesiredState(states[2], "Atras Izquierda");
        atrasDerecha.setDesiredState(states[3], "Atras Derecha");
    }

    public double getAngle(){
        //Funcion para pedirle ap Pigeon el angulo que esta midiendo
            if (isFieldOriented) {
                return (this.gyro.getAngle())%360;
            }
            else{
                return 0;
            }
        
    }

    public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(0, 0, 0);
    }

    public void runVelcAuto(ChassisSpeeds speeds){
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public ChassisSpeeds getFieldOrienteSpeeds(){
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation2d());
    }

    public Rotation2d getRotation2d(){
        return new Rotation2d(Math.toRadians(getAngle()));
    }

    public Pose2d getPose2d(){
        //return poseEstimator.getEstimatedPosition();
        return odometry.getPoseMeters();
    }

    public void setOdoPose(Pose2d pose){
        positions[0] = frenteIzquierda.getPosition();
        positions[1] = frenteDerecha.getPosition();
        positions[2] = atrasIzquierda.getPosition();
        positions[3] = atrasDerecha.getPosition();

        odometry.resetPosition(getRotation2d(), positions, pose);
        poseEstimator.resetPosition(getRotation2d(), positions, pose);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getDriveMotorOutput(){
        double averageVelocity = Math.abs((frenteIzquierda.getDriveEncoderVelocity() +
                                frenteDerecha.getDriveEncoderVelocity() +
                                atrasIzquierda.getDriveEncoderVelocity() +
                                atrasDerecha.getDriveEncoderVelocity()) / 4);

        return averageVelocity;
    }

    public Chasis(){
        AutoBuilder.configureHolonomic(
            this::getPose2d, 
            this::setOdoPose, 
            this::getChassisSpeeds, 
            this::runVelcAuto, 
            new HolonomicPathFollowerConfig(
                new PIDConstants(2.6, 0, 0), 
                new PIDConstants(2, 0, 0), 
                4.8,
                0.46,
                new ReplanningConfig()
            ), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Blue;
                }
                return false;
            },
            this);
        
        field = new Field2d();
        ConstantesShuffleboard.ChasisTab.add("Field", field).withPosition(6, 1).withSize(4, 3);
    }

    GenericEntry gyroAngleEntry = ConstantesShuffleboard.ChasisTab.add("gyro angle", getAngle()).withPosition(0, 1).withSize(2, 1).getEntry();
    GenericEntry poseXEntry = ConstantesShuffleboard.ChasisTab.add("pose x", getPose2d().getX()).withPosition(2, 1).withSize(2, 1).getEntry();
    GenericEntry poseYEntry = ConstantesShuffleboard.ChasisTab.add("pose y", getPose2d().getY()).withPosition(4, 1).withSize(2, 1).getEntry();
    GenericEntry frenteIzquierdaEntry = ConstantesShuffleboard.ChasisTab.add("frenteIzquierda angle", this.frenteIzquierda.getAngle().getDegrees()).withPosition(1, 2).withSize(2, 1).getEntry();
    GenericEntry frenteDerechaEntry = ConstantesShuffleboard.ChasisTab.add("frenteDerecha angle", this.frenteDerecha.getAngle().getDegrees()).withPosition(3, 2).withSize(2, 1).getEntry();
    GenericEntry atrasIzquierdaEntry = ConstantesShuffleboard.ChasisTab.add("atrasIzquierda angle", this.atrasIzquierda.getAngle().getDegrees()).withPosition(1, 3).withSize(2, 1).getEntry();
    GenericEntry atrasDerechaEntry = ConstantesShuffleboard.ChasisTab.add("atrasDerecha angle", this.atrasDerecha.getAngle().getDegrees()).withPosition(3, 3).withSize(2, 1).getEntry();

    public void updateShuffle(){
        Double gyroangleTemp = (this.gyro.getAngle())%360;
        poseXEntry.setDouble(gyroangleTemp);

        Double poseXTemp = getPose2d().getX();
        poseXEntry.setDouble(poseXTemp);

        Double poseYTemp = getPose2d().getY();
        poseYEntry.setDouble(poseYTemp);

        Double frenteIzquierdaTemp = this.frenteIzquierda.getAngle().getDegrees();
        frenteIzquierdaEntry.setDouble(frenteIzquierdaTemp);

        Double frenteDerechaTemp = this.frenteDerecha.getAngle().getDegrees();
        frenteDerechaEntry.setDouble(frenteDerechaTemp);

        Double atrasIzquierdaTemp = this.atrasIzquierda.getAngle().getDegrees();
        atrasIzquierdaEntry.setDouble(atrasIzquierdaTemp);

        Double atrasDerechaTemp = this.atrasDerecha.getAngle().getDegrees();
        atrasDerechaEntry.setDouble(atrasDerechaTemp);

        field.setRobotPose(odometry.getPoseMeters());

        positions[0] = frenteIzquierda.getPosition();
        positions[1] = frenteDerecha.getPosition();
        positions[2] = atrasIzquierda.getPosition();
        positions[3] = atrasDerecha.getPosition();

        odometry.update(getRotation2d(), positions);

        poseEstimator.update(getRotation2d(), positions);
    }

    @Override
    //El periodic funciona para ver cosas mínimas dentro del subsistema (Funciona aún cuando esta desabilitado)
    public void periodic() {

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        updateShuffle();

        SmartDashboard.putString("positions modules", positions.toString());
    }
}
