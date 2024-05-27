package frc.robot.subsystems.SwerveDrive;

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
import frc.robot.Constants;
import frc.robot.Constants.ConstantesShuffleboard;
import frc.robot.Constants.SwerveDriveConstants;

public class Chassi extends SubsystemBase {

    private final Field2d field;
        //Specific and fixed values ​​that each module will have, such as the ID of its motors, its PID value, etc. The data is stored in the
        //class "Constants" and are being accessed from the nomenclature Constants.'variable name'
        private Module frontLeft = new Module(SwerveDriveConstants.driveMotorIDfrontLeft, 
                                    SwerveDriveConstants.turnMotorIDfrontLeft, 
                                    SwerveDriveConstants.cancoderIDfrontLeft, 
                                    SwerveDriveConstants.genericModulekP, 
                                    SwerveDriveConstants.genericModulekI, 
                                    SwerveDriveConstants.genericModulekD, 
                                    SwerveDriveConstants.FRONT_LEFT_MODULE_STEER_OFFSET, "Front Left");

        private Module frontRight = new Module(SwerveDriveConstants.driveMotorIDfrontRight, 
                                    SwerveDriveConstants.turnMotorIDfrontRight, 
                                    SwerveDriveConstants.cancoderIDfrontRight, 
                                    SwerveDriveConstants.genericModulekP, 
                                    SwerveDriveConstants.genericModulekI, 
                                    SwerveDriveConstants.genericModulekD,
                                    SwerveDriveConstants.FRONT_RIGHT_MODULE_STEER_OFFSET, "Front Right");

        private Module backLeft = new Module(SwerveDriveConstants.driveMotorIDbackLeft, 
                                    SwerveDriveConstants.turnMotorIDbackLeft, 
                                    SwerveDriveConstants.cancoderIDbackLeft, 
                                    SwerveDriveConstants.genericModulekP, 
                                    SwerveDriveConstants.genericModulekI, 
                                    SwerveDriveConstants.genericModulekD,
                                    SwerveDriveConstants.BACK_LEFT_MODULE_STEER_OFFSET, "Back Left");

        private Module backRight = new Module(SwerveDriveConstants.driveMotorIDbackRight, 
                                    SwerveDriveConstants.turnMotorIDbackRight, 
                                    SwerveDriveConstants.cancoderIDbackRight, 
                                    SwerveDriveConstants.genericModulekP, 
                                    SwerveDriveConstants.genericModulekI, 
                                    SwerveDriveConstants.genericModulekD,
                                    SwerveDriveConstants.BACK_RIGHT_MODULE_STEER_OFFSET, "Back Right");
    
    
    //"x" and "y" values ​​that represent the location of each module in the chassis
    Translation2d frontLeftTranslation = new Translation2d(Math.toRadians(-11.5), Math.toRadians(11.5)); //Units in Meters
    Translation2d frontRightTranslation = new Translation2d(Math.toRadians(11.5), Math.toRadians(11.5)); //Units in Meters
    Translation2d backLeftTranslation = new Translation2d(Math.toRadians(-11.5), Math.toRadians(-11.5)); //Units in Meters
    Translation2d backRightTranslation = new Translation2d(Math.toRadians(11.5), Math.toRadians(-11.5)); //Units in Meters
    //Declare Gyroscope Pigeon 2
    final Pigeon2 gyro = new Pigeon2(0, "Drivetrain");

    public boolean isFieldOriented = true;

    public boolean changeDriveMode(){
        return !isFieldOriented;
    }



    final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftTranslation, backLeftTranslation, frontRightTranslation, backRightTranslation);

    SwerveModulePosition[] positions = {frontLeft.getPosition(), backLeft.getPosition(), 
        frontRight.getPosition(), backRight.getPosition()};

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));

    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));

    //Method that would be used if we did not want the robot to have court orientation (this way does not implement the gyroscope)
    public void setChassisSpeeds(double xSpeed, double ySpeed, double zSpeed){
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
    
        setModuleStates(states);
    }

    public void setFieldOrientedSpeed(double xSpeed, double ySpeed, double zSpeed){
        ChassisSpeeds chassisSpeedsFieldOriented = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation2d());    
    
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeedsFieldOriented);
        
        //Method that limits all modules proportionally to maintain the desired behavior when moving the robot
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDriveConstants.maxSpeed);

        setModuleStates(states);
    }

    public void setChassisToBreak(){
        frontLeft.SwerveMotorToBreak();
        frontRight.SwerveMotorToBreak();
        backLeft.SwerveMotorToBreak();
        backRight.SwerveMotorToBreak();
    }

    public void setChassisToCoast(){
        frontLeft.swerveMotorsToCoast();
        frontRight.swerveMotorsToCoast();
        backLeft.swerveMotorsToCoast();
        backRight.swerveMotorsToCoast();

    }

    public void setModuleStates(SwerveModuleState[] states){
        /*It is important to assign the values ​​of the array in this order because that is how it was
        declared each state separately and have to be assigned to the indicated one*/
        frontLeft.setDesiredState(states[0], "Front Left");
        frontRight.setDesiredState(states[1], "Front Right");
        backLeft.setDesiredState(states[2], "Back Left");
        backRight.setDesiredState(states[3], "Back Right");
    }

    public double getAngle(){
        //Function to ask Pigeon the angle it is measuring
        return (this.gyro.getAngle())%360;
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
        return odometry.getPoseMeters();
    }

    public void setOdoPose(Pose2d pose){
        positions[0] = frontLeft.getPosition();
        positions[1] = frontRight.getPosition();
        positions[2] = backLeft.getPosition();
        positions[3] = backRight.getPosition();

        odometry.resetPosition(getRotation2d(), positions, pose);
        poseEstimator.resetPosition(getRotation2d(), positions, pose);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public Chassi(){
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
    GenericEntry frontLeftEntry = ConstantesShuffleboard.ChasisTab.add("frontLeft angle", this.frontLeft.getAngle().getDegrees()).withPosition(1, 2).withSize(2, 1).getEntry();
    GenericEntry frontRightEntry = ConstantesShuffleboard.ChasisTab.add("frontRight angle", this.frontRight.getAngle().getDegrees()).withPosition(3, 2).withSize(2, 1).getEntry();
    GenericEntry backLeftEntry = ConstantesShuffleboard.ChasisTab.add("backLeft angle", this.backLeft.getAngle().getDegrees()).withPosition(1, 3).withSize(2, 1).getEntry();
    GenericEntry backRightEntry = ConstantesShuffleboard.ChasisTab.add("backRight angle", this.backRight.getAngle().getDegrees()).withPosition(3, 3).withSize(2, 1).getEntry();

    public void updateShuffle(){
        Double gyroangleTemp = (this.gyro.getAngle())%360;
        poseXEntry.setDouble(gyroangleTemp);

        Double poseXTemp = getPose2d().getX();
        poseXEntry.setDouble(poseXTemp);

        Double poseYTemp = getPose2d().getY();
        poseYEntry.setDouble(poseYTemp);

        Double frontLeftTemp = this.frontLeft.getAngle().getDegrees();
        frontLeftEntry.setDouble(frontLeftTemp);

        Double frontRightTemp = this.frontRight.getAngle().getDegrees();
        frontRightEntry.setDouble(frontRightTemp);

        Double backLeftTemp = this.backLeft.getAngle().getDegrees();
        backLeftEntry.setDouble(backLeftTemp);

        Double backRightTemp = this.backRight.getAngle().getDegrees();
        backRightEntry.setDouble(backRightTemp);

        field.setRobotPose(odometry.getPoseMeters());

        positions[0] = frontLeft.getPosition();
        positions[1] = frontRight.getPosition();
        positions[2] = backLeft.getPosition();
        positions[3] = backRight.getPosition();

        odometry.update(getRotation2d(), positions);

        poseEstimator.update(getRotation2d(), positions);
    }

    @Override
    //The periodic works to see minimal things within the subsystem (It works even when it is disabled)
    public void periodic() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        updateShuffle();
        SmartDashboard.putString("positions modules", positions.toString());
    }
}
