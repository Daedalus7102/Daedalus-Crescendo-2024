package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Constants {
    
    public static final class SwerveDriveConstants{
        //Front Left Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDfrontLeft = 4;
        public static final int turnMotorIDfrontLeft = 6;
        public static final int cancoderIDfrontLeft = 1;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 180;

        //Front Right Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDfrontRight = 8;
        public static final int turnMotorIDfrontRight = 5;
        public static final int cancoderIDfrontRight = 2;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 180;

        //Back Left Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDbackLeft = 2;
        public static final int turnMotorIDbackLeft = 7;
        public static final int cancoderIDbackLeft = 3;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = 180;

        //Back Right Module Information (Used in the "Chassi" class)
        public static final int driveMotorIDbackRight = 3;
        public static final int turnMotorIDbackRight = 1;
        public static final int cancoderIDbackRight = 4;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 180;

        //PID values [We will assume that we have to use the same value for all 4 modules] (Used in "Chassis" class)
        public static final double genericModulekP = 0.0048;
        public static final double genericModulekI = 0.0;
        public static final double genericModulekD = 0.0;

        public static final double standardTolerance = 0.03;
    
        //Information used to know the distance that the chassi has moved
        public static final double driveRevsToMeters = 4 * Math.PI / (39.37 * 8.14)  * 1.25;
        public static final double driveRPS2MPS = driveRevsToMeters;


        //Variable setting the maximum speed of the modules (Used in "Chassi" class)
        public static final double maxSpeed = 0.9;
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

        //Change these values accordinly to your needs (values for slew rate limiter)
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3.8;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.8;
    }

    public static final class IOConstants {
        public static final int PortID = 0;
        public static final int PortID_1 = 1; //Port detected by the FRC Driver Station of the control to be used
        
        /* Values ​​obtained experimentally through the FRC Driver Station */
        public static final int buttonSquare = 1;
        public static final int buttonCross = 2;
        public static final int buttonCircle = 3;
        public static final int buttonTriangle = 4;
        public static final int bumperRight = 6;
        public static final int bumoerLeft = 5;
        public static final int triggerLeft = 7;
        public static final int triggerRight = 8;
        public static final int arrowUp = 0;
        public static final int arrowRight = 90;
        public static final int arrowDown = 180;
        public static final int arrowLeft = 270;

        public static final double kDeadband = 0.05;
    }

    public static final class ClimberConstants {
        public static final DoubleSolenoid.Value rise = DoubleSolenoid.Value.kForward;
        public static final DoubleSolenoid.Value lower = DoubleSolenoid.Value.kReverse;
        public static final DoubleSolenoid.Value stop = DoubleSolenoid.Value.kOff;
    }

    public static final class IntakeConstants{
        public static final int pivotMotorID = 9;
        public static final int intakeMotorID = 10;
        public static final int pivotCANcoderID = 5;

        public static final double intakePivotMotorVelocity = 0.8;
        public static final double intakeMotorVelocitySuck = -0.6;
        public static final double intakeMotorVelocityThrow = 0.85;
        public static final double intakeMotorVelocityThrowForShooter = 0.7;

        public static final double floorGoalPosition = 18; //1
        public static final double ampGoalPosition = 128; //2
        public static final double shooterGoalPosition = 180.5; //3
        public static final double Intake_HighkP = 0.005;
        public static final double Intake_LowkP = 0.006;
        public static final double Intake_kI = 0;
        public static final double Intake_kD = 0;
        public static final double intakeOffset = -45.87890625;

        public static final double pivotMotorMaxOutput = 0.9;
    }
    public static final class ConstantsShooter {
        public static final double shooterMotorVelocity = 0.9;
    }

    public static class VisionConstants {
        public static double targetTXAutoAimSpeaker = -8.41;
        public static double targetTYAutoAimSpeaker = 1.63;

        public static final double xThreshold = 0;
        public static final double yThreshold = 0;
        public static final double xMaxSpeed = 0.8;
        public static final double yMaxSpeed = 0.8;

        public static final double kPdriveY = 0.06;
        public static final double kProt = 0.24;
    }

    public static final class ConstantesShuffleboard {
        public static final ShuffleboardTab AutonomousTab = Shuffleboard.getTab("Autonomous");
        public static final ShuffleboardTab ChasisTab = Shuffleboard.getTab("Chassi");
        public static final ShuffleboardTab IntakeTab = Shuffleboard.getTab("Intake");
    }
}
