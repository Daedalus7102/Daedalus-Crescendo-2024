package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Constants {
    
    //Información Módulo Frente Izquierda (Usado en clase "Chasis")
    public static final int driveMotorIDfrenteIzquierda = 4;
    public static final int turnMotorIDfrenteIzquierda = 6;
    public static final int cancoderIDfrenteIzquierda = 1;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 180;//178.330078125;//177.7148 + 90;

    //Información Módulo Frente Derecha (Usado en clase "Chasis")
    public static final int driveMotorIDfrenteDerecha = 8;
    public static final int turnMotorIDfrenteDerecha = 5;
    public static final int cancoderIDfrenteDerecha = 2;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 180;//174.7265625;//175.2539 + 90;

    //Información Módulo Atrás Izquierda (Usado en clase "Chasis")
    public static final int driveMotorIDatrasIzquierda = 2;
    public static final int turnMotorIDatrasIzquierda = 7;
    public static final int cancoderIDatrasIzquierda = 3;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 180;//70.48828125;//68.2031 + 90;

    //Información Módulo Atrás Derecha (Usado en clase "Chasis")
    public static final int driveMotorIDatrasDerecha = 3;
    public static final int turnMotorIDatrasDerecha = 1;
    public static final int cancoderIDatrasDerecha = 4;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 180;//171.123046875;//171.2109 + 90;

    //Valor de constante kP del PID [Asumiremos que tenemos que usar el mismo valor para los 4 módulos] (Usado en clase "Chasis")
    public static final double genericModulekP = 0.0048;
    public static final double genericModulekI = 0.0;
    public static final double genericModulekD = 0.0;

    public static final double standardTolerance = 0.03;


    public static final class DriveConstants {
            //Variable estableciendo la velocidad máxima de los módulos (Usado en clase "Chasis")
            public static final double maxSpeed = 0.9;
            public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
            public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

            //tunear segun requirimientos del robot
            public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
            public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                    kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

            /*
             * 
             * 
             * 
            */
            public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3.8;
            public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.8;
    }

    public static final class ConstantesIO {
        public static final int idPuerto = 0;
        public static final int idPuerto_1 = 1; // Puerto detectado por el FRC Driver Station del control a usar
        
        /* Valores obtenidos experimentalmente através del FRC Driver Station */
        public static final int botonCuadrado = 1;
        public static final int botonCruz = 2;
        public static final int botonCirculo = 3;
        public static final int botonTriangulo = 4;
        public static final int bumperDerecho = 6;
        public static final int bumperIzquierdo = 5;
        public static final int gatilloIzquierdo = 7;
        public static final int gatilloDerecho = 8;
        public static final int flechaArriba = 0;
        public static final int flechaDerecha = 90;
        public static final int flechaAbajo = 180;
        public static final int flechaIzquierda = 270;

        /* 
         * 
         * 
         *  
        */

        public static final int botonOptions = 9;

        public static final double kDeadband = 0.05;
    }
    
    

    public static final double driveRevsToMeters = 4 * Math.PI / (39.37 * 8.14)  * 1.25; //1.25
    public static final double driveRPS2MPS = driveRevsToMeters;

    public static final class ConstantesClimber {
    /* Valores obtenidos experimentalmente através del FRC Driver Station */
    public static final DoubleSolenoid.Value subir = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value bajar = DoubleSolenoid.Value.kReverse;
    public static final DoubleSolenoid.Value frenar = DoubleSolenoid.Value.kOff;
    }

    public static final class ConstantesIntake{
        public static final int pivotMotorID = 9;
        public static final int intakeMotorID = 10;
        public static final int pivotCANcoderID = 5;
        public static final int limitSwitchArribaID = 1;
        public static final int limitSwitchAbajoID = 0;

        public static final double velocidadIntakePivotNeo = 0.8;
        public static final double velocidadIntakeNeoChupar = -0.6;
        public static final double velocidadIntakeNeoEscupir = 0.85;
        public static final double velocidadIntakeNeoEscupirParaShooter = 0.7;

        public static final double floorGoalPosition = 18;  //12 3.5 aprox //1
        public static final double ampGoalPosition = 128;   //125  //2
        public static final double shooterGoalPosition = 180.5; //3
        public static final double Intake_HighkP = 0.009;
        public static final double Intake_LowkP = 0.004;
        public static final double Intake_kI = 0;
        public static final double Intake_kD = 0;
        public static final double intakeOffset = -45.87890625;

        public static final double pivotMotorMaxOutput = 0.9;
    }
    public static final class ConstantsShooter {
        public static final double velocidadNeoShooter = 0.9;
    }

    public static class VisionConstants {
        public static double targetTX = -8.40;//-13.22; //-9.8; //-8.5;
        public static double targetTY = 1.15;//1.15;//-9.25;

        public static final double xThreshold = 0;//2;
        public static final double yThreshold = 0;//1.2;
        public static final double xMaxSpeed = 0.8; //0.27;
        public static final double yMaxSpeed = 0.8; //0.25;

        public static final double kPdriveY = 0.06;

        public static final double kProt = 0.24;

        public static final int aprilLimePipe = 0;
        public static final int retroLimePipe = 2;

    }

    public static final class ConstantesShuffleboard {
        public static final ShuffleboardTab AutonomousTab = Shuffleboard.getTab("Autonomous");
        public static final ShuffleboardTab ChasisTab = Shuffleboard.getTab("Chasis");
        public static final ShuffleboardTab IntakeTab = Shuffleboard.getTab("Intake");
    }
}
