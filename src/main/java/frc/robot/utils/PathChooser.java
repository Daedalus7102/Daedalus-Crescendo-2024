package frc.robot.utils;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;

public class PathChooser{

    //  Enum for auto modes
    private enum AutonomousMode {
        DEFAULT,
        AUTO_P2_ShooterPrecargadoS3_tomarNotaLHaciaShootS3_tomarNotaMHaciaShootS3_tomarNotaRHaciaRHaciaShootS3Incompleta,
        AUTO_P2_ShooterPrecargadoS3_tomarNotaLHaciaShootS3_tomarNotaMHaciaShootS3,
        AUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3,

        AUTO_P2_ShooterPrecargadoS3_tomarNotaRHaciaShootS3_tomarNotaMHaciaShootS3_tomarNotaLHaciaRHaciaShootS3Incompleta,
        AUTO_P2_ShooterPrecargadoS3_tomarNotaRHaciaShootS3_tomarNotaMHaciaShootS3,

        AUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3_tomarNotaRHaciaShootS3,
        AUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3_tomarNotaLHaciaShootS3,

        PATHAUTO_InicioLanzaNotaDesdeXLado,
    }

    // Trajectories object
    private final ThePaths paths;
    
    private static final SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();


    // Add options to chooser
    public PathChooser(ThePaths paths) {
        this.paths = paths;
        autonomousModeChooser.setDefaultOption(
                "Default (No code)", AutonomousMode.DEFAULT);

        autonomousModeChooser.addOption("-AUTO P2_ShooterPrecargadoS3_tomarNotaLHaciaShootS3_tomarNotaMHaciaShootS3_tomarNotaRHaciaRHaciaShootS3Incompleta", AutonomousMode.AUTO_P2_ShooterPrecargadoS3_tomarNotaLHaciaShootS3_tomarNotaMHaciaShootS3_tomarNotaRHaciaRHaciaShootS3Incompleta);
        autonomousModeChooser.addOption("AUTO_P2_ShooterPrecargadoS3_tomarNotaLHaciaShootS3_tomarNotaMHaciaShootS3 LEFT PRIMERO", AutonomousMode.AUTO_P2_ShooterPrecargadoS3_tomarNotaLHaciaShootS3_tomarNotaMHaciaShootS3);
        autonomousModeChooser.addOption("AUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3", AutonomousMode.AUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3);

        autonomousModeChooser.addOption("AUTO_P2_ShooterPrecargadoS3_tomarNotaRHaciaShootS3_tomarNotaMHaciaShootS3_tomarNotaLHaciaRHaciaShootS3Incompleta", AutonomousMode.AUTO_P2_ShooterPrecargadoS3_tomarNotaRHaciaShootS3_tomarNotaMHaciaShootS3_tomarNotaLHaciaRHaciaShootS3Incompleta);
        autonomousModeChooser.addOption("AUTO_P2_ShooterPrecargadoS3_tomarNotaRHaciaShootS3_tomarNotaMHaciaShootS3 RIGHT PRIMERO", AutonomousMode.AUTO_P2_ShooterPrecargadoS3_tomarNotaRHaciaShootS3_tomarNotaMHaciaShootS3);

        //MEDIO PRIMERO Y LUEGO UN LADO
        autonomousModeChooser.addOption("-AUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3_tomarNotaRHaciaShootS3 MEDIO PRIMERO", AutonomousMode.AUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3_tomarNotaRHaciaShootS3);
        autonomousModeChooser.addOption("AUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3_tomarNotaLHaciaShootS3 MEDIO PRIMERO", AutonomousMode.AUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3_tomarNotaLHaciaShootS3);

        autonomousModeChooser.addOption("PATHAUTO_InicioLanzaNotaDesdeXLado", AutonomousMode.PATHAUTO_InicioLanzaNotaDesdeXLado);
    }

    public static SendableChooser<AutonomousMode> getModeChooser() {
        return autonomousModeChooser;
    }

    public Command autoPruebas(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        command = null;
        return command;
    }

    // MAX
    public Command getAUTO_P2_ShooterPrecargadoS3_tomarNotaLHaciaShootS3_tomarNotaMHaciaShootS3_tomarNotaRHaciaRHaciaShootS3Incompleta(RobotContainer container){
        PathPlannerPath path = paths.getInicioEnP2_ShooterPrecargadoS3_TomarNotaL();
        return resetDrivetrainPose(container, path)
        .andThen(AutoBuilder.followPath(path))
        .andThen(AutoBuilder.followPath(paths.getContinuacionRegreso_desdeL_haciaShootS3()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionIda_desdeS3_haciaTomarNotaM()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionRegreso_desdeM_haciaShootS3()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionIda_desdeS3_haciaTomarNotaR()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionRegreso_desdeR_haciaShootS3()));
    }
    //NOTA LEFT PRIMERO Y LUEGO MEDIO
    public Command getAUTO_P2_ShooterPrecargadoS3_tomarNotaLHaciaShootS3_tomarNotaMHaciaShootS3(RobotContainer container){
        PathPlannerPath path = paths.getInicioEnP2_ShooterPrecargadoS3_TomarNotaL();
        return resetDrivetrainPose(container, path)
        .andThen(AutoBuilder.followPath(path))
        .andThen(AutoBuilder.followPath(paths.getContinuacionRegreso_desdeL_haciaShootS3()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionIda_desdeS3_haciaTomarNotaM()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionRegreso_desdeM_haciaShootS3()));
    }
    //SOLO SHOOT NOTA PRECARGADA Y MEDIO
    public Command getAUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3(RobotContainer container){
        PathPlannerPath path = paths.getInicioEnP2_shooterPrecargadoS3_tomarNotaM();
        return resetDrivetrainPose(container, path)
        .andThen(AutoBuilder.followPath(path))
        .andThen(AutoBuilder.followPath(paths.getContinuacionRegreso_desdeM_haciaShootS3()));
    }




    // MAX
    public Command getAUTO_P2_ShooterPrecargadoS3_tomarNotaRHaciaShootS3_tomarNotaMHaciaShootS3_tomarNotaLHaciaRHaciaShootS3Incompleta(RobotContainer container){
        PathPlannerPath path = paths.getInicioEnP2_shooterPrecargadoS3_tomarNotaR();
        return resetDrivetrainPose(container, path)
        .andThen(AutoBuilder.followPath(path))
        .andThen(AutoBuilder.followPath(paths.getContinuacionRegreso_desdeR_haciaShootS3()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionIda_desdeS3_haciaTomarNotaM()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionRegreso_desdeM_haciaShootS3()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionIda_desdeS3_haciaTomarNotaL()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionRegreso_desdeL_haciaShootS3()));
    }

    //NOTA RIGHT PRIMERO Y LUEGO MEDIO
    public Command getAUTO_P2_ShooterPrecargadoS3_tomarNotaRHaciaShootS3_tomarNotaMHaciaShootS3(RobotContainer container){
        PathPlannerPath path = paths.getInicioEnP2_shooterPrecargadoS3_tomarNotaR();
        return resetDrivetrainPose(container, path)
        .andThen(AutoBuilder.followPath(path))
        .andThen(AutoBuilder.followPath(paths.getContinuacionRegreso_desdeR_haciaShootS3()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionIda_desdeS3_haciaTomarNotaM()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionRegreso_desdeM_haciaShootS3()));
    }
    




    //MEDIO PRIMERO Y LUEGO UN LADO
    public Command getAUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3_tomarNotaRHaciaShootS3(RobotContainer container){
        PathPlannerPath path = paths.getInicioEnP2_shooterPrecargadoS3_tomarNotaM();
        return resetDrivetrainPose(container, path)
        .andThen(AutoBuilder.followPath(path))
        .andThen(AutoBuilder.followPath(paths.getContinuacionRegreso_desdeM_haciaShootS3()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionIda_desdeS3_haciaTomarNotaR()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionRegreso_desdeR_haciaShootS3()));
    }
    //MEDIO PRIMERO Y LUEGO UN LADO
    public Command getAUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3_tomarNotaLHaciaShootS3(RobotContainer container){
        PathPlannerPath path = paths.getInicioEnP2_shooterPrecargadoS3_tomarNotaM();
        return resetDrivetrainPose(container, path)
        .andThen(AutoBuilder.followPath(path))
        .andThen(AutoBuilder.followPath(paths.getContinuacionRegreso_desdeM_haciaShootS3()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionIda_desdeS3_haciaTomarNotaL()))
        .andThen(AutoBuilder.followPath(paths.getContinuacionRegreso_desdeL_haciaShootS3()));
    }



    public Command getPATHAUTO_InicioLanzaNotaDesdeXLado(RobotContainer container){
        PathPlannerPath path = paths.getInicioLanzaNotaDesdeXLado();
        return resetDrivetrainPose(container, path);
    }



    public Command getCommand(RobotContainer container){
        switch (autonomousModeChooser.getSelected()) {
            case DEFAULT:
                return autoPruebas(container);
            // case inicioDeLado_ShootPrecargadaS1oS3:
            //     return getInicioDeLado_ShootPrecargadaS1oS3(container);

            case AUTO_P2_ShooterPrecargadoS3_tomarNotaLHaciaShootS3_tomarNotaMHaciaShootS3_tomarNotaRHaciaRHaciaShootS3Incompleta:
                return getAUTO_P2_ShooterPrecargadoS3_tomarNotaLHaciaShootS3_tomarNotaMHaciaShootS3_tomarNotaRHaciaRHaciaShootS3Incompleta(container);
            case AUTO_P2_ShooterPrecargadoS3_tomarNotaLHaciaShootS3_tomarNotaMHaciaShootS3:
                return getAUTO_P2_ShooterPrecargadoS3_tomarNotaLHaciaShootS3_tomarNotaMHaciaShootS3(container);
            case AUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3:
                return getAUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3(container);


            case AUTO_P2_ShooterPrecargadoS3_tomarNotaRHaciaShootS3_tomarNotaMHaciaShootS3_tomarNotaLHaciaRHaciaShootS3Incompleta:
                return getAUTO_P2_ShooterPrecargadoS3_tomarNotaRHaciaShootS3_tomarNotaMHaciaShootS3_tomarNotaLHaciaRHaciaShootS3Incompleta(container);
            case AUTO_P2_ShooterPrecargadoS3_tomarNotaRHaciaShootS3_tomarNotaMHaciaShootS3:
                return getAUTO_P2_ShooterPrecargadoS3_tomarNotaRHaciaShootS3_tomarNotaMHaciaShootS3(container);

            //MEDIO PRIMERO Y LUEGO UN LADO
            case AUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3_tomarNotaRHaciaShootS3:
                return getAUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3_tomarNotaRHaciaShootS3(container);
            case AUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3_tomarNotaLHaciaShootS3:
                return getAUTO_P2_ShooterPrecargadoS3_tomarNotaMHaciaShootS3_tomarNotaLHaciaShootS3(container);
            default:
                return null;
        }
    }

    public Command resetDrivetrainPose(RobotContainer container, PathPlannerPath path) {
        Optional<Alliance> ally = DriverStation.getAlliance();
        final Pose2d initialState = path.getStartingDifferentialPose();
        /*
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Blue) {
                initialState = initialState;
            } else {
                //path.flipPath();
                initialState = initialState;
            }
        } else {
            initialState = new Pose2d(0, 0, container.getChasisSubsystem().getRotation2d());
        }
         */
        return new InstantCommand(() -> resetPose(initialState, container));
    }

    public void resetPose(Pose2d pose, RobotContainer container) {
        container.getChasisSubsystem().setOdoPose(pose);
    }
}
