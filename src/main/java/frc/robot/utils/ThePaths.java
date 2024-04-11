package frc.robot.utils;

import com.pathplanner.lib.path.PathPlannerPath;

public class ThePaths{

    // private PathPlannerPath shortLine;
    // private PathPlannerPath autoSeccionado;
    // private PathPlannerPath autoSeccionado2;
    // private PathPlannerPath notaIzqIdaParaInicioMedio;
    // private PathPlannerPath noatMedioRegresoParaTirarMedio;
    // private PathPlannerPath notaMedioIdaParaInicioMedio;
    // private PathPlannerPath inicioDeLado_ShootPrecargadaS1oS3;
    private PathPlannerPath getInicioEnP2_ShooterPrecargadoS3_TomarNotaL;
    private PathPlannerPath getContinuacionRegreso_desdeL_haciaShootS3;
    private PathPlannerPath getContinuacionIda_desdeS3_haciaTomarNotaM;
    private PathPlannerPath getContinuacionRegreso_desdeM_haciaShootS3;
    private PathPlannerPath getContinuacionIda_desdeS3_haciaTomarNotaR;
    private PathPlannerPath getContinuacionRegreso_desdeR_haciaShootS3;

    private PathPlannerPath getInicioEnP2_shooterPrecargadoS3_tomarNotaM;
    private PathPlannerPath getInicioEnP2_shooterPrecargadoS3_tomarNotaR;
    private PathPlannerPath getContinuacionIda_desdeS3_haciaTomarNotaL;

    private PathPlannerPath getInicioLanzaNotaDesdeXLado;

    public ThePaths() {
        //inicioDeLado_ShootPrecargadaS1oS3 = PathPlannerPath.fromPathFile("InicioDeLado+ShootPrecargada S1 o S3");
        getInicioEnP2_ShooterPrecargadoS3_TomarNotaL = PathPlannerPath.fromPathFile("inicioEnP2_ShooterPrecargadoS3_TomarNotaL");
        getContinuacionRegreso_desdeL_haciaShootS3 = PathPlannerPath.fromPathFile("continuacionRegreso_desdeL_haciaShootS3");
        getContinuacionIda_desdeS3_haciaTomarNotaM = PathPlannerPath.fromPathFile("continuacionIda_desdeS3_haciaTomarNotaM");
        getContinuacionRegreso_desdeM_haciaShootS3 = PathPlannerPath.fromPathFile("continuacionRegreso_desdeM_haciaShootS3");
        getContinuacionIda_desdeS3_haciaTomarNotaR =  PathPlannerPath.fromPathFile("continuacionIda_desdeS3_haciaTomarNotaR");
        getContinuacionRegreso_desdeR_haciaShootS3 = PathPlannerPath.fromPathFile("continuacionRegreso_desdeR_haciaShootS3");

        getInicioEnP2_shooterPrecargadoS3_tomarNotaM = PathPlannerPath.fromPathFile("inicioEnP2_shooterPrecargadoS3_tomarNotaM");
        getInicioEnP2_shooterPrecargadoS3_tomarNotaR = PathPlannerPath.fromPathFile("inicioEnP2_shooterPrecargadoS3_tomarNotaR");
        getContinuacionIda_desdeS3_haciaTomarNotaL = PathPlannerPath.fromPathFile("continuacionIda_desdeS3_haciaTomarNotaL");

        getInicioLanzaNotaDesdeXLado = PathPlannerPath.fromPathFile("InicioLanzaNotaDesdeXLado");
    }

    // public PathPlannerPath getDriveShortLine() {
    //     return shortLine;
    // }

    // public PathPlannerPath getAutonomoSeccionado() {
    //     return autoSeccionado;
    // }


    // public PathPlannerPath getAutonomoSeccionado2(){
    //     return autoSeccionado2;
    // }

    // public PathPlannerPath getNotaIzqIdaParaInicioMedio(){
    //     return notaIzqIdaParaInicioMedio;
    // }

    // public PathPlannerPath getNoatMedioRegresoParaTirarMedio() {
    //     return noatMedioRegresoParaTirarMedio;
    // }

    // public PathPlannerPath getNotaMedioIdaParaInicioMedio() {
    //     return notaMedioIdaParaInicioMedio;
    // }

    // public PathPlannerPath getInicioDeLado_ShootPrecargadaS1oS3(){
    //     return inicioDeLado_ShootPrecargadaS1oS3;
    // }

    public PathPlannerPath getInicioEnP2_ShooterPrecargadoS3_TomarNotaL(){
        return getInicioEnP2_ShooterPrecargadoS3_TomarNotaL;
    }

    public PathPlannerPath getContinuacionRegreso_desdeL_haciaShootS3(){
        return getContinuacionRegreso_desdeL_haciaShootS3;
    }

    public PathPlannerPath getContinuacionIda_desdeS3_haciaTomarNotaM(){
        return getContinuacionIda_desdeS3_haciaTomarNotaM;
    }

    public PathPlannerPath getContinuacionRegreso_desdeM_haciaShootS3(){
        return getContinuacionRegreso_desdeM_haciaShootS3;
    }

    public PathPlannerPath getContinuacionIda_desdeS3_haciaTomarNotaR(){
        return getContinuacionIda_desdeS3_haciaTomarNotaR;
    }

    public PathPlannerPath getContinuacionRegreso_desdeR_haciaShootS3(){
        return getContinuacionRegreso_desdeR_haciaShootS3;
    }




    public PathPlannerPath getInicioEnP2_shooterPrecargadoS3_tomarNotaM(){
        return getInicioEnP2_shooterPrecargadoS3_tomarNotaM;
    }

    public PathPlannerPath getInicioEnP2_shooterPrecargadoS3_tomarNotaR(){
        return getInicioEnP2_shooterPrecargadoS3_tomarNotaR;
    }

    public PathPlannerPath getContinuacionIda_desdeS3_haciaTomarNotaL(){
        return getContinuacionIda_desdeS3_haciaTomarNotaL;
    }



    public PathPlannerPath getInicioLanzaNotaDesdeXLado(){
        return getInicioLanzaNotaDesdeXLado;
    }

    /*
    public PathPlannerPath getContinuacionRegreso_DesdeR_HaciaShootS3(){
        return continuacionRegreso_DesdeR_HaciaShootS3;
    }
    */
}