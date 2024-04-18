package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ConstantesIO;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Chasis;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class Manejo extends Command{
    Chasis chasis;
    Supplier<Double> xSpeed, ySpeed, zSpeed;
    private final SlewRateLimiter xLimiter, yLimiter, zLimiter;

    public Manejo(Chasis chasis, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> zSpeed){
        addRequirements(chasis);
        this.chasis = chasis;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.zSpeed = zSpeed;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.zLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xNeed = this.xSpeed.get();
        double yNeed = this.ySpeed.get();
        double zNeed = this.zSpeed.get();

        // 2. Apply deadband
        xNeed = Math.abs(xNeed) > ConstantesIO.kDeadband ? xNeed : 0.0;
        yNeed = Math.abs(yNeed) > ConstantesIO.kDeadband ? yNeed : 0.0;
        zNeed = Math.abs(zNeed) > ConstantesIO.kDeadband ? zNeed : 0.0;

        // 3. Make the driving smoother
        xNeed = xLimiter.calculate(xNeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //revisar valor de la constante
        yNeed = yLimiter.calculate(yNeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //revisar valor de la constante
        zNeed = zLimiter.calculate(zNeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond; //revisar valor de la constante
        
        chasis.setFieldOrientedSpeed(xNeed, yNeed, zNeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
