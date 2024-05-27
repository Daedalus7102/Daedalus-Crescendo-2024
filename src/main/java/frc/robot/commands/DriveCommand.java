package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrive.Chassi;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class DriveCommand extends Command{
    Chassi chassi;
    Supplier<Double> xSpeed, ySpeed, zSpeed;
    private final SlewRateLimiter xLimiter, yLimiter, zLimiter;

    public DriveCommand(Chassi chassi, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> zSpeed){
        addRequirements(chassi);
        this.chassi = chassi;

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.zSpeed = zSpeed;

        this.xLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.zLimiter = new SlewRateLimiter(SwerveDriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xNeed = this.xSpeed.get();
        double yNeed = this.ySpeed.get();
        double zNeed = this.zSpeed.get();

        // 2. Apply deadband
        xNeed = Math.abs(xNeed) > IOConstants.kDeadband ? xNeed : 0.0;
        yNeed = Math.abs(yNeed) > IOConstants.kDeadband ? yNeed : 0.0;
        zNeed = Math.abs(zNeed) > IOConstants.kDeadband ? zNeed : 0.0;

        // 3. Make the driving smoother
        xNeed = xLimiter.calculate(xNeed) * SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //revisar valor de la constante
        yNeed = yLimiter.calculate(yNeed) * SwerveDriveConstants.kTeleDriveMaxSpeedMetersPerSecond; //revisar valor de la constante
        zNeed = zLimiter.calculate(zNeed) * SwerveDriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond; //revisar valor de la constante
        
        chassi.setFieldOrientedSpeed(xNeed, yNeed, zNeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
