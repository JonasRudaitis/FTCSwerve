package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

public class TurnCommand extends CommandBase {

    private final SwerveSubsystem drive;
    private final double targetDegrees; // relative turn: + = CCW, - = CW
    private Rotation2d startHeading;

    // Dynamic P parameters
    private final double minKp = 3.0;   // gentle at start/end
    private final double maxKp = 16.0;  // fast in the middle
    private final double rampZone = Math.toRadians(20); // radians within which kP ramps down

    public TurnCommand(SwerveSubsystem drive, double angleDegrees) {
        this.drive = drive;
        this.targetDegrees = angleDegrees;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        startHeading = drive.getHeading();
    }

    @Override
    public void execute() {
        // Calculate desired heading
        Rotation2d desired = startHeading.plus(Rotation2d.fromDegrees(targetDegrees));
        Rotation2d current = drive.getHeading();

        // Error in radians
        double error = desired.minus(current).getRadians();
        double absError = Math.abs(error);

        // Dynamic kP: ramp down near target
        double kP;
        if (absError > rampZone) {
            kP = maxKp;
        } else {
            kP = minKp + (maxKp - minKp) * (absError / rampZone);
        }

        // Compute omega
        double omega = error * kP;

        // Clamp omega to reasonable max speed
        double maxOmega = Math.PI; // rad/s
        omega = Math.max(-maxOmega, Math.min(maxOmega, omega));

        // Drive in place
        drive.drive(new ChassisSpeeds(0, 0, omega));
    }

    @Override
    public boolean isFinished() {
        Rotation2d desired = startHeading.plus(Rotation2d.fromDegrees(targetDegrees));
        Rotation2d current = drive.getHeading();
        double errorDeg = Math.toDegrees(desired.minus(current).getRadians());
        return Math.abs(errorDeg) < 2.5; // tighter tolerance
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds(0, 0, 0)); // stop rotation
    }
}
