package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

public class StrafeCommand extends CommandBase {

    private final SwerveSubsystem drive;
    private final double distance; // meters (+ = left, - = right)
    private Pose2d startPose;
    private final double kHeading = 4.0; // P gain for heading correction
    private final double speed;         // max strafing speed (m/s)

    public StrafeCommand(SwerveSubsystem drive, double distanceMeters, double maxSpeed) {
        this.drive = drive;
        this.distance = distanceMeters;
        this.speed = Math.abs(maxSpeed);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        startPose = drive.getPose(); // remember starting pose
    }

    @Override
    public void execute() {
        Pose2d current = drive.getPose();

        // Calculate error along Y (strafe)
        double errorY = (current.getY() - startPose.getY()) - distance; // positive = overshoot

        // Heading correction
        double headingError = startPose.getRotation().minus(current.getRotation()).getRadians();
        double omega = headingError * kHeading;

        // Determine strafe speed
        double vy = Math.signum(distance) * speed; // constant speed in desired direction
        if (Math.abs(errorY) < 0.02) vy = 0;       // stop when within 2 cm

        drive.drive(new ChassisSpeeds(0, vy, omega));
    }

    @Override
    public boolean isFinished() {
        Pose2d current = drive.getPose();
        double traveled = current.getY() - startPose.getY();
        return Math.abs(traveled - distance) < 0.02; // 2 cm tolerance
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds(0, 0, 0));
    }
}
