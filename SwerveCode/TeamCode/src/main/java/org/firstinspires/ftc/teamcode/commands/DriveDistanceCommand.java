package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;

public class DriveDistanceCommand extends CommandBase {

    private final SwerveSubsystem drive;
    private final double distance; // meters, + forward, - backward
    private Pose2d targetPose;

    private final double maxSpeed;   // m/s
    private final double kY = 2.0;   // P gain for Y correction
    private final double kHeading = 4.0; // P gain for heading correction (rad -> vx adjustment)

    public DriveDistanceCommand(SwerveSubsystem drive, double distanceMeters, double maxSpeed) {
        this.drive = drive;
        this.distance = distanceMeters;
        this.maxSpeed = Math.abs(maxSpeed);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d start = drive.getPose();

        // Compute target X,Y along heading
        double heading = start.getHeading();
        double dx = distance * Math.cos(heading);
        double dy = distance * Math.sin(heading);

        targetPose = new Pose2d(
                start.getX() + dx,
                start.getY() + dy,
                new Rotation2d(start.getHeading())
        );
    }

    @Override
    public void execute() {
        Pose2d current = drive.getPose();

        // X error along heading
        double heading = targetPose.getHeading();
        double errorX = (targetPose.getX() - current.getX()) * Math.cos(heading)
                + (targetPose.getY() - current.getY()) * Math.sin(heading);

        // Y error perpendicular to heading
        double errorY = -(targetPose.getX() - current.getX()) * Math.sin(heading)
                + (targetPose.getY() - current.getY()) * Math.cos(heading);

        // Heading error
        double errorHeading = targetPose.getHeading() - current.getHeading();

        // Wrap heading error to [-pi, pi]
        while (errorHeading > Math.PI) errorHeading -= 2 * Math.PI;
        while (errorHeading < -Math.PI) errorHeading += 2 * Math.PI;

        // Compute chassis speeds
        double vx = Math.signum(distance) * Math.min(Math.abs(errorX), maxSpeed);
        double vy = kY * errorY;
        double omega = kHeading * errorHeading;

        drive.drive(new ChassisSpeeds(vx, vy, omega));
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        Pose2d current = drive.getPose();
        double errorX = (targetPose.getX() - current.getX()) * Math.cos(targetPose.getHeading())
                + (targetPose.getY() - current.getY()) * Math.sin(targetPose.getHeading());
        return Math.abs(errorX) < 0.02; // finish when X along heading is within 2 cm
    }
}
