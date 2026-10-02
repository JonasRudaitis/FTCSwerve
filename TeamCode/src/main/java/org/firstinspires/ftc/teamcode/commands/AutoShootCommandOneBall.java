package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class AutoShootCommandOneBall extends CommandBase {
    private final SwerveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final KickerSubsytem kicker;
    private final VisionSubsystem vision;
    private final Telemetry telemetry;
    private final GamepadEx gamepad;
    InterpLUT velocity = new InterpLUT(); // Distance: Velocity
    private final double kP = 0.01;

    private double targetVelocity = 0;
    private boolean shotScheduled = false;
    private double x=0;

    public AutoShootCommandOneBall(SwerveSubsystem drive, ShooterSubsystem shooter, KickerSubsytem kicker, VisionSubsystem vision, GamepadEx gamepad, Telemetry telemetry) {
        this.drive = drive;
        this.shooter = shooter;
        this.kicker = kicker;
        this.vision = vision;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        addRequirements(drive, shooter, kicker, vision);

        velocity.add(-1.0, 0);
        velocity.add(0,1000);
        velocity.add(4,1000);
        velocity.add(7,1100);
        velocity.add(8,1300);
        velocity.add(10,1500);
        velocity.add(11,2150);
        velocity.add(12,2400);
        velocity.add(2000,2400);
        velocity.createLUT();
    }

    @Override
    public void initialize() {
        shotScheduled = false;
    }

    @Override
    public void execute() {
        double yawError = vision.getGoalYaw();      // degrees
        double distance = vision.getGoalDistance(); // meters

        telemetry.addData("Yaw error", yawError);
        telemetry.addData("distance", distance);


        if (distance == -1){
            gamepad.gamepad.rumble(500);
            shotScheduled = true;
            return;
        }

        targetVelocity = velocity.get(distance);
        shooter.setVelocity((int) targetVelocity);

        // Step 3: check alignment & shoot once
        if (!shotScheduled) {
            CommandScheduler.getInstance().schedule(
                    /*
                    new SequentialCommandGroup(
                            new TurnCommand(drive, vision.getGoalYaw()),
                            new ShootCommand(shooter, kicker, (int) targetVelocity, true)
                    )

                     */
                    new ShootCommand(shooter, kicker, (int) targetVelocity, true)
            );
            shotScheduled = true;
        }
        telemetry.update();
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds(0, 0, 0));
        shooter.stop();
        kicker.in();
    }

    @Override
    public boolean isFinished() {
        // End when we've scheduled a shot and it's done
        return shotScheduled;
    }
}
