package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.commands.AutoShootCommandOneBall;
import org.firstinspires.ftc.teamcode.commands.KickCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp
public class RedTeleOp extends CommandOpMode {
    private SwerveSubsystem drive;
    private ShooterSubsystem shooter;
    private KickerSubsytem kicker;
    private VisionSubsystem vision;

    @Override
    public void initialize() {
        telemetry.addLine("initializing...");
        telemetry.update();

        // Subsystems
        drive = new SwerveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsytem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap, "red");

        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);

        driverOp.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new ShootCommand(shooter, kicker, 250, true)

        );
        driverOp.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ShootCommand(shooter, kicker, 1400, true)
        );
        driverOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new AutoShootCommand(drive, shooter, kicker, vision, driverOp, telemetry)
        );
        driverOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new AutoShootCommandOneBall(drive, shooter, kicker, vision, driverOp, telemetry)
        );
        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new KickCommand(kicker)
        );
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new TurnCommand(drive, drive.getHeading().minus(new Rotation2d(Math.PI / 4.0)).getDegrees())
        );
        driverOp.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new InstantCommand(() -> {
                    drive.resetHeading();
                })
        );
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> {
                    drive.forward();
                })
        );
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> {
                    drive.back();
                })
        );

        driver2.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> {
                    CommandScheduler.getInstance().cancelAll();
                    shooter.setVelocity(0);
                    kicker.in();
                }
                )
        );
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    drive.backLeftWheel.flip();
                }
                )
        );
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    drive.backRightWheel.flip();
                }
                )
        );
        Trigger leftTrigger = new Trigger(() ->
                driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.4
        );
        leftTrigger.whenActive(
                new InstantCommand(() -> {
                    drive.frontLeftWheel.flip();
                }
                )
        );
        Trigger rightTrigger = new Trigger(() ->
                driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.4
        );
        rightTrigger.whenActive(
                new InstantCommand(() -> {
                    drive.frontRightWheel.flip();
                }
                )
        );


        drive.setDefaultCommand(
                new RunCommand(() -> {
                    double y = driverOp.getLeftY() * drive.max_speed;
                    double x = -driverOp.getLeftX() * drive.max_speed;
                    double rot = (-driverOp.getRightX() + -driver2.getRightX()) * Math.PI;

                    // deadzones
                    if (Math.abs(x) < 0.05) x = 0;
                    if (Math.abs(y) < 0.05) y = 0;
                    if (Math.abs(rot) < 0.05) rot = 0;

                    //ChassisSpeeds speeds = new ChassisSpeeds(y, x, rot);
                    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(y, x, rot, drive.getHeading());
                    // drive.teleDrive(speeds);
                    drive.drive(speeds);

                }, drive)
        );

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Pose x: ", drive.getPose().getX());
        telemetry.addData("Pose y: ", drive.getPose().getY());
        telemetry.addData("Pose heading: ", drive.getPose().getHeading());
    }
}
