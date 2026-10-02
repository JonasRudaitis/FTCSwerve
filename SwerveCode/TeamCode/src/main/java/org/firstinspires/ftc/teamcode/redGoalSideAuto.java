package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeCommand;
import org.firstinspires.ftc.teamcode.commands.TurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwerveSubsystem;
import org.firstinspires.ftc.teamcode.commands.DriveDistanceCommand;
@Autonomous(preselectTeleOp = "RedTeleOp")
public class redGoalSideAuto extends CommandOpMode {
    private SwerveSubsystem drive;
    private ShooterSubsystem shooter;
    private KickerSubsytem kicker;

    @Override
    public void initialize() {
        // Initialize the swerve drive subsystem
        drive = new SwerveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        kicker = new KickerSubsytem(hardwareMap);
        register(drive, shooter, kicker);

        telemetry.addLine("Red Goal Auto Initialized");
        telemetry.update();
        
        schedule(
                    new SequentialCommandGroup(
                        // +ve forward, -ve backward
                        new DriveDistanceCommand(drive, -1.5, 0.5),
                        new ShootCommand(shooter, kicker, 1000, true),
                        new WaitCommand(300),
                        new ShootCommand(shooter, kicker, 1000, true),
                        new WaitCommand(300),
                        new ShootCommand(shooter, kicker, 1000, true),

                        // +ve right, -ve left
                        new StrafeCommand(drive, -0.5, 0.5),
                        // +ve left, -ve right
                        new TurnCommand(drive, -45),

                        // needs to be last: stops the wheels, then points them forward
                        new InstantCommand(() -> {drive.stop();}, drive),
                        new InstantCommand(() -> {drive.forward();}, drive),
                        new RunCommand(()-> {drive.stop();}, drive))


        );


    }
}