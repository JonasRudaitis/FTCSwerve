package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.KickerSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(ShooterSubsystem shooter, KickerSubsytem kicker, int targetVelocity, boolean stop) {

        if (stop) {
            addCommands(
                    new InstantCommand(() -> shooter.setVelocity(targetVelocity), shooter),
                    new WaitUntilCommand(shooter::isAtTarget),

                    new InstantCommand(kicker::out, kicker),
                    new WaitCommand(500),
                    new InstantCommand(kicker::in, kicker),

                    //new WaitCommand(200),
                    new InstantCommand(shooter::stop, shooter)
            );
        }
        else {
            addCommands(
                    new InstantCommand(() -> shooter.setVelocity(targetVelocity), shooter),
                    new WaitUntilCommand(shooter::isAtTarget),

                    new InstantCommand(kicker::out, kicker),
                    new WaitCommand(500),
                    new InstantCommand(kicker::in, kicker)
            );
        }
    }
}
