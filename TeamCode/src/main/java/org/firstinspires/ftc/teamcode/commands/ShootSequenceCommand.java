package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.*;

import org.firstinspires.ftc.teamcode.subsystems.KickerSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootSequenceCommand extends SequentialCommandGroup {

    public ShootSequenceCommand(ShooterSubsystem shooter, KickerSubsytem kicker, int targetVelocity) {
        addCommands(
                new ShooterCommand(shooter, targetVelocity),                // spin up
                new WaitUntilCommand(shooter::isAtTarget),                  // wait until at target
                new KickCommand(kicker),                                    // fire once
                new InstantCommand(shooter::stop, shooter)                  // stop shooter
        );
    }
}