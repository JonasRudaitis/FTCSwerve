package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.KickerSubsytem;


public class KickCommand extends SequentialCommandGroup {
    KickerSubsytem kicker;

    public KickCommand(KickerSubsytem kicker) {
        this.kicker = kicker;
        addCommands(
                new InstantCommand(kicker::out, kicker),
                new WaitCommand(400),
                new InstantCommand(kicker::in, kicker)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        kicker.in();
    }
}