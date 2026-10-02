package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShooterCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final int targetVelocity;

    public ShooterCommand(ShooterSubsystem shooter, int targetVelocity) {
        this.shooter = shooter;
        this.targetVelocity = targetVelocity;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setVelocity(targetVelocity);
    }

    @Override
    public boolean isFinished() {
        return shooter.isAtTarget();
    }
}