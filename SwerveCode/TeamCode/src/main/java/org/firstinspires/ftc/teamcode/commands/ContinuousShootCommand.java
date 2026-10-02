package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsytem;


public class ContinuousShootCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final KickerSubsytem kicker;
    private final int targetVelocity;
    private final ElapsedTime timer = new ElapsedTime();

    private boolean hasKicked = false;

    public ContinuousShootCommand(ShooterSubsystem shooter, KickerSubsytem kicker, int targetVelocity) {
        this.shooter = shooter;
        this.kicker = kicker;
        this.targetVelocity = targetVelocity;
        addRequirements(shooter, kicker);
    }

    @Override
    public void initialize() {
        shooter.setVelocity(targetVelocity);
        timer.reset();
        hasKicked = false;
    }

    @Override
    public void execute() {
        if (shooter.isAtTarget()) {
            // Every 0.5s, perform a kick cycle (out -> in)
            if (timer.seconds() >= 0.5) {
                kicker.out();
                hasKicked = true;
                timer.reset();
            } else if (hasKicked && timer.seconds() >= 0.25) {
                // After 0.25s of being out, return in
                kicker.in();
                hasKicked = false;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        kicker.in(); // ensure safe reset
    }

    @Override
    public boolean isFinished() {
        return false; // run until button released
    }
}