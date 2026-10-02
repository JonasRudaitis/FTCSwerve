package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.KickerSubsytem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Config
@TeleOp
public class ShooterTuner extends OpMode {
    ShooterSubsystem shooter;
    VisionSubsystem vision;
    KickerSubsytem kicker;
    public static int target = 0;
    public static boolean kick = false;

    @Override
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        vision = new VisionSubsystem(hardwareMap,  "blue");
        kicker = new KickerSubsytem(hardwareMap);
    }

    @Override
    public void loop() {
        shooter.periodic();
        vision.periodic();
        if (kick)
            kicker.out();
        else
            kicker.in();

        shooter.setVelocity(target);
        telemetry.addData("Distance", vision.getGoalDistance());
        telemetry.update();
    }
}
