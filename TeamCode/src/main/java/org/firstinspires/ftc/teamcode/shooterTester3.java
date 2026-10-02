package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
@Disabled
@Config
@TeleOp(name = "Shooter Tester 3", group = "Testing")
public class shooterTester3 extends OpMode {

    public static int targetVel = 1500;   // ticks per second
    public static int tolerance = 100;    // acceptable deviation

    DcMotorEx leftShooter;
    DcMotorEx rightShooter;
    VisionSubsystem vision;

    private ElapsedTime timer = new ElapsedTime();

    private boolean belowTarget = false;
    private boolean timerRunning = false;
    private double recoveryTime = 0;
    private double spinupTime = 0;
    private boolean spinupRecorded = false;

    @Override
    public void init() {
        vision = new VisionSubsystem(hardwareMap, "red");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");

        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // --- Command both shooters to target velocity ---
        leftShooter.setVelocity(targetVel);
        rightShooter.setVelocity(targetVel);

        double leftVel = leftShooter.getVelocity();
        double rightVel = rightShooter.getVelocity();
        double avgVel = (leftVel + rightVel) / 2.0;

        // --- Record spin-up time from rest (first time reaching tolerance) ---
        if (!spinupRecorded && Math.abs(avgVel - targetVel) < tolerance) {
            spinupTime = timer.seconds();
            spinupRecorded = true;
        }

        // --- Detect when shooter slows down below target (e.g. after a shot) ---
        if (avgVel < targetVel - tolerance && !belowTarget) {
            belowTarget = true;
            timer.reset();           // start timing recovery
            timerRunning = true;
        }

        // --- Detect when it recovers to target velocity ---
        if (belowTarget && Math.abs(avgVel - targetVel) < tolerance) {
            belowTarget = false;
            if (timerRunning) {
                recoveryTime = timer.seconds();
                timerRunning = false;
            }
        }

        // --- Telemetry output ---
        telemetry.addData("Target Velocity", targetVel);
        telemetry.addData("Left Vel", leftVel);
        telemetry.addData("Right Vel", rightVel);
        telemetry.addData("Average Vel", avgVel);
        telemetry.addData("Within Tolerance", Math.abs(avgVel - targetVel) < tolerance);
        telemetry.addData("Spin-up Time (0 → target)", spinupTime);
        telemetry.addData("Recovery Time (after dip)", recoveryTime);
        telemetry.addData("Timer Running", timerRunning);
        telemetry.addData("distance: ", vision.getRedGoalDistance());
        telemetry.update();
    }

    @Override
    public void start() {
        // Reset everything when opmode starts
        timer.reset();
        belowTarget = false;
        timerRunning = false;
        recoveryTime = 0;
        spinupTime = 0;
        spinupRecorded = false;
    }
}