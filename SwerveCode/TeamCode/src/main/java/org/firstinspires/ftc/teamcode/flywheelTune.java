package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.arcrobotics.ftclib.controller.PIDFController;
@Disabled
@TeleOp(name = "Flywheel PIDF Tuner (Ticks/sec)", group = "Tuning")
public class flywheelTune extends OpMode {

    // ------------------ Hardware ------------------
    private DcMotorEx flywheel;

    // ------------------ PIDF Controller ------------------
    private PIDFController controller;

    // Initial PIDF values you will tune
    private double kP = 0.0005;
    private double kI = 0.0;
    private double kD = 0.0;
    private double kF = 0.0;

    // Adjustment increments
    private final double pInc = 0.0001;
    private final double iInc = 0.00001;
    private final double dInc = 0.0001;
    private final double fInc = 0.0001;
    private final double spdInc = 50;  // ticks/sec change per click

    // ------------------ State ------------------
    private double targetTps = 2000;      // ticks per second
    private boolean closedLoop = false;   // A button toggles closed loop
    private double lastPower = 0;

    // Debounce
    boolean aPrev,bPrev,xPrev,yPrev;
    boolean upPrev,downPrev,leftPrev,rightPrev;
    boolean lbPrev,rbPrev;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "leftShooter");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        controller = new PIDFController(kP, kI, kD, kF);
        controller.setSetPoint(targetTps);

        telemetry.addLine("Flywheel PIDF Tuner Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ---------------- BUTTONS ----------------
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        boolean lb = gamepad1.left_bumper;
        boolean rb = gamepad1.right_bumper;

        // -------- Toggle Closed Loop (A) --------
        if (a && !aPrev) closedLoop = !closedLoop;

        // -------- Reset PIDF (B) --------
        if (b && !bPrev) {
            kP = 0.0005;
            kI = 0.0;
            kD = 0.0;
            kF = 0.0;
        }

        // -------- Increase/decrease P --------
        if (up && !upPrev)    kP += pInc;
        if (down && !downPrev && kP - pInc >= 0) kP -= pInc;

        // -------- Increase/decrease I --------
        if (right && !rightPrev)   kI += iInc;
        if (left && !leftPrev && kI - iInc >= 0) kI -= iInc;

        // -------- Increase/decrease D --------
        if (rb && !rbPrev)   kD += dInc;
        if (lb && !lbPrev && kD - dInc >= 0) kD -= dInc;

        // -------- Increase/decrease F --------
        if (gamepad1.right_trigger > 0.5) kF += fInc;
        if (gamepad1.left_trigger > 0.5 && kF - fInc >= 0) kF -= fInc;

        // -------- Change target speed (Left Stick Y) --------
        double stick = -gamepad1.left_stick_y;
        if (Math.abs(stick) > 0.15) {
            targetTps += stick * spdInc;
            if (targetTps < 0) targetTps = 0;
        }

        // Update controller parameters
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
        controller.setF(kF);
        controller.setSetPoint(targetTps);

        // ---------------- CONTROL ----------------
        double currentTps = flywheel.getVelocity(); // this IS ticks/sec

        if (closedLoop) {
            double output = controller.calculate(currentTps);
            output = clamp(output, -1, 1);
            lastPower = output;
            flywheel.setPower(output);
        } else {
            double manual = gamepad1.right_stick_x;
            lastPower = manual;
            flywheel.setPower(manual);
        }

        // ---------------- TELEMETRY ----------------
        telemetry.addLine(closedLoop ? "MODE: CLOSED LOOP" : "MODE: MANUAL OPEN LOOP");
        telemetry.addData("Target (tps)", "%.1f", targetTps);
        telemetry.addData("Velocity (tps)", "%.1f", currentTps);
        telemetry.addData("Power", "%.3f", lastPower);

        telemetry.addLine("\nPIDF:");
        telemetry.addData("P", kP);
        telemetry.addData("I", kI);
        telemetry.addData("D", kD);
        telemetry.addData("F", kF);

        telemetry.addLine("\nControls:");
        telemetry.addLine("A = Toggle Control Mode");
        telemetry.addLine("Left Stick Y = Target Speed");
        telemetry.addLine("DPad Up/Down = P +/-");
        telemetry.addLine("DPad Right/Left = I +/-");
        telemetry.addLine("RB/LB = D +/-");
        telemetry.addLine("RT/LT = F +/-");
        telemetry.addLine("Right Stick X (manual mode) = power");
        telemetry.addLine("B = Reset PIDF");
        telemetry.update();

        // ------------- Update previous states -------------
        aPrev=a; bPrev=b;
        upPrev=up; downPrev=down; leftPrev=left; rightPrev=right;
        lbPrev=lb; rbPrev=rb;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
