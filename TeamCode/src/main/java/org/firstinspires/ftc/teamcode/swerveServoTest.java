package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
@Disabled
@Config
@TeleOp
public class swerveServoTest extends OpMode {
    double lastLeftAngle;
    double lastRightAngle;

    int LHalfRots =0;
    int RHalfRots =0;
    public static double P  = -0.02;
    CRServo leftSwervo, rightSwervo;
    AnalogInput leftHeading, rightHeading;
    public static double targetR = 90;
    public static double targetL = 90;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        leftHeading = hardwareMap.get(AnalogInput.class, "leftSwerveHeading");
        rightHeading = hardwareMap.get(AnalogInput.class, "rightSwerveHeading");
        leftSwervo = hardwareMap.get(CRServo.class, "leftSwervo");
        rightSwervo = hardwareMap.get(CRServo.class, "rightSwervo");
    }
    @Override
    public void loop(){


        double leftAngle = (leftHeading.getVoltage()/3.3*180.0);
        double rightAngle = (rightHeading.getVoltage()/3.3)*180.0;

        double deltaL = leftAngle - lastLeftAngle;
        double deltaR = rightAngle - lastRightAngle;

        if (Math.abs(deltaL) > 90) { // threshold for wrap detection
            if (deltaL < 0) {
                // went from ~180 to ~0: completed another half turn
                LHalfRots++;
            } else {
                // went from ~0 to ~180: reversed
                LHalfRots--;
            }
        }
        if (Math.abs(deltaR) > 90) { // threshold for wrap detection
            if (deltaR < 0) {
                // went from ~180 to ~0: completed another half turn
                RHalfRots++;
            } else {
                // went from ~0 to ~180: reversed
                RHalfRots--;
            }
        }

        lastLeftAngle = leftAngle;
        lastRightAngle = rightAngle;

        // Calculate total angle
        double totalAngleL = LHalfRots * 180.0 + leftAngle;
        double totalAngleR = RHalfRots * 180.0 + rightAngle;

        // Wrap into 0 - 360
        double wrappedAngleL = totalAngleL % 360.0;
        if (wrappedAngleL < 0) wrappedAngleL += 360.0;

        leftAngle = wrappedAngleL;

        double wrappedAngleR = totalAngleR % 360.0;
        if (wrappedAngleR < 0) wrappedAngleR += 360.0;

        rightAngle = wrappedAngleR;


        // Normalize angles to [0, 360)
        leftAngle = (leftAngle % 360 + 360) % 360;
        targetL = (targetL % 360 + 360) % 360;

        // Calculate shortest angular difference
        double difference = ((targetL - leftAngle + 540) % 360) - 180;

        if (Math.abs(difference) <= 90) {
            // Turn directly toward target
            leftSwervo.setPower(difference * P);
            //leftMotor.setPower(moduleStates[0].speedMetersPerSecond);
        } else {
            // Flip 180 degrees and drive in reverse
            double flippedTarget = (targetL + 180) % 360;
            double flippedDifference = ((flippedTarget - leftAngle + 540) % 360) - 180;

            leftSwervo.setPower(flippedDifference * P);
            //leftMotor.setPower(-moduleStates[0].speedMetersPerSecond);
        }

        telemetry.addLine("Target Angles");
        telemetry.addData("L: ", targetL);
        telemetry.addData("R: ", targetL);

        telemetry.addData("L Pow: ", leftSwervo.getPower());
        telemetry.addData("R Pow: ", rightSwervo.getPower());


        telemetry.addData("L Head: ", leftAngle);
        telemetry.addData("R Head: ", rightAngle);

        telemetry.update();
    }
}
