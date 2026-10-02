package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
@TeleOp(name="Swerve Debug - 4 Modules", group="Debug")
public class SwerveModuleTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // ======== CREATE SWERVE MODULES (your hardware names) =========
        SwerveWheel fl = new SwerveWheel(hardwareMap,
                "frontLeftMotor", "frontLeftSwervo", "frontLeftHeading");

        SwerveWheel fr = new SwerveWheel(hardwareMap,
                "frontRightMotor", "frontRightSwervo", "frontRightHeading");

        SwerveWheel bl = new SwerveWheel(hardwareMap,
                "backLeftMotor", "backLeftSwervo", "backLeftHeading");

        SwerveWheel br = new SwerveWheel(hardwareMap,
                "backRightMotor", "backRightSwervo", "backRightHeading");

        // direct hardware access (manual testing)
        DcMotorEx flDrive = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        DcMotorEx frDrive = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        DcMotorEx blDrive = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        DcMotorEx brDrive = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        CRServo flSteer = hardwareMap.get(CRServo.class, "frontLeftSwervo");
        CRServo frSteer = hardwareMap.get(CRServo.class, "frontRightSwervo");
        CRServo blSteer = hardwareMap.get(CRServo.class, "backLeftSwervo");
        CRServo brSteer = hardwareMap.get(CRServo.class, "backRightSwervo");

        AnalogInput flHead = hardwareMap.get(AnalogInput.class, "frontLeftHeading");
        AnalogInput frHead = hardwareMap.get(AnalogInput.class, "frontRightHeading");
        AnalogInput blHead = hardwareMap.get(AnalogInput.class, "backLeftHeading");
        AnalogInput brHead = hardwareMap.get(AnalogInput.class, "backRightHeading");
        // ==================================================================

        telemetry.addLine("4-Module Swerve Debug Ready");
        telemetry.addLine("Left Stick X = steer all servos");
        telemetry.addLine("Right Stick Y = drive all motors");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double steerPower = gamepad1.left_stick_x * 0.4;
            double drivePower = gamepad1.right_stick_y * 0.3;

            // manual testing (servo + motor)
            flSteer.setPower(steerPower);
            frSteer.setPower(steerPower);
            blSteer.setPower(steerPower);
            brSteer.setPower(steerPower);

            flDrive.setPower(drivePower);
            frDrive.setPower(drivePower);
            blDrive.setPower(drivePower);
            brDrive.setPower(drivePower);

            telemetry.addLine("========== FRONT LEFT ==========");
            telemetry.addData("Voltage", "%.3f", flHead.getVoltage());
            telemetry.addData("Raw Angle", "%.1f°", (flHead.getVoltage() / 3.3) * 180);
            telemetry.addData("Unwrapped", "%.1f°", fl.getAngle());
            telemetry.addData("Velocity", "%.1f", fl.getVelocity());

            telemetry.addLine("\n========== FRONT RIGHT ==========");
            telemetry.addData("Voltage", "%.3f", frHead.getVoltage());
            telemetry.addData("Raw Angle", "%.1f°", (frHead.getVoltage() / 3.3) * 180);
            telemetry.addData("Unwrapped", "%.1f°", fr.getAngle());
            telemetry.addData("Velocity", "%.1f", fr.getVelocity());

            telemetry.addLine("\n========== BACK LEFT ==========");
            telemetry.addData("Voltage", "%.3f", blHead.getVoltage());
            telemetry.addData("Raw Angle", "%.1f°", (blHead.getVoltage() / 3.3) * 180);
            telemetry.addData("Unwrapped", "%.1f°", bl.getAngle());
            telemetry.addData("Velocity", "%.1f", bl.getVelocity());

            telemetry.addLine("\n========== BACK RIGHT ==========");
            telemetry.addData("Voltage", "%.3f", brHead.getVoltage());
            telemetry.addData("Raw Angle", "%.1f°", (brHead.getVoltage() / 3.3) * 180);
            telemetry.addData("Unwrapped", "%.1f°", br.getAngle());
            telemetry.addData("Velocity", "%.1f", br.getVelocity());

            telemetry.update();
            sleep(20);
        }
    }
}
