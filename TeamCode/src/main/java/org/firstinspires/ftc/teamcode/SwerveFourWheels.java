package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.swerve.SwerveWheel;
import org.firstinspires.ftc.teamcode.swerve.TempSwerveWheel;
@Disabled
@TeleOp
public class SwerveFourWheels extends OpMode {
    TempSwerveWheel frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel;
    GamepadEx driver;

    SwerveDriveKinematics kinematics;

    DcMotorEx frontLeftMotor;
    @Override
    public void init() {
        LynxModule module = hardwareMap.getAll(LynxModule.class).get(0);
        module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);


        driver = new GamepadEx(gamepad1);
        frontLeftWheel = new TempSwerveWheel(hardwareMap, "frontLeftMotor", "frontLeftSwervo");
        frontRightWheel = new TempSwerveWheel(hardwareMap, "frontRightMotor", "frontRightSwervo");
        backLeftWheel = new TempSwerveWheel(hardwareMap, "backLeftMotor", "backLeftSwervo");
        backRightWheel = new TempSwerveWheel(hardwareMap, "backRightMotor", "backRightSwervo");

        // +x is forward, +y is left relative to center
        kinematics = new SwerveDriveKinematics(
                new Translation2d(0.105, 0.105), new Translation2d(0.105, -0.105),
                new Translation2d(-0.105, 0.105), new Translation2d(-0.105, -0.105));

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        //frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        ChassisSpeeds speeds = new ChassisSpeeds(driver.getLeftY()*2, -driver.getLeftX()*2, -driver.getRightX()*2);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        frontLeftWheel.run(moduleStates[0]);
        frontRightWheel.run(moduleStates[1]);
        backLeftWheel.run(moduleStates[2]);
        backRightWheel.run(moduleStates[3]);

        telemetry.addData("FL Velocity", frontLeftWheel.getVelocity());
        telemetry.addData("FR Velocity", frontRightWheel.getVelocity());
        telemetry.addData("BL Velocity", backLeftWheel.getVelocity());
        telemetry.addData("BR Velocity", backRightWheel.getVelocity());
        telemetry.addData("FL Mode", frontLeftMotor.getCurrentPosition());

        telemetry.update();

        //frontLeftMotor.setVelocity(300);
    }
}
