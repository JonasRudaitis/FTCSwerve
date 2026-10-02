package org.firstinspires.ftc.teamcode.swerve;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp
public class swerveV4 extends OpMode {
    SwerveWheel leftWheel, rightWheel;
    GamepadEx driver;

    SwerveDriveKinematics kinematics;

    @Override
    public void init() {
        leftWheel = new SwerveWheel(hardwareMap, "frontLeftMotor", "frontLeftSwervo", "frontLeftHeading");
        rightWheel = new SwerveWheel(hardwareMap, "frontRightMotor", "frontRightSwervo", "frontRightHeading");
        driver = new GamepadEx(gamepad1);

        // +x is forward, +y is left relative to center
        kinematics = new SwerveDriveKinematics(new Translation2d(0, 0.105), new Translation2d(0, -0.105));
    }

    @Override
    public void loop() {
        ChassisSpeeds speeds = new ChassisSpeeds(driver.getLeftY()*2, driver.getLeftX()*2, -driver.getRightX()*2);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        leftWheel.run(moduleStates[0]);
        rightWheel.run(moduleStates[1]);

        telemetry.addData("Left Angle: ", leftWheel.getAngle());
        telemetry.addData("Left Velocity", leftWheel.getVelocity());
        telemetry.addData("Right Angle: ", rightWheel.getAngle());
        telemetry.addData("Right Velocity", rightWheel.getVelocity());

        telemetry.update();
    }
}
