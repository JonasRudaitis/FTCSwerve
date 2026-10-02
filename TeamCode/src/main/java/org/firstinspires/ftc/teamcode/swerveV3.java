package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Disabled
@TeleOp
public class swerveV3 extends OpMode {
    Swerve swerve = new Swerve();
    Shooter shooter = new Shooter();
    AprilTagReader aprilTag = new AprilTagReader();

    GamepadEx driver;

    @Override
    public void init(){
        driver = new GamepadEx(gamepad1);
        swerve.initialize(hardwareMap);
        shooter.initialize(hardwareMap);
        aprilTag.initialize(hardwareMap);
    }
    @Override
    public void loop(){
        ChassisSpeeds speeds = new ChassisSpeeds(driver.getLeftX()/2, driver.getLeftY()/2, driver.getRightX()*2.5);

        if (driver.isDown(GamepadKeys.Button.A)) {
            shooter.shootFromDistance(8.0);
        }
        else {
            shooter.setVelocity(0);
        }
        if (driver.isDown(GamepadKeys.Button.B)) {
            double angle = aprilTag.getRedGoalYaw();
            if (angle > 5.0) {speeds = new ChassisSpeeds(0, 0, 1.5);}
            if (angle < -5.0) {speeds = new ChassisSpeeds(0, 0, -1.5);}
            telemetry.addData("tagDistance", aprilTag.getRedGoalDistance());
        }

        speeds = Swerve.drive(speeds);
        telemetry.update();
    }
}
