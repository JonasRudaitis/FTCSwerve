package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.swerve.SwerveWheel;
import org.firstinspires.ftc.teamcode.swerve.TempSwerveWheel;

public class SwerveSubsystem extends SubsystemBase {
    private final ElapsedTime runtime = new ElapsedTime();
    public SwerveWheel frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel;
    GamepadEx driver;

    IMU gyro;
    private SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;

    Pose2d pose;
    public double max_speed = 1.5;
    private Rotation2d desiredHeading = new Rotation2d();


    public SwerveSubsystem(final HardwareMap hMap) {
        runtime.reset();

        gyro = hMap.get(IMU.class, "imu");
        gyro.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
        ));
        resetHeading();
//        frontLeftWheel = new SwerveWheel(hMap, "frontLeftMotor", "frontLeftSwervo", "frontLeftHeading");
//        frontRightWheel = new SwerveWheel(hMap, "frontRightMotor", "frontRightSwervo", "frontRightHeading");
//        backLeftWheel = new SwerveWheel(hMap, "backLeftMotor", "backLeftSwervo", "backLeftHeading");
//        backRightWheel = new SwerveWheel(hMap, "backRightMotor", "backRightSwervo","backRightHeading");
        backRightWheel = new SwerveWheel(hMap, "frontLeftMotor", "frontLeftSwervo", "frontLeftHeading");
        backLeftWheel = new SwerveWheel(hMap, "frontRightMotor", "frontRightSwervo", "frontRightHeading");
        frontRightWheel = new SwerveWheel(hMap, "backLeftMotor", "backLeftSwervo", "backLeftHeading");
        frontLeftWheel = new SwerveWheel(hMap, "backRightMotor", "backRightSwervo","backRightHeading");

        // +x is forward, +y is left relative to center
        kinematics = new SwerveDriveKinematics(
                new Translation2d(0.105, 0.105), new Translation2d(0.105, -0.105),
                new Translation2d(-0.105, 0.105), new Translation2d(-0.105, -0.105)
        );

        odometry = new SwerveDriveOdometry(
                        kinematics, new Rotation2d(gyro.getRobotYawPitchRollAngles().getYaw()),
                        new Pose2d(0, 0, new Rotation2d()
                        )
                );
        pose = odometry.getPoseMeters();
    } // end of constructor
    @Override
    public void periodic() {
        // Get wheel positions (in meters)
        double fl = frontLeftWheel.getVelocity();
        double fr = frontRightWheel.getVelocity();
        double bl = backLeftWheel.getVelocity();
        double br = backRightWheel.getVelocity();


        // Get the gyro angle
        Rotation2d heading = getHeading();
        // Use runtime as time base (seconds)
        double currentTime = runtime.seconds();

        // Update odometry with time, heading, and wheel distances
        pose = odometry.updateWithTime(currentTime, heading,
                new SwerveModuleState(frontLeftWheel.getVelocity(), new Rotation2d(Math.toRadians(frontLeftWheel.getAngle()))),
                new SwerveModuleState(frontRightWheel.getVelocity(), new Rotation2d(Math.toRadians(frontRightWheel.getAngle()))),
            new SwerveModuleState(backLeftWheel.getVelocity(), new Rotation2d(Math.toRadians(backLeftWheel.getAngle()))),
            new SwerveModuleState(backRightWheel.getVelocity(), new Rotation2d(Math.toRadians(backRightWheel.getAngle())))
                );
    }
    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        frontLeftWheel.run(states[0]);
        frontRightWheel.run(states[1]);
        backLeftWheel.run(states[2]);
        backRightWheel.run(states[3]);
    } // end of drive

    public void teleDrive(ChassisSpeeds speeds){
        if (Math.abs(speeds.omegaRadiansPerSecond) > 0.1){
            desiredHeading = getHeading();
        }
        else {
            speeds.omegaRadiansPerSecond = desiredHeading.minus(getHeading()).getRadians() * Math.PI * 1.5;
        }
        drive(speeds);
    }
    public void stop(){
        drive(new ChassisSpeeds(0, 0, 0));
    }
    public void forward(){
        frontLeftWheel.forward();
        frontRightWheel.forward();
        backLeftWheel.forward();
        backRightWheel.forward();
    }
    public void back(){
        frontLeftWheel.back();
        frontRightWheel.back();
        backLeftWheel.back();
        backRightWheel.back();
    }
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getHeading(){
        return new Rotation2d(gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }
    public void resetHeading() {
        gyro.resetYaw();
        desiredHeading = new Rotation2d();
    }
} // end of MecanumSubsytstem
