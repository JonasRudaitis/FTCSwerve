package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumSubsystem extends SubsystemBase {
    private final ElapsedTime runtime = new ElapsedTime();
    private final DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    IMU gyro;
    private MecanumDriveKinematics kinematics;
    MecanumDriveOdometry odometry;
    private double tickToMeters = 4583.66;

    public MecanumSubsystem(final HardwareMap hMap) {
        runtime.reset();

        gyro = hMap.get(IMU.class, "imu");
        gyro.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)
        ));
        // Get Motors
        frontLeftMotor = hMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hMap.get(DcMotorEx.class, "backRightMotor");
        // Set directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Positions of wheels relative to robot center (in metres)
        Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
        Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
        Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
        Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
        kinematics = new MecanumDriveKinematics
                (
                        m_frontLeftLocation, m_frontRightLocation,
                        m_backLeftLocation, m_backRightLocation
                );
        odometry = new MecanumDriveOdometry
                (
                        kinematics, new Rotation2d(gyro.getRobotYawPitchRollAngles().getYaw()),
                        new Pose2d(0, 0, new Rotation2d()
                        )
                );
    } // end of constructor
    @Override
    public void periodic() {
        // Get wheel positions (in meters)
        double fl = frontLeftMotor.getCurrentPosition() / tickToMeters;
        double fr = frontRightMotor.getCurrentPosition() / tickToMeters;
        double bl = backLeftMotor.getCurrentPosition() / tickToMeters;
        double br = backRightMotor.getCurrentPosition() / tickToMeters;


        // Get the gyro angle
        Rotation2d heading = new Rotation2d(
                Math.toRadians(gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES))
        );
        // Use runtime as time base (seconds)
        double currentTime = runtime.seconds();

        // Update odometry with time, heading, and wheel distances
        odometry.updateWithTime(currentTime, heading, new MecanumDriveWheelSpeeds(fl, fr, bl, br));
    }
    public void drive(ChassisSpeeds speeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        frontLeftMotor.setVelocity(wheelSpeeds.frontLeftMetersPerSecond * tickToMeters);
        frontRightMotor.setVelocity(wheelSpeeds.frontRightMetersPerSecond * tickToMeters);
        backLeftMotor.setVelocity(wheelSpeeds.rearLeftMetersPerSecond * tickToMeters);
        backRightMotor.setVelocity(wheelSpeeds.rearRightMetersPerSecond * tickToMeters);
    } // end of drive
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Rotation2d getHeading(){
        return new Rotation2d(gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }
} // end of MecanumSubststem
