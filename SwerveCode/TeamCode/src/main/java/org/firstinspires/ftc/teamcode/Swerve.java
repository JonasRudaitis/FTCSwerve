package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Swerve {
    static final double ticksToMeters = 850.8;
    HardwareMap hmap;
    static DcMotorEx leftMotor, rightMotor;
    static AnalogInput leftHeading, rightHeading;
    static CRServo leftSwervo, rightSwervo;

    static SwerveDriveKinematics kinematics;

    static double lastRawLeftAngle, lastRawRightAngle;

    static int LHalfRots =0, RHalfRots =0;

    static double targetR = 90, targetL = 90;

    static public double P  =-0.02;

    public void initialize(HardwareMap hardwareMap) {
        hmap = hardwareMap;

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftDrive");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightDrive");
        leftHeading = hardwareMap.get(AnalogInput.class, "leftSwerveHeading");
        rightHeading = hardwareMap.get(AnalogInput.class, "rightSwerveHeading");
        leftSwervo = hardwareMap.get(CRServo.class, "leftSwervo");
        rightSwervo = hardwareMap.get(CRServo.class, "rightSwervo");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSwervo.setPower(0);
        rightSwervo.setPower(0);

        kinematics = new SwerveDriveKinematics(new Translation2d(0.127, 0), new Translation2d(-0.127, 0));
    }
    // Call this function every loop to drive. Returns the actual chassisSpeeds of the wheels
    public static ChassisSpeeds drive(ChassisSpeeds speeds) {

        // --- Read raw angles from sensors (0–180 range) ---
        double rawLeftAngle = (leftHeading.getVoltage() / 3.3) * 180.0;
        double rawRightAngle = (rightHeading.getVoltage() / 3.3) * 180.0;

        // --- Mirror left sensor if physically inverted ---
        //rawLeftAngle = 180.0 - rawLeftAngle;

// --- Compute deltas for wrap detection (use raw values only) ---
        double deltaL = rawLeftAngle - lastRawLeftAngle;
        double deltaR = rawRightAngle - lastRawRightAngle;

// --- Detect wrap-around for left servo ---
        if (Math.abs(deltaL) > 90) {  // threshold for detecting wrap
            if (deltaL < 0) {
                // went from ~180 → ~0 (forward rotation)
                LHalfRots++;
            } else {
                // went from ~0 → ~180 (reverse rotation)
                LHalfRots--;
            }
        }

// --- Detect wrap-around for right servo ---
        if (Math.abs(deltaR) > 90) {
            if (deltaR < 0) {
                RHalfRots++;
            } else {
                RHalfRots--;
            }
        }

// --- Compute continuous total angle (can exceed 360) ---
        double totalAngleL = LHalfRots * 180.0 + rawLeftAngle;
        double totalAngleR = RHalfRots * 180.0 + rawRightAngle;

// --- Wrap into [0, 360) range ---
        double leftAngle  = ((totalAngleL % 360.0) + 360.0) % 360.0;
        double rightAngle = ((totalAngleR % 360.0) + 360.0) % 360.0;

// --- Save raw angles for next loop iteration ---
        lastRawLeftAngle = rawLeftAngle;
        lastRawRightAngle = rawRightAngle;


        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        SwerveModuleState leftOptimized = SwerveModuleState.optimize(moduleStates[0], new Rotation2d(Math.toRadians(leftAngle)));
        SwerveModuleState rightOptimized = SwerveModuleState.optimize(moduleStates[1], new Rotation2d(Math.toRadians(rightAngle)));

        // only change direction of wheels if we need to
        if (leftOptimized.speedMetersPerSecond != 0 || rightOptimized.speedMetersPerSecond != 0) {
            targetR = ((rightOptimized.angle.getDegrees() % 360) + 360) % 360;
            targetL = ((leftOptimized.angle.getDegrees() % 360) + 360) % 360;
        }

        // Calculate shortest angular difference
        double difference = ((targetL - leftAngle + 540) % 360) - 180;

        leftSwervo.setPower(difference * P);
        leftMotor.setVelocity(leftOptimized.speedMetersPerSecond*ticksToMeters);

        difference = ((targetR - rightAngle + 540) % 360) - 180;

        rightSwervo.setPower(difference * P);
        rightMotor.setVelocity(rightOptimized.speedMetersPerSecond*ticksToMeters);


        // Calculate actual ChassisSpeeds
        SwerveModuleState actualLeftState = new SwerveModuleState(leftMotor.getVelocity()/ticksToMeters, new Rotation2d(Math.toRadians(leftAngle)));
        SwerveModuleState actualRightState = new SwerveModuleState(rightMotor.getVelocity()/ticksToMeters, new Rotation2d(Math.toRadians(rightAngle)));

        return kinematics.toChassisSpeeds(actualLeftState, actualRightState);
    }

}
