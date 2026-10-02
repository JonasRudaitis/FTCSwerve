package org.firstinspires.ftc.teamcode.swerve;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class SwerveWheel2 {

    // ==== CONFIG ====
    public static double P = 0.007;              // Base steering gain
    public static double FORWARD_OFFSET = 90;     // Forward reference
    public static double TICKS_PER_REV = 145.1 * 2;
    public static double WHEEL_CIRC = 0.072 * Math.PI;
    public static double METERS_PER_TICK = WHEEL_CIRC / TICKS_PER_REV;
    public static double DEADZONE_ANGLE = 1.0;    // Ignore errors <1° near ±90°
    public static double REDUCED_GAIN_FACTOR = 0.5; // Reduce P near ±90°/270°
    public static double NEAR_SIDEWAYS = 5.0;    // Degrees threshold for sideways

    // Hardware
    private final DcMotorEx driveMotor;
    private final CRServo steerServo;
    private final AnalogInput encoder;

    // Continuous angle tracking
    private double lastRaw = 90;
    private int halfTurns = 0;

    // Filtered / target angles
    private double targetAngle;       // Sticky target angle
    private double angleFiltered = 0; // Smoothed sensor reading
    private double currentVelocity = 0;

    public SwerveWheel2(HardwareMap hMap, String motor, String steer, String heading) {
        driveMotor = hMap.get(DcMotorEx.class, motor);
        steerServo = hMap.get(CRServo.class, steer);
        encoder = hMap.get(AnalogInput.class, heading);

        driveMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        steerServo.setPower(0);

        // Initialize target to forward
        targetAngle = FORWARD_OFFSET;
    }

    public void run(SwerveModuleState requested) {

        // --- Read raw sensor and unwrap ---
        double raw = (encoder.getVoltage() / 3.3) * 180.0;
        double delta = raw - lastRaw;

        if (Math.abs(delta) > 90 && Math.abs(delta) < 150) {
            if (delta < 0) halfTurns++;
            else halfTurns--;
        }

        double totalAngle = halfTurns * 180 + raw;
        double angleDeg = ((totalAngle - FORWARD_OFFSET) % 360 + 360) % 360;
        lastRaw = raw;
        angleFiltered = angleDeg;

        // --- Update target only if wheel is moving ---
        if (Math.abs(requested.speedMetersPerSecond) > 0.001) {
            SwerveModuleState optimized = SwerveModuleState.optimize(
                    requested,
                    new Rotation2d(Math.toRadians(angleDeg))
            );
            targetAngle = -((optimized.angle.getDegrees() % 360) + 360) % 360;
        }

        // --- Compute shortest angular difference relative to sticky target ---
        double difference = ((targetAngle - angleDeg + 540) % 360) - 180;

        // --- Deadband near sideways angles ---
        boolean nearSideways = Math.abs(targetAngle - 90) < NEAR_SIDEWAYS
                || Math.abs(targetAngle - 270) < NEAR_SIDEWAYS;

        if (nearSideways && Math.abs(difference) < DEADZONE_ANGLE) {
            difference = 0; // ignore tiny errors
        }

        // --- Reduce gain near sideways angles ---
        double gain = P;
        if (nearSideways) gain *= REDUCED_GAIN_FACTOR;

        // --- Apply servo power ---
        steerServo.setPower(difference * gain);

        // --- Drive motor ---
        driveMotor.setVelocity(requested.speedMetersPerSecond / METERS_PER_TICK);
        currentVelocity = requested.speedMetersPerSecond;
    }

    // ==== GETTERS ====
    public double getAngle() {
        return angleFiltered;
    }

    public double getVelocity() {
        return currentVelocity;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                currentVelocity,
                Rotation2d.fromDegrees(angleFiltered)
        );
    }
}
