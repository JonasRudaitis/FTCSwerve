package org.firstinspires.ftc.teamcode.swerve;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class TempSwerveWheel {

    // Hardware
    private final DcMotorEx motor;
    private final Servo steerServo;

    // Internal steering angle (0–360)
    private double currentAngle = 0;

    // Servo constants
    private static final double SERVO_ZERO = 0.5;       // forward = 0°
    private static final double SERVO_RANGE_DEG = 300;  // physical travel
    private static final double SERVO_HALF = SERVO_RANGE_DEG / 2.0; // 150/2 = 75°

    // Drive constants
    private final double MOTOR_TICKS_PER_REV = 145.1; // GoBilda 1150 RPM motor
    private final double GEAR_RATIO = 2.0;           // 2:1 reduction (motor -> wheel)
    private final double WHEEL_CIRCUMFERENCE = 0.072 * Math.PI; // meters
    private final double METERS_PER_TICK = WHEEL_CIRCUMFERENCE / (MOTOR_TICKS_PER_REV * GEAR_RATIO);

    public TempSwerveWheel(HardwareMap hMap, String motorName, String steerName) {
        motor = hMap.get(DcMotorEx.class, motorName);

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Safe PIDF values
        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 3, 0, 0));

        steerServo = hMap.get(Servo.class, steerName);
        steerServo.setPosition(SERVO_ZERO);
    }

    private double norm360(double deg) {
        return ((deg % 360) + 360) % 360;
    }

    public void run(SwerveModuleState state) {

        double targetAngle = norm360(state.angle.getDegrees());
        double speed = state.speedMetersPerSecond;

        // Shortest path to angle
        double delta = targetAngle - currentAngle;
        delta = ((delta + 540) % 360) - 180;

        // Flip wheel if needed
        if (Math.abs(delta) > 90) {
            speed *= -1;
            targetAngle = norm360(targetAngle + 180);
            delta = targetAngle - currentAngle;
            delta = ((delta + 540) % 360) - 180;
        }

        currentAngle = norm360(currentAngle + delta);

        // Servo mapping
        double centeredAngle = ((currentAngle + 180) % 360) - 180;
        centeredAngle = Math.max(-SERVO_HALF, Math.min(SERVO_HALF, centeredAngle));
        double servoPos = SERVO_ZERO + (centeredAngle / SERVO_RANGE_DEG);
        servoPos = Math.max(0.0, Math.min(1.0, servoPos));

        if (Math.abs(speed) > 0.0001)
            steerServo.setPosition(servoPos);

        // Convert m/s -> ticks/sec (consider gearing)
        double ticksPerSec = speed / METERS_PER_TICK;
        motor.setVelocity(ticksPerSec);
    }

    public double getAngle() {
        return currentAngle;
    }

    public double getVelocity() {
        // Convert ticks/sec back to m/s
        return motor.getVelocity() * METERS_PER_TICK;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getVelocity(),
                new Rotation2d(Math.toRadians(currentAngle))
        );
    }
}
