package org.firstinspires.ftc.teamcode.swerve;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class SwerveWheel {

    public static double P = 0.007;
    public static double FORWARD_OFFSET = 90;
    public static double TICKS_PER_REV = 145.1 * 2;
    public static double WHEEL_CIRC = 0.072 * Math.PI;
    public static double METERS_PER_TICK = WHEEL_CIRC / TICKS_PER_REV;

    // === ACCELERATION VALUE (m/s^2) ===
    public static double MAX_ACCEL = 8.0;     // increase if you want snappier accel

    private final DcMotorEx driveMotor;
    private final CRServo steerServo;
    private final AnalogInput encoder;

    private double lastRaw = 90;
    private int halfTurns = 0;
    private double angle = 90;

    private double targetAngle = 0;

    // internal velocity used for ramping
    private double currentVelocityMPS = 0;

    long lastTime = System.nanoTime();

    public SwerveWheel(HardwareMap hMap, String motor, String steer, String heading) {
        driveMotor = hMap.get(DcMotorEx.class, motor);
        steerServo = hMap.get(CRServo.class, steer);
        encoder = hMap.get(AnalogInput.class, heading);

        driveMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients old = driveMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        double limit = 0.6;   // max 60% voltage
        double newF = old.f * limit;

        driveMotor.setVelocityPIDFCoefficients(old.p, old.i, old.d, newF);


        steerServo.setDirection(DcMotorSimple.Direction.REVERSE);
        steerServo.setPower(0);

        targetAngle = FORWARD_OFFSET;
    }

    public void run(SwerveModuleState state) {

        // ---------------- ENCODER ANGLE WRAP ----------------
        double rawAngle = (encoder.getVoltage()/3.3) * 180.0;
        double delta = rawAngle - lastRaw;

        if (Math.abs(delta) > 90) {
            halfTurns++;
        }

        angle = -((rawAngle + (halfTurns * 180)) % 360) - 90;
        lastRaw = rawAngle;


        // ---------------- OPTIMIZE STATE ----------------
        SwerveModuleState optimized = SwerveModuleState.optimize(
                state,
                new Rotation2d(Math.toRadians(angle))
        );

        if (Math.abs(state.speedMetersPerSecond) > 0.1)
            targetAngle = optimized.angle.getDegrees();


        // ---------------- STEERING ----------------
        double difference = ((targetAngle - angle + 540) % 360) - 180;
        steerServo.setPower(difference * P);


        // ---------------- ACCELERATION LIMITING (NO BRAKING LIMIT) ----------------
        double desiredVel = optimized.speedMetersPerSecond;

        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        double maxDelta = MAX_ACCEL * dt;
        double deltaV = desiredVel - currentVelocityMPS;

        if (deltaV > 0) {
            // accelerating forward -> limit
            currentVelocityMPS += Math.min(deltaV, maxDelta);
        } else {
            // braking or reversing -> no limit, instantly apply
            currentVelocityMPS = desiredVel;
        }

// convert m/s → ticks/s
        double ticksPerSecond = currentVelocityMPS / METERS_PER_TICK;

        //driveMotor.setVelocity(Math.acos(Math.toRadians(difference)) * ticksPerSecond);
        driveMotor.setVelocity((Math.acos(Math.toRadians(difference)) * desiredVel) / METERS_PER_TICK);
    }


    public double getVelocity() {
        return driveMotor.getVelocity() * METERS_PER_TICK;
    }

    public double getAngle() {
        return angle;
    }

    public void flip(){
        halfTurns++;
        targetAngle = 180;
    }

    public void forward() {
        driveMotor.setVelocity(0);
        targetAngle = 180;
    }

    public void back() {
        targetAngle = 0;
    }
}
