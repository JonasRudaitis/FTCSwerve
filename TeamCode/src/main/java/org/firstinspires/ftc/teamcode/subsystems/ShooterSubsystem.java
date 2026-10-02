package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem extends SubsystemBase {
    private final Telemetry telemetry;
    private final DcMotorEx leftShooter, rightShooter;
    private int targetVelocity = 0;
    private final int tolerance = 67;
    public ShooterSubsystem(final HardwareMap hMap, final Telemetry telemetry){
        this.telemetry = telemetry;
        leftShooter = hMap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hMap.get(DcMotorEx.class, "rightShooter");


        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter.setDirection(DcMotorSimple.Direction.FORWARD);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(-40, 0, 0, 0));
        //rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(-40, 0, 0, 0));
    } // end of constructor


    @Override
    public void periodic () {
        telemetry.addData("left shooter", leftShooter.getVelocity());
        telemetry.addData("right shooter", rightShooter.getVelocity());
        telemetry.update();
        leftShooter.setVelocity(targetVelocity);
        rightShooter.setVelocity(targetVelocity);
    } // end of periodic
    public void setVelocity(int velocity) {
        targetVelocity = velocity;
    } // end of setVelocity
    public void stop() {
        targetVelocity = 0;
    } // end of stop

    public boolean isAtTarget() {
        double lowerLimit = targetVelocity - tolerance;
        double upperLimit = targetVelocity + tolerance;

        double leftVel = leftShooter.getVelocity();
        double rightVel = rightShooter.getVelocity();

        return leftVel >= lowerLimit && leftVel <= upperLimit
                && rightVel >= lowerLimit && rightVel <= upperLimit;
    }
} // end of ShooterSubsystem
