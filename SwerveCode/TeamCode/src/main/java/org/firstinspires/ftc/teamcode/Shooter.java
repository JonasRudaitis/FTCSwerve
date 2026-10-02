package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    InterpLUT velocity = new InterpLUT(); // Distance: Velocity
    static HardwareMap hmap;
    static DcMotorEx leftShooter, rightShooter;
    Shooter () {
        // Add data to LUT
        velocity.add(-1.0, 0);
        velocity.add(0.0, 600);
        velocity.add(4.0, 600);
        velocity.add(4.5, 700);
        velocity.add(5.0, 800);
        velocity.add(5.5, 900);
        velocity.add(6.25, 1000);
        velocity.add(7.5, 1200);
        velocity.add(9.0, 1300);
        velocity.add(9.5, 1500);
        velocity.add(10.0, 1600);
        velocity.add(10.5, 1700);

        velocity.createLUT();
    }
    public void initialize(HardwareMap hardwareMap) {
        hmap = hardwareMap;
        leftShooter = hmap.get(DcMotorEx.class, "leftShooter");
        rightShooter = hmap.get(DcMotorEx.class, "rightShooter");

        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void shootFromDistance(double distance) {
        leftShooter.setVelocity(velocity.get(distance));
        rightShooter.setVelocity(velocity.get(distance));
    }
    public void setVelocity(double ticksPerSecond) {
        leftShooter.setVelocity(ticksPerSecond);
        rightShooter.setVelocity(ticksPerSecond);
    }
}
