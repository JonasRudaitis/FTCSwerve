package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class KickerSubsytem extends SubsystemBase {
    final Servo kicker;
    final Servo kicker2;
    public KickerSubsytem (final HardwareMap hMap) {
        kicker = hMap.get(Servo.class, "kicker");
        kicker2 = hMap.get(Servo.class, "kicker2");

        in();
    }
    public void out() {
        kicker.setPosition(0);
        kicker2.setPosition(0.2);
    }
    public void in () {
        kicker.setPosition(0.2);
        kicker2.setPosition(0);
    }
}