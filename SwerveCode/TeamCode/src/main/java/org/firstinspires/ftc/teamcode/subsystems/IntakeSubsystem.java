package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx intake;

    public IntakeSubsystem (final HardwareMap hMap) {
        intake = hMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void on(){
        intake.setPower(1);
    }
    public void on(double power){
        intake.setPower(power);
    }
    public void off(){
        intake.setPower(0);
    }
}
