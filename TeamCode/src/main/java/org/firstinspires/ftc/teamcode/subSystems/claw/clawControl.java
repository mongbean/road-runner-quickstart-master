package org.firstinspires.ftc.teamcode.subSystems.claw;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

public class clawControl {
    private LinearOpMode myOpMode = null;
    public double pos;
    public static final double openPos = 0.38888888888888888888;
    public static final double closePos = 0;
    Servo cServo;
    public void clawInit() {
        cServo = myOpMode.hardwareMap.get(Servo.class, "clawwwwwwwwwwwwwwwwwwww");
        myOpMode.telemetry.addData(">", "Servo Initialized" );
        myOpMode.telemetry.update();
    }
    public void open() {
        cServo.setPosition(openPos);
    }
    public void close() {
        cServo.setPosition(closePos);
    }
    public void toggle() {
        pos = cServo.getPosition();
        if (Math.abs(pos-openPos)<0.05){
            cServo.setPosition(closePos);
        }
        else if (Math.abs(pos-closePos)<0.05){
            cServo.setPosition(openPos);
        }
    }
}
