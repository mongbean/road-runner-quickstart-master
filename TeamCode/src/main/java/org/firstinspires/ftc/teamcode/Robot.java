package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subSystems.claw.clawControl;
import org.firstinspires.ftc.teamcode.subSystems.slides.LinearSlidePID;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot {
    private LinearOpMode myOpMode = null;
    public clawControl claw = new clawControl();
    public LinearSlidePID lslide = new LinearSlidePID();

    public void initRobot() {
        claw.clawInit();
        lslide.slideInit();
    }
}
