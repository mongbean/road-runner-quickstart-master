/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.subSystems.slides;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */

public class LinearSlidePID {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor armMotor1 = null;
    private DcMotor armMotor2 = null;


    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double Kp = 1;
    public static final double Ki = 1;
    public static final double Kd = 1;
    public static final double Kg = 1;
    public static final double epsilon = 0.01;

    public static double integral = 0;
    public double error;
    public double lastError=0;
    public double derivative;
    public double out;
    public ElapsedTime timer = new ElapsedTime();

    public void slideInit()    {
        // ACTUAL NAMES
        armMotor1  = myOpMode.hardwareMap.get(DcMotor.class, "dljfladfadjf");
        armMotor2 = myOpMode.hardwareMap.get(DcMotor.class, "drasdfadf");
        armMotor1.setDirection(DcMotor.Direction.REVERSE);
        armMotor2.setDirection(DcMotor.Direction.FORWARD);
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        timer.reset();

        myOpMode.telemetry.addData(">", "Slide Initialized");
        myOpMode.telemetry.update();

    }

    public void slidePID(double height) {
        error = height - armMotor1.getCurrentPosition();

        // rate of change of the error
        derivative = (error - lastError) / timer.milliseconds();

        // sum of all error over time
        integral = integral + (error * timer.milliseconds());

        out = (Kp * error) + (Ki * integral) + (Kd * derivative) + Kg;

        armMotor1.setPower(out);
        armMotor2.setPower(out);

        error = height - armMotor1.getCurrentPosition();
        timer.reset();

    }
    public void slideGround(){
        while (Math.abs(armMotor1.getCurrentPosition()) > epsilon) {
            slidePID(0);
        }
    }
    public void slideLow(){
        while (Math.abs(armMotor1.getCurrentPosition()-200) > epsilon){
            slidePID(200);
        }
    }
    public void slideMid(){
        while (Math.abs(armMotor1.getCurrentPosition()-300) > epsilon){
            slidePID(300);
        }
    }
    public void slideHigh(){
        while (Math.abs(armMotor1.getCurrentPosition()-400) > epsilon){
            slidePID(400);
        }
    }

}
