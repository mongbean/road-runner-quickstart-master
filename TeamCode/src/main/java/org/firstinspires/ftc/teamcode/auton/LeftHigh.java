package org.firstinspires.ftc.teamcode.auton;
//36, 60
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;



@Autonomous(name="LeftHigh", group="Robot")
public class LeftHigh extends LinearOpMode{
    Robot robot;

    @Override
    public void runOpMode() {
        robot.initRobot();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(36, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        telemetry.addData("status", "init robot done");
        telemetry.update();
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{ //offset needs to be tested
                    robot.lslide.slideHigh();
                })
                .addTemporalMarker(0.8, ()->{
                    robot.claw.open();
                })
                .splineToLinearHeading(new Pose2d(60, 12, Math.toRadians(0)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->{
                    robot.lslide.slideLow();
                })
                .addTemporalMarker(3, ()->{
                    robot.claw.close();
                })
                .splineToLinearHeading(new Pose2d(36, 0, Math.toRadians(180)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->{
                    robot.lslide.slideHigh();
                })
                .addTemporalMarker(6, ()->{
                    robot.claw.open();
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->{
                    robot.lslide.slideLow();
                })
                .lineTo(new Vector2d(60, 12))
                .build();
        telemetry.addData("status", "traj buildt");
        telemetry.update();
        waitForStart();
        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
        }
        telemetry.addData("status", "traj done");
        telemetry.update();
    }
}
