package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


@Autonomous(name="SigmaSixSeven")

public class SigmaSixSevenAuto extends LinearOpMode {

    private Pose2d startPos = new Pose2d(0, 0, Math.toRadians(90));

    @Override
    public void runOpMode() {
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, startPos);

        waitForStart();

        if (isStopRequested()) return;

        Action path = drivetrain.actionBuilder(startPos)
                        //.splineTo(new Vector2d(30, 30), Math.PI / 2)
                        //.splineTo(new Vector2d(0, 60), Math.PI)
                        .lineToX(30)
                        .build();

        Actions.runBlocking(new SequentialAction(path));
    }
}
