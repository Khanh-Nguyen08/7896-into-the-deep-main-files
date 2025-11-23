package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="OtherSigmaAuto")
public class AltSigmaAuto extends LinearOpMode {

    private Pose2d startPos = new Pose2d(0, 0, Math.toRadians(90));

    @Override
    public void runOpMode() {
        MecanumDrive drivetrain = new MecanumDrive(hardwareMap, startPos);

        waitForStart();

        if (isStopRequested()) return;

        drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(-20, 0), 0));
        //drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(20, 0), 0));
    }
}
