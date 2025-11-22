package org.firstinspires.ftc.teamcode.badCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
/*
@Autonomous(name="BaseAutoShortRight", group="Linear Opmode")
public class BaseAutoShortRight extends LinearOpMode{
    private Blinker control_Hub;
    private Blinker expansion_Hub_1;
    private DcMotor arm;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor hand;
    private Gyroscope imu;
    private DcMotor lift_bottom;
    private DcMotor lift_top;
    private Servo servoBottomLeft;
    private Servo servoBottomRight;
    private Servo servoLeft;
    private Servo servoRight;

    // todo: write your code here
    public void runOpMode() throws InterruptedException {

        // Gets left and right motors from the configuration file
        // motorLeft = hardwareMap.dcMotor.get("motorLeft");
        // motorRight = hardwareMap.dcMotor.get("motorRight");
        // claw = hardwareMap.servo.get("servo_1");
        // motorArm = hardwareMap.dcMotor.get("motorArm");
        waitForStart();
        front_left   = hardwareMap.get(DcMotor.class, "front_left");
        front_right  = hardwareMap.get(DcMotor.class, "front_right");
        back_left    = hardwareMap.get(DcMotor.class, "back_left");
        back_right   = hardwareMap.get(DcMotor.class, "back_right");
        servoBottomLeft   = hardwareMap.get(Servo.class, "servoBottomLeft");
        servoBottomRight   = hardwareMap.get(Servo.class, "servoBottomRight");
        hand    = hardwareMap.get(DcMotor.class, "hand");
        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);
        
//-------------------------------------------------------------------------BEGIN AUTO--------------------------------------//
        
        //servo close
        servoBottomLeft.setPosition(0.9);
        servoBottomRight.setPosition(0.3);
        sleep(1000);

        //strafe right maybe
        front_left.setPower(0.5);
        front_right.setPower(-0.5);
        back_left.setPower(-0.5);
        back_right.setPower(0.5);
        sleep(835);

        //stop
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
}
*/