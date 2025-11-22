package org.firstinspires.ftc.teamcode.badCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @author Brandon Gong
 */
 
/**
 * ---TO DO---
 * Rounding function
 * Test current driving
 * Telemetry data
 * Configure the four new motors (once they are all there)
 * Program those motors
 * Servo programming
 * Test if we want buttons or joystick for the "hand"
 * More controls (speed dampener, fixed motor movements, 180 spin for specimen stuff, I think that's it?)
 * Gambling minigame
 * Backup artillery
 */
/*
@TeleOp(name="Moode", group="Iterative Opmode")
public class RealOpMode extends OpMode {

    // Motor and Servo positions are based on looking at the robot from the "back," or the linear slide side.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private DcMotor arm         = null;
    private DcMotor hand        = null;
    private DcMotor lift_bottom = null;
    private DcMotor lift_top    = null;
    private Servo servoLeft     = null;
    private Servo servoRight    = null;
    private Servo servoBottomLeft = null;
    private Servo servoBottomRight = null;
    private double boost = 0.5;
    private int servonum = 0;
    private boolean servoToggle = false;
    
    static double shorten(double rounded) {
        rounded *= 1000;
        rounded = Math.round(rounded);
        rounded /= 1000.0;
        return rounded;
    }
    @Override
    public void init() {

        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left   = hardwareMap.get(DcMotor.class, "front_left");
        front_right  = hardwareMap.get(DcMotor.class, "front_right");
        back_left    = hardwareMap.get(DcMotor.class, "back_left");
        back_right   = hardwareMap.get(DcMotor.class, "back_right");
        //Configure the four other motors (arm, hand, lift_bottom, lift_top)
        arm          = hardwareMap.get(DcMotor.class, "arm");
        hand         = hardwareMap.get(DcMotor.class, "hand");
        lift_bottom  = hardwareMap.get(DcMotor.class, "lift_bottom");
        DcMotor lift_bottom = hardwareMap.dcMotor.get("lift_bottom");
        lift_bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_bottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift_top  = hardwareMap.get(DcMotor.class, "lift_top");
        DcMotor lift_top = hardwareMap.dcMotor.get("lift_top");
        lift_top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_top.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        servoLeft    = hardwareMap.get(Servo.class, "servoLeft");
        servoRight   = hardwareMap.get(Servo.class, "servoRight");
        servoBottomLeft   = hardwareMap.get(Servo.class, "servoBottomLeft");
        servoBottomRight   = hardwareMap.get(Servo.class, "servoBottomRight");
        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        
        
    }

    @Override
    public void loop() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive   = gamepad1.left_trigger - gamepad1.right_trigger;
        double strafe  = gamepad1.left_stick_x;
        double twist   = -1 * gamepad1.right_stick_x;
        //double oneLY   = gamepad1.left_stick_y;
        double angle   = -1 * gamepad2.left_stick_y;
        double hands   = -1 * gamepad2.right_stick_y;
        double elevate = gamepad2.left_trigger - gamepad2.right_trigger;
        boolean liftUp = gamepad2.left_trigger > 0.1;
        boolean liftDown = gamepad2.right_trigger > 0.1;
        boolean serve  = gamepad2.y;
        double lbpos = lift_bottom.getCurrentPosition();
        double ltpos = lift_top.getCurrentPosition();
    



        

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
            (boost * (drive + strafe + twist)),
            (-1 * boost * (drive - strafe - twist)),
            (boost * (drive - strafe + twist)),
            (-1 * boost * (drive + strafe - twist))
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        if (gamepad2.a) {
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);
        } else {
            front_left.setPower(speeds[0]);
            front_right.setPower(speeds[1]);
            back_left.setPower(speeds[2]);
            back_right.setPower(speeds[3]);
        }
        if (gamepad1.x) {
            boost = 0.15;
            //boost extra parameter lbpos > -2500
        } else if (gamepad1.b) {
            boost = 0.15;
        } else {
            boost = 0.5;
        }
        arm.setPower(0.7 * angle);
        //hand being elevate is TEMPORARY
        hand.setPower(-1 * hands);
    
        //nvm I fixed it 
        lift_bottom.setPower(elevate);
        lift_top.setPower(elevate);
        // if (gamepad2.right_bumper) {
        //     lift_top.setPower(0.5);
        // }
        // if (gamepad2.left_bumper) {
        //     lift_top.setPower(-0.5);
        //}
        /*if (liftUp) {
            lift_bottom.setTargetPosition(-5500);
            lift_bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            lift_bottom.setTargetPosition(-500);
            lift_bottom.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }*/
        /*
        //servos
        if (serve != servoToggle) {
            servoToggle = !servoToggle;
            servonum ++;
        }
        
        //open
        if (servonum % 2 == 0) {
            servoLeft.setPosition(0);
            servoRight.setPosition(1);
            
        //close
        } else {
            servoLeft.setPosition(0.48);
            servoRight.setPosition(0.44);
        }
        //close bottom
        if (gamepad2.a) {
            servoBottomLeft.setPosition(0.9);
            servoBottomRight.setPosition(0.3);
        } else {
            servoBottomLeft.setPosition(0);
            servoBottomRight.setPosition(0.6);
        }
        // --- telemetry display ---
        // battery
            //telemetry.addData("battery", "%.1f volts", this::getBatteryVoltage;
        // motors
        /*.addData("front_left", speeds[0]);
        telemetry.addData("front_right", speeds[1]);
        telemetry.addData("back_left", speeds[2]);
        telemetry.addData("back_right", speeds[3]);
        telemetry.addData("arm", angle);
        telemetry.addData("hand", hands);
        telemetry.addData("drive", drive);
        telemetry.addData("strafe", strafe);
        telemetry.addData("twist", twist);
        // telemetry.addData("serve", serve);
        // telemetry.addData("servoToggle", servoToggle);
        // telemetry.addData("servonum", lbpos);
        // telemetry.addData("lbpos", lbpos);
        //telemetry.addData("gamepad1", shorten(drive) + " " + shorten(strafe) + " " + shorten(twist));
        // telemetry.addData("lbpos", lbpos);
        // telemetry.addData("ltpos", ltpos);
        // telemetry.addData("liftUp", liftUp);
        // telemetry.addData("liftDown", liftDown);
        // telemetry.addData("boostable", lbpos > -2500);
        // controller data
          //telemetry.addData("Driver: ", "left(" + twist + ", " + oneLY + ") right(" + drive + ", " + strafe + ")");
          //telemetry.addData("Gripper: ", "left(" + twist + ", " + angle + ") right(" + drive + ", " + strafe + ")");
        // counters
        
    }
    
    // Copy rigght Aiden Tourtillott with no help from others with a computer
    // built in a cave
    // ACTUALLY Coded by Caden Hamilton right outside of the cave :3 x3 owo >w< *nuzzles you* >w> uwu
} 
//:) from Will

// System.out.println("Hello world")
// Scanner robot = new Scanner(System.in); from Misha

// System.out.println("i am incredibly small") from Sam
*/