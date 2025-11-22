package org.firstinspires.ftc.teamcode.badCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
@TeleOp(name="Mecanum Drive Example", group="Iterative Opmode")
public class OpMode_1 extends OpMode {

    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private DcMotor arm         = null;
    private DcMotor hand        = null;
    private DcMotor lift_bottom = null;
    private DcMotor lift_top    = null;
    
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
    }

    @Override
    public void loop() {

        // Mecanum drive is controlled with three axes: drive (front-and-back),
        // strafe (left-and-right), and twist (rotating the whole chassis).
        double drive   = gamepad1.right_stick_x;
        double strafe  = gamepad1.right_stick_y;
        double twist   = gamepad1.left_stick_x;
        double oneLY   = gamepad1.left_stick_y;
        double angle   = gamepad2.left_stick_y;
        double elevate = gamepad2.right_stick_y;

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
            (drive + strafe + twist),
            (drive - strafe - twist),
            (drive - strafe + twist),
            (drive + strafe - twist)
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
        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);
        arm.setPower(angle);
        //hand being elevate is TEMPORARY
        hand.setPower(-1 * elevate);
        
        
        // --- telemetry display ---
        // battery
            //telemetry.addData("battery", "%.1f volts", this::getBatteryVoltage;
        // motors
        telemetry.addData("front_left", speeds[0]);
        telemetry.addData("front_right", speeds[1]);
        telemetry.addData("back_left", speeds[2]);
        telemetry.addData("back_right", speeds[3]);
        telemetry.addData("arm", angle);
        telemetry.addData("hand", elevate);
        
        // controller data
          //telemetry.addData("Driver: ", "left(" + twist + ", " + oneLY + ") right(" + drive + ", " + strafe + ")");
          //telemetry.addData("Gripper: ", "left(" + twist + ", " + angle + ") right(" + drive + ", " + strafe + ")");
        // counters
        
    }
    
    // Copy rigght Aiden Tourtillott with no help from others with a computer
    // built in a cave
} 
*/