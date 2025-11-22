package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.util.Arrays;
import java.util.HashMap;

@TeleOp(name="Shooter Test")
public class ShooterTest extends LinearOpMode{
    private DcMotorEx shooterBeta = null;
    private DcMotorEx shooterAlpha = null;
    private DcMotor shooterSpinner = null;
    private DcMotor gluttonousRizzler = null;
    private double shooterSpeed = 1.0;
    public static double kP = 5.0; // Proportional gain
    public static double kI = 0.0;  // Integral gain
    public static double kD = 0.0;  // Derivative gain
    public static double kF = 30.0; // Feedforward gain - Start tuning this first!
    private double targetShooterVelocity = 0;
    private double MAX_VELOCITY_TICKS_PER_SEC = 2880;
    private double spinnerSpeed = 0.5;
    private boolean angleMode = false;
    private boolean canShoot = false;
    private boolean shooting = false;
    private boolean inCodeSequence = false;
    private double maxTimeBetweenInputs = 1000;
    double timeSinceLastInput = System.currentTimeMillis();
    private boolean[] sequence = new boolean[8];
    private HashMap<String, Integer> angles = new HashMap<>();

    @Override
    public void runOpMode(){
        shooterBeta = hardwareMap.get(DcMotorEx.class, "beta");
        shooterAlpha = hardwareMap.get(DcMotorEx.class, "alpha");
        shooterSpinner = hardwareMap.get(DcMotor.class, "spinny");
        gluttonousRizzler = hardwareMap.get(DcMotor.class, "rizzler");

        telemetry.addData("Components", "Acquired");
        telemetry.update();

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, kI, kD, kF);

        shooterSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterBeta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterAlpha.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterBeta.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterAlpha.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        angles.put("low", 100);
        angles.put("mid-low", 206);
        angles.put("mid-high", 389);
        angles.put("high", 538);

        waitForStart();

        while(opModeIsActive()){
            codeSequence();
            // Spinny speed up or down
            if(gamepad2.dpadUpWasPressed() && spinnerSpeed < 0.5){
                spinnerSpeed += 0.05;
            }
            if(gamepad2.dpadDownWasPressed() && spinnerSpeed > 0.05){
                spinnerSpeed -= 0.05;
            }
            // Enter sequence omde
            if(gamepad2.dpad_right){
                inCodeSequence = true;
            } else{
                inCodeSequence = false;
            }
            // Changes to angle mode if left dpad is held
            if(!inCodeSequence) {
                if (gamepad2.dpad_left) {
                    angleMode = true;
                } else {
                    angleMode = false;
                }
            }
            // Angle mode
            if(angleMode && !inCodeSequence){
                if (gamepad2.a) {
                    shooterSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shooterSpinner.setTargetPosition(angles.get("low"));
                    shooterSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shooterSpinner.setPower(spinnerSpeed);
                }
                if (gamepad2.x) {
                    shooterSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shooterSpinner.setTargetPosition(angles.get("mid-low"));
                    shooterSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shooterSpinner.setPower(spinnerSpeed);
                }
                if (gamepad2.y) {
                    shooterSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shooterSpinner.setTargetPosition(angles.get("mid-high"));
                    shooterSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shooterSpinner.setPower(spinnerSpeed);
                }
                if (gamepad2.b) {
                    shooterSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    shooterSpinner.setTargetPosition(angles.get("high"));
                    shooterSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shooterSpinner.setPower(spinnerSpeed);
                }
            } else if (!inCodeSequence) {
                if (gamepad2.a) {
                    shooterSpeed = .25;
                }
                if (gamepad2.x) {
                    shooterSpeed = .5;
                }
                if (gamepad2.y) {
                    shooterSpeed = .75;
                }
                if (gamepad2.b) {
                    shooterSpeed = 1;
                }
            }
            // Intakes
            if(!inCodeSequence) {
                if (gamepad2.right_stick_y > 0) {
                    gluttonousRizzler.setPower(-1);
                } else if (gamepad2.right_stick_y < 0) {
                    gluttonousRizzler.setPower(1);
                } else {
                    gluttonousRizzler.setPower(0);
                }
            }
            // Shoot
            if(!inCodeSequence){
                if (gamepad2.right_trigger > 0) {
                    shooting = true;
                    targetShooterVelocity = MAX_VELOCITY_TICKS_PER_SEC * shooterSpeed;
                } else if (gamepad2.left_trigger > 0) {
                    shooting = true;
                    targetShooterVelocity = -MAX_VELOCITY_TICKS_PER_SEC * shooterSpeed;
                } else {
                    if(shooting){
                        shooting = false;
                        canShoot = false;
                    }
                    targetShooterVelocity = 0;
                }
            }
            // Spinngy
            if(!inCodeSequence){
                if(gamepad2.right_bumper){
                    shooterSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    shooterSpinner.setPower(spinnerSpeed);
                } else if (gamepad2.left_bumper){
                    shooterSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    shooterSpinner.setPower(-spinnerSpeed);
                } else{
                    shooterSpinner.setPower(0);
                }
            }
            //Shooter go
            shooterBeta.setVelocity(targetShooterVelocity);
            shooterAlpha.setVelocity(targetShooterVelocity);

            double alphaCurrentVel = shooterAlpha.getVelocity();
            double betaCurrentVel = shooterBeta.getVelocity();

            telemetry.addData("Spinner Speed", spinnerSpeed);
            telemetry.addData("Spinner Position", shooterSpinner.getCurrentPosition());
            telemetry.addData("Spinner Target Position", shooterSpinner.getTargetPosition());
            telemetry.addData("In Angle Mode", angleMode);

            //telemetry.addData("Target Velocity", targetShooterVelocity);
            //telemetry.addData("Alpha Current Velocity", alphaCurrentVel);
            //telemetry.addData("Beta Current Velocity", betaCurrentVel);
            //telemetry.addData("Alpha Error", targetShooterVelocity - alphaCurrentVel);
            //telemetry.addData("Beta Error", targetShooterVelocity - betaCurrentVel);

            telemetry.update();
        }
    }

    private void codeSequence(){
        if(inCodeSequence) {
            if(isTimeOver(timeSinceLastInput, maxTimeBetweenInputs)){
                Arrays.fill(sequence, false);
            }
            if (gamepad2.x && !(sequence[0])) {
                timeSinceLastInput = System.currentTimeMillis();
                sequence[0] = true;
            }
            if (gamepad2.b && sequence[0]) {
                timeSinceLastInput = System.currentTimeMillis();
                sequence[1] = true;
            }
            if (gamepad2.b && sequence[1]){
                timeSinceLastInput = System.currentTimeMillis();
                sequence[2] = true;
            }
            if (gamepad2.left_bumper && sequence[2]) {
                timeSinceLastInput = System.currentTimeMillis();
                sequence[3] = true;
            }
            if (gamepad2.a && sequence[3]) {
                timeSinceLastInput = System.currentTimeMillis();
                sequence[4] = true;
            }
            if (gamepad2.right_bumper && sequence[4]) {
                timeSinceLastInput = System.currentTimeMillis();
                sequence[5] = true;
            }
            if (gamepad2.left_bumper && sequence[5]){
                timeSinceLastInput = System.currentTimeMillis();
                sequence[6] = true;
            }
            if (gamepad2.y && sequence[6]) {
                timeSinceLastInput = System.currentTimeMillis();
                sequence[7] = true;
            }
            if (gamepad2.x && sequence[7]) {
                timeSinceLastInput = System.currentTimeMillis();
                sequence[8] = true;
            }
        }

        if(areAllTrue(sequence)){
            canShoot = true;
            Arrays.fill(sequence, false);
        }

        for(int i = 0; i < sequence.length; i++) {
            telemetry.addData("Sequence " + i, sequence[i]);
        }
    }

    private static boolean isTimeOver(double time, double timer){
        return System.currentTimeMillis() - time > timer;
    }
    public static boolean areAllTrue(boolean[] arr) {
        for (boolean b : arr) { // Enhanced for-each loop
            if (!b) { // If any element is false, return false immediately
                return false;
            }
        }
        return true; // If the loop completes, all elements were true
    }
}
