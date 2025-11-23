/*
Copyright 2025 FIRST Tech Challenge Team 7896

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

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

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp(name="Aura Goober", group="ATestes")

public class goober extends LinearOpMode {
    // Wheel Motors
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    // Shooters
    private DcMotorEx shooterBeta = null;
    private DcMotorEx shooterAlpha = null;
    // Intake
    private DcMotor gluttonousRizzler = null;
    private DcMotor shooterSpinner = null;
    private Servo cadenHamilton = null;
    private Servo aidenTourtillott = null;
    private Servo chuckleSemicolonFuck5 = null;
    // odometry
    // Spinny
    //private DcMotor ballJuggler = null;
    // Outtake gate opener servo
    //private Servo bouncer = null;

    private double boost = 0.5;
    private double prevBoost = boost;
    private double shooterSpeed = 1.0;
    //private double alphaSpeed = 1.0;
    //private double shooterTicksPerRev;
    public static double kP = 2.0; // Proportional gain
    public static double kI = 0.0;  // Integral gain
    public static double kD = 0.0;  // Derivative gain
    public static double kF = 30.0; // Feedforward gain - Start tuning this first!
    private double targetShooterVelocity = 0;
    private double MAX_VELOCITY_TICKS_PER_SEC = 2880;
    private double targetMovementVelocity = 0;
    private double MAX_VELOCITY_TICKS_MOVEMENT = 1000;
    private double spinnerSpeed = 0.5;
    private boolean angleMode = false;
    private boolean inHoldPos = false;
    private boolean canShoot = true;
    private boolean canIntake = true;
    private boolean intaking = false;
    private boolean canPushy = true;
    private boolean shooting = false;
    private boolean inCodeSequence = false;
    private double maxTimeBetweenInputs = 1000;
    private double timeSinceLastInput = System.currentTimeMillis();
    private boolean inSecureMode = false;
    private boolean[] sequence = new boolean[5];
    private int lowAnglePos = -204;
    private int highAnglePos = -384;
    private double startTime = 0.0;
    private int changeRate = 25;
    private int spinnerUpperLimit = -454;
    private boolean blocking = false;
    private boolean gamepad2xIsGood = true;


    //private boolean inLoadingMode = false;

    //private Chamber xChamber = new Chamber(false, -1, 6, 7);
    //private Chamber yChamber = new Chamber(false, -1, 6, 7);
    //private Chamber bChamber = new Chamber(false, -1, 6, 7);
    //private Chamber currentOuttakeChamber = null;
    //private Chamber currentIntakeChamber = null;
    //private Chamber[] chambers = {xChamber, yChamber, bChamber};

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        shooterBeta = hardwareMap.get(DcMotorEx.class, "beta");
        shooterAlpha = hardwareMap.get(DcMotorEx.class, "alpha");
        gluttonousRizzler = hardwareMap.get(DcMotor.class, "rizzler");
        shooterSpinner = hardwareMap.get(DcMotor.class, "spinny");
        cadenHamilton = hardwareMap.get(Servo.class, "caden");
        aidenTourtillott = hardwareMap.get(Servo.class, "aiden");
        chuckleSemicolonFuck5 = hardwareMap.get(Servo.class, "fuck5");
        //ballJuggler = hardwareMap.get(DcMotor.class, "juggler");

        //bouncer = hardwareMap.get(Servo.class, "bouncer");

        telemetry.addData("Components", "Acquired");
        telemetry.update();

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, kI, kD, kF);

        shooterBeta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterAlpha.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterBeta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterAlpha.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterSpinner.setTargetPosition(highAnglePos);
        shooterSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooterSpinner.setPower(0.25);

        shooterBeta.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterAlpha.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterBeta.setDirection(DcMotorSimple.Direction.REVERSE);


        cadenHamilton.setPosition(0);
        aidenTourtillott.setPosition(0);
        chuckleSemicolonFuck5.setPosition(0);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){
            movementHandler();
            //boostHandler();
            //bouncerHandler();
            powerDuoHandler();
            //jugglerHandler();

            //odo.update();
            //Pose2d pose = odo.getPose();
            //telemetry.addData("X", pose.position.x);
            //telemetry.addData("Y", pose.position.y);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }


    private void movementHandler(){
        double drive  = -gamepad1.left_stick_x;
        double strafe = gamepad1.left_stick_y;
        double twist  = -gamepad1.right_stick_x;

        double[] speeds = {
                2 * (drive + strafe - twist),
                2 * (drive - strafe - twist),
                2 * (drive - strafe + twist),
                2 * (drive + strafe + twist)
        };

        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if(max < Math.abs(speeds[i])){
                max = Math.abs(speeds[i]);
            }
        }

        if(max > 1){
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        frontLeft.setPower(speeds[0]);
        frontRight.setPower(speeds[1]);
        backLeft.setPower(speeds[2]);
        backRight.setPower(speeds[3]);
    }

    private void velocityMovementHandler(){
        double drive  = gamepad1.left_stick_x;
        double strafe = -gamepad1.left_stick_y;
        double twist  = gamepad1.right_stick_x;

        double[] speeds = {
                boost * (drive + strafe + twist),
                boost * (drive - strafe - twist),
                boost * (drive - strafe + twist),
                boost * (drive + strafe - twist)
        };

        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if(max < Math.abs(speeds[i])){
                max = Math.abs(speeds[i]);
            }
        }

        if(max > 1){
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        frontLeft.setPower(-speeds[0]);
        frontRight.setPower(speeds[1]);
        backLeft.setPower(-speeds[2]);
        backRight.setPower(speeds[3]);
    }

    private void boostHandler(){
        // Increase boost strength
        if(gamepad1.dpadUpWasPressed()){
            if(boost < 0.75){
                prevBoost += 0.05;
            }
        }
        // Decrease boost strength
        if(gamepad1.dpadDownWasPressed()){
            if(boost > 0.05){
                prevBoost -= 0.05;
            }
        }
        // Store previous boost amount
        if(gamepad1.bWasPressed()){
            //prevBoost = boost;
        }
        // Set previous boost amount
        if(gamepad1.bWasReleased()){
            //boost = prevBoost;
        }
        // Set boost to max strength
        if(gamepad1.b){
            boost = 1.0;
        }else{
            boost = prevBoost;
        }
    }

    private void bouncerHandler(){
        /* TODO: REQUIRES ATTENTIONS????????? IDFK
        //
        //
        //
        //
        //
        if(gamepad2.dpadRightWasPressed()){
            if(frontdoorBouncer.getPosition() == frontdoorBouncer.MIN_POSITION){
                frontdoorBouncer.setPosition(frontdoorBouncer.MAX_POSITION);
            }else if(frontdoorBouncer.getPosition() == frontdoorBouncer.MAX_POSITION){
                frontdoorBouncer.setPosition(frontdoorBouncer.MIN_POSITION);
            }
        }
        
        if(gamepad2.dpadLeftWasPressed()){
            if(backdoorBouncer.getPosition() == backdoorBouncer.MIN_POSITION){
                backdoorBouncer.setPosition(backdoorBouncer.MAX_POSITION);
            }else if(backdoorBouncer.getPosition() == backdoorBouncer.MAX_POSITION){
                backdoorBouncer.setPosition(backdoorBouncer.MIN_POSITION);
            }
        }
        */
        //Telemetry.addData("Touching", "Myeslf");
    }

    private void powerDuoHandler() {
        codeSequence();
        // Spinny speed up or down
//        if(gamepad2.dpadUpWasPressed() && spinnerSpeed < 0.5){
//            spinnerSpeed += 0.05;
//        }
//        if(gamepad2.dpadDownWasPressed() && spinnerSpeed > 0.05){
//            spinnerSpeed -= 0.05;
//        }
        if (!inCodeSequence && canIntake) {
            if (gamepad2.left_trigger > 0) {
                gluttonousRizzler.setPower(1);
                canPushy = false;
                intaking = true;
            } else if (gamepad2.left_bumper) {
                gluttonousRizzler.setPower(-1);
                canPushy = false;
                intaking = false;
            } else {
                gluttonousRizzler.setPower(0);
                canPushy = true;
                intaking = false;
            }
        }
        if (gamepad2.x && gamepad2xIsGood){
            gamepad2xIsGood = false;
            cadenHamilton.setPosition(0);
            aidenTourtillott.setPosition(0);
            if (!blocking){
                chuckleSemicolonFuck5.setPosition(1);
                blocking = true;
                inHoldPos = false;
            } else {
                chuckleSemicolonFuck5.setPosition(0);
                blocking = false;
            }
        }
        if (!gamepad2.x){
            gamepad2xIsGood = true;
        }
        // Enter sequence omde
        if (gamepad2.dpad_right) {
            inCodeSequence = true;
        } else {
            inCodeSequence = false;
        }
//        // Changes to angle mode if left dpad is held
//        if(!inCodeSequence) {
//            if (gamepad2.dpad_left) {
//                angleMode = true;
//            } else {
//                angleMode = false;
//            }
//        }


        // Angle mode
        if (!inCodeSequence) {
            if (gamepad2.right_bumper) {
                shooterSpinner.setTargetPosition(lowAnglePos);
                shooterSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shooterSpinner.setPower(0.25);
            }
            // if (gamepad2.right_bumper) {
            //     shooterSpinner.setTargetPosition(highAnglePos);
            //     shooterSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //     shooterSpinner.setPower(0.25);
            // }

            if (gamepad2.dpad_up){
                shooterSpinner.setTargetPosition(shooterSpinner.getCurrentPosition() - changeRate);
                shooterSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shooterSpinner.setPower(0.25);
            }
            if (gamepad2.dpad_down){
                shooterSpinner.setTargetPosition(shooterSpinner.getCurrentPosition() + changeRate);
                shooterSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                shooterSpinner.setPower(0.25);
            }
            if (shooterSpinner.getTargetPosition() < spinnerUpperLimit){
                shooterSpinner.setTargetPosition(spinnerUpperLimit);
            }
            if (shooterSpinner.getTargetPosition() > 0){
                shooterSpinner.setTargetPosition(0);
            }
        }


        // Intakes
//        if(!inCodeSequence && canIntake) {
//            if (gamepad2.left_trigger > 0){
//                inSecureMode = false;
//                gluttonousRizzler.setPower(1);
//            } else if(!inSecureMode){
//                gluttonousRizzler.setPower(0);
//            }
//        }
        // Keeps the gluttonous rizzler slowly spinning (dont use)
//        if (!inCodeSequence) {
//            if (gamepad2.bWasPressed()) {
//                inSecureMode = !inSecureMode;
//            }
//            if (inSecureMode) {
//                gluttonousRizzler.setPower(0.1);
//            }
//        }

        // Servo pushers
        if (!inCodeSequence && canPushy && !intaking) {
            if (gamepad2.b && !blocking) {
                canIntake = false;
                aidenTourtillott.setPosition(1);
                if (startTime == 0.0) {
                    startTime = System.currentTimeMillis();
                }
                if (isTimeOver(startTime, 500)) {
                    cadenHamilton.setPosition(1);
                }
            } else if (gamepad2.a && !blocking) {
                canIntake = false;
                inHoldPos = false;
                cadenHamilton.setPosition(1);
            } else if (inHoldPos) {
                canIntake = false;
            } else {
                canIntake = true;
                cadenHamilton.setPosition(0);
                aidenTourtillott.setPosition(0);
                startTime = 0.0;
            }
        }
        // Servo hold pos
        if (!inCodeSequence) {
            if (gamepad2.yWasPressed() && !intaking && !blocking) {
                inHoldPos = !inHoldPos;
            }
            if (inHoldPos) {
                cadenHamilton.setPosition(0.5);
            }
        }
        // Shoot BRBRBRBBRBRBRBRRB
        if (canShoot) {
            if (gamepad2.right_trigger > 0){
                shooting = true;
                targetShooterVelocity = MAX_VELOCITY_TICKS_PER_SEC;
            } else {
                if (shooting) {
                    shooting = false;
                }
                targetShooterVelocity = 0;
            }
        }
        // // SHoot but wfor loseress LOLOLOLOL
        // if (!inCodeSequence) {
        //     if (gamepad2.left_stick_y > 0) {
        //k_y > 0) {
        //         shooting = true;
        //         targetShooterVelocity = MAX_VELOCITY_TICKS_PER_SEC;
        //     } else {
        //         if (shooting) {
        //             shooting = false;
        //             canShoot = false;
        //         }
        //         targetShooterVelocity = 0;
        //     }
        // }

//        // Spinngy
//        if(!inCodeSequence){
//            if(gamepad2.right_bumper){
//                shooterSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                shooterSpinner.setPower(spinnerSpeed);
//            } else if (gamepad2.left_bumper){
//                shooterSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                shooterSpinner.setPower(-spinnerSpeed);
//            } else{
//                shooterSpinner.setPower(0);
//            }
//        }

        //Shooter go
        shooterBeta.setVelocity(targetShooterVelocity);
        shooterAlpha.setVelocity(targetShooterVelocity);

        telemetry.addData("Servo Position", cadenHamilton.getPosition());

        telemetry.addData("Spinner Speed", spinnerSpeed);
        telemetry.addData("Spinner Position", shooterSpinner.getCurrentPosition());
        telemetry.addData("Spinner Target Position", shooterSpinner.getTargetPosition());

        double alphaCurrentVel = shooterAlpha.getVelocity();
        double betaCurrentVel = shooterBeta.getVelocity();

        telemetry.addData("Target Velocity", targetShooterVelocity);
        telemetry.addData("Alpha Current Velocity", alphaCurrentVel);
        telemetry.addData("Beta Current Velocity", betaCurrentVel);
        telemetry.addData("Alpha Error", targetShooterVelocity - alphaCurrentVel);
        telemetry.addData("Beta Error", targetShooterVelocity - betaCurrentVel);
        telemetry.addData("Right Trigger", gamepad2.right_trigger);
        telemetry.addData("Shooter Target", targetShooterVelocity);

        /*
        // Use right trigger to fire the ball inside
        // Of the current outtake chamber
        if(gamepad2.right_trigger > 0){
            currentOuttakeChamber.setOccupancy(false);
            bouncer.setPosition(bouncer.MAX_POSITION);
            
            shooterAlpha.setPower(-alphaSpeed);
            shooterBeta.setPower(shooterSpeed);
        }else{
            bouncer.setPosition(bouncer.MIN_POSITION);
            
            shooterAlpha.setPower(0);
            shooterBeta.setPower(0);
        }
        // Change to far shooting by raising
        // The speed of the top motor
        if(gamepad2.dpad_up){
            alphaSpeed = 0.75;
        }
        // Change to close shooting by lowering
        // The speed of the top motor
        if(gamepad2.dpad_down){
            alphaSpeed = shooterSpeed;
        }
        */
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

    private void jugglerHandler(){
        /*
        // Goes to nearest full chamber in shooting mode
        // Goes to nearest empty chamber in loading mode
        if(gamepad2.a){
            if(inLoadingMode){
                // Declares a closest chamber variable
                Chamber closestChamber = null;
                // Loops through the chambers to find the closest one
                for(Chamber chamber : chambers){
                    if(!(chamber.isOccupied()) && closestChamber != null){
                        double chamberDistance = chamber.getIntakeDistance(ballJuggler.getCurrentPosition());
                        if(chamberDistance < closestChamber.getIntakeDistance(ballJuggler.getCurrentPosition())){
                            closestChamber = chamber;
                        }
                    }
                }
                // Set the encoder's target position to the closest
                // Chamber's position and then set closest chamber
                // To current intake chamber
                ballJuggler.setTargetPosition((int) closestChamber.getOuttakePos());
                currentIntakeChamber = closestChamber;
            }else{
                Chamber closestChamber = null;
                // Loops through the chambers to find the closest one
                for(Chamber chamber : chambers){
                    if(chamber.isOccupied() && closestChamber != null){
                        double chamberDistance = chamber.getOuttakeDistance(ballJuggler.getCurrentPosition());
                        if(chamberDistance < closestChamber.getOuttakeDistance(ballJuggler.getCurrentPosition())){
                            closestChamber = chamber;
                        }
                    }
                }
                // Set the encoder's target position to the closest
                // Chamber's position and then set closest chamber
                // To current intake chamber
                ballJuggler.setTargetPosition((int) closestChamber.getOuttakePos());
                currentOuttakeChamber = closestChamber;
            }
            // Run the revolver to the target position at MAX speed
            ballJuggler.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ballJuggler.setPower(1);
        }
        
        if(gamepad2.xWasPressed()){
            // Turn the revolver so that the blue
            // Chamber either faces the intake or 
            // Outtake position
            if(inLoadingMode){
                // Set revolver intake to blue chamber
                // And set current intake chamber to blue chamber
                ballJuggler.setTargetPosition((int)xChamber.getIntakePos());
                currentIntakeChamber = xChamber;
            } else{
                // Set revolver outtake to blue chamber
                // And set current outtake chamber to blue chamber
                ballJuggler.setTargetPosition((int)xChamber.getOuttakePos());
                currentOuttakeChamber = xChamber;
            }
            // Turn the revolver at MAX speed
            ballJuggler.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ballJuggler.setPower(1);
        }
        if(gamepad2.yWasPressed()){
            // Turn the revolver so that the yellow
            // Chamber either faces the intake or 
            // Outtake position
            if(inLoadingMode){
                // Set revolver intake to yellow chamber
                // And set current intake chamber to yellow chamber
                ballJuggler.setTargetPosition((int)yChamber.getIntakePos());
                currentIntakeChamber = yChamber;
            } else{
                // Set revolver outtake to yellow chamber
                // And set current outtake chamber to yellow chamber
                ballJuggler.setTargetPosition((int)yChamber.getOuttakePos());
                currentOuttakeChamber = yChamber;
            }
            // Turn the revolver at MAX speed
            ballJuggler.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ballJuggler.setPower(1);
        }
        if(gamepad2.bWasPressed()){
            // Turn the revolver so that the red
            // Chamber either faces the intake or 
            // Outtake position
            if(inLoadingMode){
                // Set revolver intake to red chamber
                // And set current intake chamber to red chamber
                ballJuggler.setTargetPosition((int)bChamber.getIntakePos());
                currentIntakeChamber = bChamber;
            } else{
                // Set revolver outtake to red chamber
                // And set current outtake chamber to red chamber
                ballJuggler.setTargetPosition((int)bChamber.getOuttakePos());
                currentOuttakeChamber = bChamber;
            }
            // Turn the revolver at MAX speed
            ballJuggler.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ballJuggler.setPower(1);
        }
        */
    }
}
