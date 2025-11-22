package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="SpinnyTest")
public class SpinnyTest extends LinearOpMode {

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

    private int highPosition = 500;
    private int lowPosition = 0;

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
//
//        frontLeft = hardwareMap.get(DcMotor.class, "fl");
//        frontRight = hardwareMap.get(DcMotor.class, "fr");
//        backLeft = hardwareMap.get(DcMotor.class, "bl");
//        backRight = hardwareMap.get(DcMotor.class, "br");
//
//        shooterBeta = hardwareMap.get(DcMotorEx.class, "beta");
//        shooterAlpha = hardwareMap.get(DcMotorEx.class, "alpha");
        gluttonousRizzler = hardwareMap.get(DcMotor.class, "rizzler");
//        shooterSpinner = hardwareMap.get(DcMotor.class, "spinny");
        //ballJuggler = hardwareMap.get(DcMotor.class, "juggler");

        //bouncer = hardwareMap.get(Servo.class, "bouncer");

        telemetry.addData("Components", "Acquired");
        telemetry.update();

        //shooterSpinner.setDirection(DcMotor.Direction.REVERSE);
        //shooterSpinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while(opModeIsActive()){
//            if(gamepad2.y){
//                shooterSpinner.setTargetPosition(highPosition);
//                shooterSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                shooterSpinner.setPower(0.5);
//            }
//            if(gamepad2.a){
//                shooterSpinner.setTargetPosition(lowPosition);
//                shooterSpinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                shooterSpinner.setPower(0.5);
//            }
//
//            double pos = shooterSpinner.getCurrentPosition();
//            double wantedPos = shooterSpinner.getTargetPosition();

            if (gamepad2.left_stick_y > 0) {
                gluttonousRizzler.setPower(-1);
            } else if (gamepad2.left_stick_y < 0) {
                gluttonousRizzler.setPower(1);
            } else {
                gluttonousRizzler.setPower(0);
            }

//            telemetry.addData("Current Pos", pos);
//            telemetry.addData("Wanted Pos", wantedPos);

            telemetry.update();
        }
    }
}
