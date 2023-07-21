package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="SovaTechTeleOp", group="Linear Opmode")
//@Disabled
public class SovaTechTeleOp extends LinearOpMode {

    // Declare
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor topLeft = null;
    private DcMotor topRight = null;
    private DcMotor bottomRight = null;
    private DcMotor bottomLeft = null;

    private DcMotorEx linear = null;


    private Servo gheara;


    double powerTopLeft = 0;
    double powerTopRight = 0;
    double powerBottomRight = 0;
    double powerBottomLeft = 0;



    boolean butonGheara = false;
    boolean deschis = true;

    double ghearaDeschisa = 0.75;
    double ghearaInchisa = 1;

    double clipValue = 1;


    /**
     * configurare hardware map:
     *   W1     W2
     *       *
     *   W4     W3
     */


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //Declare Motors
        topLeft = hardwareMap.get(DcMotor.class, "left_top");
        topRight = hardwareMap.get(DcMotor.class, "right_top");
        bottomRight = hardwareMap.get(DcMotor.class, "right_bottom");
        bottomLeft = hardwareMap.get(DcMotor.class, "left_bottom");

        linear = hardwareMap.get(DcMotorEx.class, "linear");

        gheara = hardwareMap.get(Servo.class, "gheara");


        /// de verificat directiile
        topLeft.setDirection(DcMotor.Direction.FORWARD);
        topRight.setDirection(DcMotor.Direction.REVERSE);
        bottomRight.setDirection(DcMotor.Direction.REVERSE);
        bottomLeft.setDirection(DcMotor.Direction.FORWARD);



        linear.setDirection(DcMotorSimple.Direction.REVERSE);

        linear.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

        linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            boolean butonGhearaOld = butonGheara;
            butonGheara = gamepad2.a;


            double vx = gamepad1.left_stick_x;
            double vy = gamepad1.left_stick_y;
            double r = -gamepad1.right_stick_x;

            if( gamepad1.right_trigger > 0 ) {
                clipValue = 1;
            } else {

                if( gamepad1.left_trigger > 0 ){
                    clipValue = 4;
                } else {
                    clipValue = 1.5;
                }

            }


            powerTopLeft = Range.clip((vy - vx + r), -0.8, 0.8);
            powerTopRight = Range.clip((vy + vx - r), -0.8, 0.8);
            powerBottomRight = Range.clip((vy - vx - r), -0.8, 0.8);
            powerBottomLeft = Range.clip((vy + vx + r), -0.8, 0.8);

            powerTopLeft /= clipValue;
            powerTopRight /= clipValue;
            powerBottomLeft /= clipValue;
            powerBottomRight /= clipValue;


            topLeft.setPower(powerTopLeft);
            topRight.setPower(powerTopRight);
            bottomRight.setPower(powerBottomRight);
            bottomLeft.setPower(powerBottomLeft);


            if( gamepad2.dpad_up ){

                linear.setPower(0.9);

            } else {

                if(gamepad2.dpad_down ){

                    linear.setPower(-0.9);

                } else {

                    linear.setPower(0);

                }

            }


            if( butonGheara && butonGheara != butonGhearaOld ) {

                deschis = !deschis;
                if(deschis) {
                    gheara.setPosition(ghearaDeschisa);
                }
                else {
                    gheara.setPosition(ghearaInchisa);
                }

            }


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


        }

    }



}