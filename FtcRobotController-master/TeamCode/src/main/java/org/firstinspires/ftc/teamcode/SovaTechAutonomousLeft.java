package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;



@Autonomous(name = "SovaTechAutonomousLeft", group = "Linear Opmode")
public class SovaTechAutonomousLeft extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    // declararea motoarelor

    private DcMotor topLeft = null;
    private DcMotor topRight = null;
    private DcMotor bottomRight = null;
    private DcMotor bottomLeft = null;


    private Servo gheara;

    private DcMotorEx linear = null;


    boolean deschis = true;

    private double ghearaDeschisa = 0.75;
    private double ghearaInchisa = 1;

    private double powerLinear = 0.9;



    OpenCvWebcam webcam;


    @Override
    public void runOpMode() {


        // instantierea camerei
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = null;
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); // numele camerei
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);


        // pipeline
        colorDetector visionPipeline = new colorDetector();
        webcam.setPipeline(visionPipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Failed","");
                telemetry.update();
            }
        });




        topLeft = hardwareMap.get(DcMotor.class, "left_top");
        topRight = hardwareMap.get(DcMotor.class, "right_top");
        bottomRight = hardwareMap.get(DcMotor.class, "right_bottom");
        bottomLeft = hardwareMap.get(DcMotor.class, "left_bottom");

        gheara = hardwareMap.get(Servo.class, "gheara");
        linear = hardwareMap.get(DcMotorEx.class, "linear");


        // de verificat directiile



        /*
        topLeft.setDirection(DcMotor.Direction.REVERSE);
        topRight.setDirection(DcMotor.Direction.FORWARD);
        bottomRight.setDirection(DcMotor.Direction.REVERSE);
        bottomLeft.setDirection(DcMotor.Direction.REVERSE);
llllllllllllllllllllllllllllllllllllllllo
         */

        linear.setDirection(DcMotorSimple.Direction.REVERSE);

        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /**
         * linear reverse?
         */



        waitForStart();
        runtime.reset();

        if( opModeIsActive() ){


            //sleep(1000);

            int result = visionPipeline.getResult();

            sleep(200);

            result = visionPipeline.getResult();

            telemetry.addData( "result ", ""+result );
            telemetry.update();

            //waitForStart();



            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Pose2d startPose = new Pose2d(0, 0, 0);

            drive.setPoseEstimate(startPose);

            gheara.setPosition(ghearaInchisa);
            sleep(800);


            ridicareLinear(800);




            /// A2
            TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                    .forward (3)
                    .strafeRight(23)
                    .forward(38)

                    .turn(Math.toRadians(90))
                    .back(3)

                    .build();

            drive.followTrajectorySequence(trajSeq1);

            ridicareLinear(3000);

            TrajectorySequence tr2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                    .forward(6)
                    .build();

            drive.followTrajectorySequence(tr2);



            coborareLinear(1500);

            gheara.setPosition(ghearaDeschisa);
            deschis=true;
            sleep(700);








           // result=1;

            TrajectorySequence trajSeqThree= drive.trajectorySequenceBuilder(tr2.end())
                    .back(4)
                    .strafeRight(12)
                    .build();




            TrajectorySequence trajSeqTwo = drive.trajectorySequenceBuilder(tr2.end())
                    .back(4)
                    .strafeRight(12)
                    .forward(24)
                    .build();


            TrajectorySequence trajSeqOne = drive.trajectorySequenceBuilder(tr2.end())
                    .back(4)
                    .strafeRight(12)
                    .forward(46)
                    .build();



            if (!isStopRequested()){
                if( result == 1 ){
                    drive.followTrajectorySequence(trajSeqOne);
                } else {
                    if( result == 2 ){
                        drive.followTrajectorySequence(trajSeqTwo);
                    } else {
                        drive.followTrajectorySequence(trajSeqThree);
                    }
                }
            }












        }

    }

    private void ridicareLinear(int mil){

        /*
        linear.setTargetPosition(-5000);


        // set motors to run to target encoder position and stop with brakes on.
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linear.setPower(-powerLinear);

        while (opModeIsActive() && linear.isBusy())
        {
            telemetry.addData("encoder", linear.getCurrentPosition());
            telemetry.update();
            idle();
        }


        linear.setPower(0);


         */

        linear.setPower(0.9);
        sleep(mil);
        linear.setPower(0);


    }

    private void coborareLinear(int mil){

        /*
        linear.setTargetPosition(0);


        // set motors to run to target encoder position and stop with brakes on.
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linear.setPower(powerLinear);

        while (opModeIsActive() && linear.isBusy())
        {
            telemetry.addData("encoder", linear.getCurrentPosition());
            telemetry.update();
            idle();
        }


        linear.setPower(0);

         */

        linear.setPower(-0.9);
        sleep(mil);
        linear.setPower(0);

    }


    public class colorDetector extends OpenCvPipeline {


        Mat mat = new Mat();
        Mat mat1 = new Mat();
        Mat mat2 = new Mat();
        Mat mat3 = new Mat();


        Scalar GRAY = new Scalar(100, 100, 100); // RGB values for gray

        public boolean element1 = false;
        public boolean element2 = false;
        public boolean element3 = false;




        int ROILeft, lineDistFromTop, ROIRight;



        final Rect ROI_1 = new Rect(
                new Point(80, 0),
                new Point(110, 240));

        final Rect ROI_2 = new Rect(
                new Point(110, 0),
                new Point(160, 240));




        double GREEN_THRESHOLD = 30; // de schimbat



        @Override
        public Mat processFrame(Mat input) {

            Imgproc.rectangle( input, ROI_1, GRAY, 5 );
            Imgproc.rectangle( input, ROI_2, GRAY, 5 );

            Imgproc.cvtColor( input, mat1, Imgproc.COLOR_RGB2YCrCb );
            Core.extractChannel( mat1, mat2, 1 );
            Imgproc.threshold(mat2, mat3, 119, 255, Imgproc.THRESH_BINARY_INV);

            Mat sumMat = mat3.submat(ROI_1);
            Mat sumMat_2 = mat3.submat(ROI_2);

            double greenValue_1 = Core.sumElems(sumMat).val[0] / 255.0;
            double greenValue_2 = Core.sumElems(sumMat_2).val[0] / 255.0;

            if( greenValue_1 >= GREEN_THRESHOLD && greenValue_2 < GREEN_THRESHOLD ){
                element1 = true;
            } else {

                if( greenValue_2 >= GREEN_THRESHOLD && greenValue_1 < GREEN_THRESHOLD ){
                    element2 = true;
                } else {

                    if( greenValue_1 >= GREEN_THRESHOLD && greenValue_2 >= GREEN_THRESHOLD ){
                        if( greenValue_1 > greenValue_2 ){
                            element1 = true;
                        } else {
                            element2 = true;
                        }
                    } else {
                        element3 = true;
                    }
                }

            }


            return mat3;
        }


        public int getResult(){

            int res=0;

            if(element1){
                res=1;
            } else {
                if(element2){
                    res=2;
                } else {
                    res=3;
                }
            }

            return res;
        }

    }




}
