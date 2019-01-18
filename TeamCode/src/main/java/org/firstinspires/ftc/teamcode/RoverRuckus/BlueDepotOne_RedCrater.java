package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.revextensions2.RevExtensions2;

import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;
@Autonomous(name = "BlueDepot_RedC", group = "Autonomous")
public class BlueDepotOne_RedCrater extends LinearOpMode {
    public RoverHardware robot = new RoverHardware(); //Create a new instance of the

    //Create detector gateway
    private GoldAlignDetector detector;

    //Create Variables
    float angleTurn;
    int blue;
    Timer dropMark;
    TimerTask armOut;
    int blockPosition;

    public void runOpMode() {
        robot.init(hardwareMap);

        RevExtensions2.init();

        //Initialize OpenCV
        detector = new GoldAlignDetector(); //Create Detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); //Declare Camera View Display
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.perfectAreaScorer.perfectArea = 10000;
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        //Initialize Gyro
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES; //Declare units to work in
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; //Declare acceleration units
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");//hardware map
        robot.imu.initialize(parameters1);

        composeTelemetry();//create the heading, roll, and pitch

        telemetry.addLine()//Telemetry for all 3 of our gyro measures
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle);
                    }
                });
        while (!opModeIsActive()) {//In the initializing phase we can look at the pitch heading and roll to setup
            telemetry.update();
        }
        waitForStart();//waits for the start button to be pressed

        //Raise arm
//        while (robot.upperLimit.red() > 200 && opModeIsActive()) {
//            robot.hang.setPower(-1);
//        }
//        robot.hang.setPower(0);
//        sleep(200);

        //Drive forward slightly
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBop.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.left1.setTargetPosition(450);//900
        robot.right1.setTargetPosition(450);//900
        robot.left1.setPower(0.8);
        robot.right1.setPower(0.8);

        while (robot.left1.isBusy() && robot.right1.isBusy() && opModeIsActive()) {
            telemetry.addData("right power", robot.right1.getPower());
            telemetry.addData("right position", robot.right1.getCurrentPosition());
            telemetry.addData("left power", robot.left1.getPower());
            telemetry.addData("left position", robot.left1.getCurrentPosition());
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);


//        while(robot.bottomDrop.getState()) {
//            robot.drop.setPower(-0.65);
//            telemetry.addData("bottomDrop", robot.bottomDrop.getState());
//            telemetry.update();
//        }
        //robot.drop.setPower(0);
//        dropMark = new Timer();
//        armOut = new BlueDepotOne_BlueCrater().armOut;
//        dropMark.schedule(armOut, 500, 10000);
//        sleep(4000);

        if (detector.getAligned() == true || detector.getAligned() == false) {
            //Hunt for the Block
            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (detector.getXPosition() < 235 && opModeIsActive() || detector.getXPosition() > 345 && opModeIsActive()) {
                telemetry.addData("Status", "searching for angle");
                telemetry.addData("xpos", detector.getXPosition());
                telemetry.addData("IsAligned", detector.getAligned());
                if (detector.getXPosition() < 235) {
                    robot.left1.setPower(-.8);
                    robot.right1.setPower(.8);
                } else if (detector.getXPosition() > 340) {
                    robot.left1.setPower(.8);
                    robot.right1.setPower(-.8);
                }
            }
        } else {
            detector.disable();

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle > 15 && opModeIsActive() || robot.angles.firstAngle < 25 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                robot.left1.setPower(Math.abs((30 - robot.angles.firstAngle) / 40) * 0.6);
                robot.right1.setPower(Math.abs((30 - robot.angles.firstAngle) / 40) * -0.6);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
                telemetry.update();
            }
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);
        sleep(100);

        telemetry.addData("heading", robot.angles.firstAngle);
        telemetry.update();
        if (robot.angles.firstAngle > 15) {
            blockPosition = 1; //Block is located ein the left spot
        }
        if (robot.angles.firstAngle < 15 && robot.angles.firstAngle > -15) {
            blockPosition = 2;//Block is located in the middle.
        }
        if (robot.angles.firstAngle < -15) {
            blockPosition = 3;//Block is located in the right spot
        }

//        //Lower intake and extend arm out
        telemetry.addData("heading", robot.angles.firstAngle);
        telemetry.update();

        if (blockPosition == 1) {
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(1800);//3600
            robot.right1.setTargetPosition(1800);//3600
            robot.left1.setPower(0.8);
            robot.right1.setPower(0.8);
            while (robot.left1.isBusy() && opModeIsActive()) {
            }

            telemetry.update();
            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (robot.angles.firstAngle < 105 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(((130 - angleTurn) / 45) * 0.8);
                robot.right1.setPower(((130 - angleTurn) / 45) * -0.8);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
                telemetry.update();
            }
            robot.left1.setPower(0);
            //robot.drop.setPower(0);
            robot.right1.setPower(0);

//            //Drive forward and slightly into the wall
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(3900);//7800
            robot.right1.setTargetPosition(3900);//7800
            robot.left1.setPower(0.9 * 1.1);
            robot.right1.setPower(0.9);
            while (robot.right1.isBusy() && robot.left1.isBusy() && opModeIsActive()) {
            }

        }
        if (blockPosition == 2) {
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(1750);
            robot.right1.setTargetPosition(1750);//2500
            robot.left1.setPower(0.9);
            robot.right1.setPower(0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
            }

            robot.left1.setTargetPosition(250);
            robot.right1.setTargetPosition(250);//500
            robot.left1.setPower(-0.7);
            robot.right1.setPower(-0.7);
            while (robot.left1.isBusy() && opModeIsActive()) {
            }


            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle > -70 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(((100 - angleTurn) / 67) * 0.6);
                robot.right1.setPower(((100 - angleTurn) / 67) * -0.6);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
                telemetry.update();
            }
//
//            //Drive to the team marker area
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(4200);//8400
            robot.right1.setTargetPosition(4200);
            robot.left1.setPower(0.8 * 1.4);
            robot.right1.setPower(0.8);


            while (robot.left1.isBusy() && opModeIsActive()) {
            }
            robot.right1.setPower(0);
            robot.left1.setPower(0);


        }
        if (blockPosition == 3) {
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(1500);//3000
            robot.right1.setTargetPosition(1500);
            robot.left1.setPower(0.9);
            robot.right1.setPower(0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
            }

            robot.left1.setTargetPosition(250);//500
            robot.right1.setTargetPosition(250);
            robot.left1.setPower(-0.7);
            robot.right1.setPower(-0.7);
            while (robot.left1.isBusy() && opModeIsActive()) {
            }


            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle > 90 && opModeIsActive() || robot.angles.firstAngle < 63 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(((100 - angleTurn) / 67) * 0.6);
                robot.right1.setPower(((100 - angleTurn) / 67) * -0.6);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
                telemetry.update();
            }
//
//            //Drive to the team marker area
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(4300);//8600
            robot.right1.setTargetPosition(4300);
            robot.left1.setPower(0.8 * 1.6);
            robot.right1.setPower(0.8);


            while (robot.left1.isBusy() && opModeIsActive()) {
            }
            robot.right1.setPower(0);
            robot.left1.setPower(0);


        }
    }
        class armOut extends TimerTask {
            boolean flag1 = false;

            public void run() {

                robot.leftBop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightBop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftBop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightBop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftBop.setPower(0.55);
                robot.rightBop.setPower(0.55);

                while (flag1 == false) {
                    if (robot.bopLimit.red() >= 110 && robot.bopLimit.alpha() < 300 && opModeIsActive()) {
                        robot.leftBop.setPower(0);
                        robot.rightBop.setPower(0);
                        robot.leftBop.setTargetPosition(robot.leftBop.getCurrentPosition());
                        robot.rightBop.setTargetPosition(robot.rightBop.getCurrentPosition());
                        robot.rotateMech.setPower(0);
                        flag1 = true;
                    }
                    if (!robot.topDrop.getState()) {
                        //robot.drop.setPower(0);
                        robot.intake.setPower(0);
                    }
//                else{
//                    robot.drop.setPower(0.65);
//
//                }
                    telemetry.addData("Waiting for Flag", "");
                    telemetry.update();
                }
                telemetry.addData("Passed Flag", "");
                telemetry.update();
                armOut.cancel();
                dropMark.cancel();
            }
        }

        void composeTelemetry () {


            telemetry.addAction(new Runnable() {
                @Override
                public void run() {
                    robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    robot.gravity = robot.imu.getGravity();
                }
            });
            telemetry.addLine()
                    .addData("status", new Func<String>() {
                        @Override
                        public String value() {
                            return robot.imu.getSystemStatus().toShortString();
                        }
                    })
                    .addData("calib", new Func<String>() {
                        @Override
                        public String value() {
                            return robot.imu.getCalibrationStatus().toString();
                        }
                    });
            telemetry.addLine()
                    .addData("heading", new Func<String>() {
                        @Override
                        public String value() {
                            return robot.formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
                        }
                    })
                    .addData("roll", new Func<String>() {
                        @Override
                        public String value() {
                            return robot.formatAngle(robot.angles.angleUnit, robot.angles.secondAngle);
                        }
                    })
                    .addData("pitch", new Func<String>() {
                        @Override
                        public String value() {
                            return robot.formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle);
                        }
                    });

            telemetry.addLine()
                    .addData("grvty", new Func<String>() {
                        @Override
                        public String value() {
                            return robot.gravity.toString();
                        }
                    })
                    .addData("mag", new Func<String>() {
                        @Override
                        public String value() {
                            return String.format(Locale.getDefault(), "%.3f",
                                    Math.sqrt(robot.gravity.xAccel * robot.gravity.xAccel
                                            + robot.gravity.yAccel * robot.gravity.yAccel
                                            + robot.gravity.zAccel * robot.gravity.zAccel));
                        }
                    });

        }

    }