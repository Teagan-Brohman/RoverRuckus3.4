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

@Autonomous(name = "BlueDepotOneBlock", group = "Autonomous")
public class BlueDepotOneBlock extends LinearOpMode {
    public RoverHardware robot = new RoverHardware(); //Initialize our Hardware referenced in our hardware class


    private GoldAlignDetector detector; //Initialize detector used to search and find the golden block

    float angleTurn; //create a float variable used to store our current heading position while turning
    int blue; //create a integer variable used to store the amount of blue being registered from a color sensor
    int red; //create a integer variable used to store the amount of red being registered from a color sensor
    Timer waitTimer;
    int blockPosition;
    int timer;

    public void runOpMode() {//Starts running the code
        robot.init(hardwareMap); //register the hardware mappings from the hardware class with names given to motors servos, etc.
        RevExtensions2.init(); //create name for our RevExpansion hub to look at things like voltage
        //robot.drop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Initialize OpenCV
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        //Initialize Gyro
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters1);

        composeTelemetry();

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
        while (!opModeIsActive()) {
            telemetry.addData("DetectorXPos", detector.getXPosition());
            telemetry.update();
        }
        waitForStart();

//        //Raise arm
//        while (robot.upperLimit.red() > 250 && opModeIsActive()) {
//            robot.hang.setPower(1);
//        }
//        robot.hang.setPower(0);
//        sleep(200);

        telemetry.addData("BlockPosition", blockPosition);
        telemetry.update();

        //Drive forward slightly
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

            if (opModeIsActive() == false) {
                break;
            }
        }

        double Xpos = detector.getXPosition();
        telemetry.addData("XPos", Xpos);
        telemetry.update();
        //Block is located in the middle spot
        if (Xpos < 400 && Xpos > 10) {
            blockPosition = 2;//Block is located in the middle.
        } else if (Xpos > 400) {
            blockPosition = 3;//Block is located in the right spot
        } else {
            blockPosition = 1;//block is left
        }


        if (detector.getAligned() == true || detector.getAligned() == false) {
            //Hunt for the Block
            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (detector.getXPosition() < 235 && opModeIsActive() || detector.getXPosition() > 345 && opModeIsActive()) {
                telemetry.addData("Status", "searching for angle");
                telemetry.addData("xpos", detector.getXPosition());
                telemetry.addData("IsAligned", detector.getAligned());
                if (detector.getXPosition() < 235) {
                    robot.left1.setPower(-.4);
                    robot.right1.setPower(.4);
                } else if (detector.getXPosition() > 340) {
                    robot.left1.setPower(.4);
                    robot.right1.setPower(-.4);
                }
            }
        } else {
            detector.disable();

            telemetry.addData("Status:", "no Block Seen");
            telemetry.update();

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle > 25 && opModeIsActive() || robot.angles.firstAngle < 35 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                robot.left1.setPower(Math.abs((30 - robot.angles.firstAngle) / 40) * 0.8);
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

        if (blockPosition == 1) {
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(1900);//3400
            robot.right1.setTargetPosition(1900);//3400
            robot.left1.setPower(0.8);
            robot.right1.setPower(0.8);
            while (robot.left1.isBusy() && opModeIsActive()) {}

            robot.left1.setTargetPosition(900);//3400
            robot.right1.setTargetPosition(900);//3400
            robot.left1.setPower(0.8);
            robot.right1.setPower(0.8);
            while (robot.left1.isBusy() && opModeIsActive()) {}


            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle < 68.2 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(((66.2 - angleTurn) / 35) * -0.45);
                robot.right1.setPower(((66.2 - angleTurn) / 35) * 0.45);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
                telemetry.update();
            }
//
            //Drive to the team marker area
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(2200);//5200
            robot.right1.setTargetPosition(2200);//5200
            robot.left1.setPower(0.9);
            robot.right1.setPower(0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
            }
            robot.right1.setPower(0);
            robot.left1.setPower(0);

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle < 142 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(((142 - angleTurn) / 43) * -0.3);
                robot.right1.setPower(((142 - angleTurn) / 43) * 0.3);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
                telemetry.update();
            }
            robot.left1.setPower(0);
            robot.right1.setPower(0);

            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(-2000);//5200
            robot.right1.setTargetPosition(-2000);//5200
            robot.left1.setPower(-0.9);
            robot.right1.setPower(-0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
            }
            robot.right1.setPower(0);
            robot.left1.setPower(0);
        }
        if (blockPosition == 2) {
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(1450);//2500
            robot.right1.setTargetPosition(1450);//2500
            robot.left1.setPower(0.9);
            robot.right1.setPower(0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
                telemetry.addData("left1", robot.left1.getCurrentPosition());
                telemetry.update();
            }

            robot.left1.setTargetPosition(400);
            robot.right1.setTargetPosition(400);
            robot.left1.setPower(-0.6);
            robot.right1.setPower(-0.6);
            while (robot.left1.isBusy() && opModeIsActive()) {}
            robot.left1.setPower(0);
            robot.right1.setPower(0);

            sleep(500);

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle < 68.2 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(((66.2 - angleTurn) / 50) * -0.36);
                robot.right1.setPower(((66.2 - angleTurn) / 50) * 0.36);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
                telemetry.update();
            }
//
            //Drive to the team marker area
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(2900);//5200
            robot.right1.setTargetPosition(2900);//5200
            robot.left1.setPower(0.9);
            robot.right1.setPower(0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
            }
            robot.right1.setPower(0);
            robot.left1.setPower(0);

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle < 141 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(((142 - angleTurn) / 43) * -0.3);
                robot.right1.setPower(((142 - angleTurn) / 43) * 0.3);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
                telemetry.update();
            }
            robot.left1.setPower(0);
            robot.right1.setPower(0);

            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(-1800);//5200
            robot.right1.setTargetPosition(-100);//5200
            robot.left1.setPower(-0.9);
            robot.right1.setPower(-0.9);
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


//            robot.bop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.bop.setTargetPosition(-400);
//            robot.bop.setPower(-0.9);
//            while (robot.bop.isBusy() && opModeIsActive()) {
//                telemetry.addData("encoder", robot.bop.getCurrentPosition());
//                telemetry.addData("power", robot.bop.getPower());
//                telemetry.addData("mode", robot.bop.getMode());
//                telemetry.update();
//            }
//            robot.bop.setPower(0);
//            //}
//            //Change the arm angle so it can hit the block
//            robot.rotateMech.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            sleep(100);
//            robot.rotateMech.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rotateMech.setTargetPosition(180);
//            robot.rotateMech.setPower(0.8);
//            while (robot.rotateMech.getCurrentPosition() < 160 && opModeIsActive()) {
//                telemetry.addData("rotate mech position", robot.rotateMech.getCurrentPosition());
//                telemetry.update();
//            }
//            robot.rotateMech.setPower(0);
//            //}
////
////            robot.drop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            robot.drop.setTargetPosition(robot.BOTTOM_INTAKE);
////            robot.drop.setPower(-0.8);
////            while(robot.drop.getCurrentPosition() <= 180 && opModeIsActive()){
////                telemetry.addData("Drop Motor Power", robot.drop.getPower());
////                telemetry.addData("Drop Motor Position", robot.drop.getCurrentPosition());
////            }
//            while (robot.bottomDrop.getState()) {
//                robot.drop.setPower(-0.65);
//                telemetry.addData("bottomDrop", robot.bottomDrop.getState());
//                telemetry.update();
//            }
//            robot.drop.setPower(0);
////
//
//            robot.bop.setTargetPosition(-1800);
//            robot.bop.setPower(-0.6);
//            while (robot.bop.isBusy() && opModeIsActive()) {
//                robot.intake.setPower(0);
//                //Drive forward slightly
//            }
//            robot.bop.setTargetPosition(-1400);
//            robot.bop.setPower(-0.9);
//            while (robot.bop.isBusy() && opModeIsActive()) {
//            }
//
//            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            robot.bop.setTargetPosition(-150);
//            robot.bop.setPower(0.9);
//            while (robot.bop.isBusy() && opModeIsActive()) {
//                if (robot.topDrop.getState()) {
//                    robot.drop.setPower(.7);
//                }
//                robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.left1.setTargetPosition(900);
//                robot.right1.setTargetPosition(900);
//                robot.left1.setPower(0.8);
//                robot.right1.setPower(0.8);
//                while (robot.left1.isBusy() && robot.right1.isBusy() && opModeIsActive()) {
//                    telemetry.addData("right power", robot.right1.getPower());
//                    telemetry.addData("right position", robot.right1.getCurrentPosition());
//                    telemetry.addData("left power", robot.left1.getPower());
//                    telemetry.addData("left position", robot.left1.getCurrentPosition());
//                    telemetry.update();
//                    if (opModeIsActive() == false) {
//                        break;
//                    }
//                }
//            }

//        if (detector.getAligned() == true || detector.getAligned() == false) {
//            //Hunt for the Block
//            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            while (detector.getXPosition() < 235 && opModeIsActive() || detector.getXPosition() > 345 && opModeIsActive()) {
//                telemetry.addData("Status", "searching for angle");
//                telemetry.addData("xpos", detector.getXPosition());
//                telemetry.addData("IsAligned", detector.getAligned());
//                if (detector.getXPosition() < 235) {
//                    robot.left1.setPower(.8);
//                    robot.right1.setPower(-.8);
//                } else if (detector.getXPosition() > 340) {
//                    robot.left1.setPower(-.8);
//                    robot.right1.setPower(.8);
//                }
//            }
//        } else {
//            robot.angles.firstAngle = angleTurn;
//
//            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            while (robot.angles.firstAngle > 12 && opModeIsActive() || robot.angles.firstAngle < 23 && opModeIsActive()) {
//                angleTurn = robot.angles.firstAngle;
//                robot.left1.setPower(Math.abs((18 - angleTurn) / 47) * 0.5);
//                robot.right1.setPower(Math.abs((18 - angleTurn) / 47) * -0.5);
//                telemetry.addData("left1 power", robot.left1.getPower());
//                telemetry.addData("right1 power", robot.right1.getPower());
//                telemetry.addData("heading", robot.angles.firstAngle);
//                telemetry.addData("angle var:", angleTurn);
//                telemetry.update();
//
//            }
//        }
//        robot.left1.setPower(0);
//        robot.right1.setPower(0);
//        sleep(100);
//
//
////        //Lower intake and extend arm out
//        telemetry.addData("heading", robot.angles.firstAngle);
//        telemetry.update();
//        if (robot.angles.firstAngle < 10 && robot.angles.firstAngle > -10) {
//            robot.bop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.bop.setTargetPosition(-400);
//            robot.bop.setPower(-0.6);
//            while (robot.bop.isBusy() && opModeIsActive()) {
//                telemetry.addData("encoder", robot.bop.getCurrentPosition());
//                telemetry.addData("power", robot.bop.getPower());
//                telemetry.addData("mode", robot.bop.getMode());
//                telemetry.update();
//            }
//            robot.bop.setPower(0);
//            //}
//            //Change the arm angle so it can hit the block
//            //if (robot.angles.firstAngle > 2) {
////            robot.drop.setPower(0);
////            robot.drop.setTargetPosition(0);
////            if(robot.angles.firstAngle < 25) {
//            robot.rotateMech.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rotateMech.setTargetPosition(180);
//            robot.rotateMech.setPower(0.8);
//            while (robot.rotateMech.isBusy() && opModeIsActive()){
//            }
//            //}
////
//            robot.drop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.drop.setTargetPosition(robot.BOTTOM_INTAKE);
//            robot.drop.setPower(-0.8);
//            while(robot.drop.getCurrentPosition() <= -170 && opModeIsActive()){
//                telemetry.addData("Drop Motor Power", robot.drop.getPower());
//                telemetry.addData("Drop Motor Position", robot.drop.getCurrentPosition( ));
//            }
//            robot.intake.setPower(0.9);
////
//            robot.bop.setTargetPosition(-1400);
//            robot.bop.setPower(-0.6);
//            while (robot.bop.isBusy() && opModeIsActive()) {
//                robot.intake.setPower(0.9);
//            }
//
//        }
//        else{
//            robot.bop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.bop.setTargetPosition(-400);
//            robot.bop.setPower(-0.6);
//            while (robot.bop.isBusy() && opModeIsActive()) {
//                telemetry.addData("encoder", robot.bop.getCurrentPosition());
//                telemetry.addData("power", robot.bop.getPower());
//                telemetry.addData("mode", robot.bop.getMode());
//                telemetry.update();
//            }
//            robot.bop.setPower(0);
//            //}
//            //Change the arm angle so it can hit the block
//            //if (robot.angles.firstAngle > 2) {
////            robot.drop.setPower(0);
////            robot.drop.setTargetPosition(0);
//
//            if(robot.angles.firstAngle < 25){
//                robot.rotateMech.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.rotateMech.setTargetPosition(100);
//                robot.rotateMech.setPower(0.8);
//                while (robot.rotateMech.isBusy() && opModeIsActive()){
//                }
//            }
////
//            robot.drop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.drop.setTargetPosition(robot.BOTTOM_INTAKE);
//            robot.drop.setPower(-0.8);
//            while(robot.drop.getCurrentPosition() <= -170 && opModeIsActive()){
//                telemetry.addData("Drop Motor Power", robot.drop.getPower());
//                telemetry.addData("Drop Motor Position", robot.drop.getCurrentPosition( ));
//            }
//            robot.intake.setPower(0.9);
////
//            if(robot.angles.firstAngle < 40) {
//                robot.bop.setTargetPosition(-1750);
//                robot.bop.setPower(-0.6);
//                while (robot.bop.isBusy() && opModeIsActive()) {
//                    robot.intake.setPower(0.9);
//                }
//            } else{
//                robot.bop.setTargetPosition(-2000);
//                robot.bop.setPower(-0.6);
//                while (robot.bop.isBusy() && opModeIsActive()) {
//                    robot.intake.setPower(0.9);
//                }
//            }
//        }
//
//
//        //bring arm back in
////        robot.bop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        robot.bop.setTargetPosition(-50);
////        robot.bop.setPower(0.9);
////        while (robot.bop.isBusy() && opModeIsActive()){
////            robot.intake.setPower(0.9);
////        }
//        robot.bop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.bop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        while (robot.bopLimit.red() <= 130 && robot.bopLimit.alpha() < 300 && opModeIsActive()){
//            robot.bop.setPower(0.5);
//            robot.rotateMech.setPower(-0.1);
//            robot.rotateMech.setTargetPosition(0);
//        }
//        robot.bop.setPower(0);
//        robot.rotateMech.setPower(0);
//
//        robot.drop.setTargetPosition(robot.TOP_INTAKE);
//        robot.drop.setPower(0.8);
//
//
//        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.left1.setTargetPosition(1300);
//        robot.right1.setTargetPosition(1300);
//        robot.left1.setPower(0.8);
//        robot.right1.setPower(0.8);
//
//        while(robot.left1.getCurrentPosition() < 500 && opModeIsActive()){
//
//        }
//
//        //turn right
//        telemetry.update();
//        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        while (robot.angles.firstAngle > -75 && opModeIsActive() || (robot.angles.firstAngle < -90 && robot.angles.firstAngle < 0) && opModeIsActive()) {
//            robot.left1.setPower(Math.abs((-87 - robot.angles.firstAngle) / -50) * -0.8);
//            robot.right1.setPower(Math.abs((-87 - robot.angles.firstAngle) / -50) * 0.8);
//            telemetry.addData("left1 power", robot.left1.getPower());
//            telemetry.addData("right1 power", robot.right1.getPower());
//            telemetry.addData("heading", robot.angles.firstAngle);
//            telemetry.addData("angle var:", angleTurn);
//            telemetry.update();
//        }
//        robot.left1.setPower(0);
//        robot.right1.setPower(0);
//
//        //Drive to other blocks
//        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.left1.setTargetPosition(5970);
//        robot.right1.setTargetPosition(5970);
//        robot.left1.setPower(0.9);
//        robot.right1.setPower(0.9);
//        while (robot.left1.isBusy() && opModeIsActive() || robot.right1.isBusy() && opModeIsActive()) {
//            robot.intake.setPower(0.9);
//        }
//        robot.right1.setPower(0);
//        robot.left1.setPower(0);
//        robot.intake.setPower(0);
//
//        //turn right
//        telemetry.update();
//        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        while (robot.angles.firstAngle > -159 && opModeIsActive() || (robot.angles.firstAngle < -190 && robot.angles.firstAngle < 0) && opModeIsActive()) {
//            robot.left1.setPower(Math.abs((-180 - robot.angles.firstAngle) / -43) * -0.8);
//            robot.right1.setPower(Math.abs((-180 - robot.angles.firstAngle) / -43) * 0.8);
//            telemetry.addData("left1 power", robot.left1.getPower());
//            telemetry.addData("right1 power", robot.right1.getPower());
//            telemetry.addData("heading", robot.angles.firstAngle);
//            telemetry.addData("angle var:", angleTurn);
//            telemetry.update();
//        }
//        robot.left1.setPower(0);
//        robot.right1.setPower(0);
////
//        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.left1.setTargetPosition(-1650);
//        robot.right1.setTargetPosition(-1650);
//        robot.left1.setPower(-0.8 * 1.1);
//        robot.right1.setPower(-0.8);
//        while (robot.left1.isBusy() && opModeIsActive()) {}
//        robot.left1.setPower(-0.8 * 1.1);
//        robot.right1.setPower(-0.8);
//        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
//        telemetry.update();
//
//        while(robot.cornerSensor.blue() <= 45 &&opModeIsActive()){
//            blue = robot.cornerSensor.blue();
//            robot.left1.setPower(-0.4);
//            robot.right1.setPower(-0.4);
//            telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
//            telemetry.addData("Alpha", robot.cornerSensor.alpha());
//            telemetry.addData("BLUE", blue);
//            telemetry.addData("Searching", "");
//            telemetry.update();
//        }
//        robot.left1.setPower(0);
//        robot.right1.setPower(0);
//
//        //Turn and prep to drop off marker
//        telemetry.update();
//        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        while (robot.angles.firstAngle > -74 && opModeIsActive() || (robot.angles.firstAngle < -86 && robot.angles.firstAngle < 0) && opModeIsActive()) {
//            robot.left1.setPower(Math.abs((-75 - robot.angles.firstAngle) / -45) * 0.8);
//            robot.right1.setPower(Math.abs((-75 - robot.angles.firstAngle) / -45) * -0.8);
//            telemetry.update();
//        }
//        robot.left1.setPower(0);
//        robot.right1.setPower(0);
//
//        robot.marker.setPosition(robot.DILBERT_DOWN);
//        sleep(500);
//
//
//        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.left1.setTargetPosition(-11000);
//        robot.right1.setTargetPosition(-11000);
//        robot.left1.setPower(-0.9 * 1.1);
//        robot.right1.setPower(-0.9);
//        while (robot.left1.isBusy() && opModeIsActive()) {}
    }

//    class armOut extends TimerTask {
//        boolean flag1 = false;
//        public void run(){
//
//            robot.leftBop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.rightBop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.leftBop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightBop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.leftBop.setPower(0.55);
//            robot.rightBop.setPower(0.55);
//
//            while (flag1 == false){
//                if (robot.bopLimit.red() >= 110 && robot.bopLimit.alpha() < 300 && opModeIsActive()) {
//                    robot.leftBop.setPower(0);
//                    robot.rightBop.setPower(0);
//                    robot.leftBop.setTargetPosition(robot.leftBop.getCurrentPosition());
//                    robot.rightBop.setTargetPosition(robot.rightBop.getCurrentPosition());
//                    robot.rotateMech.setPower(0);
//                    flag1 = true;
//                }
//                if(!robot.topDrop.getState()){
//                    //robot.drop.setPower(0);
//                    robot.intake.setPower(0);
//                }
////                else{
////                    robot.drop.setPower(0.65);
////
////                }
//                telemetry.addData("Waiting for Flag","");
//                telemetry.update();
//            }
//            telemetry.addData("Passed Flag","");
//            telemetry.update();
//            armOut.cancel();
//            dropMark.cancel();
//        }
//    }

    void composeTelemetry(){


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