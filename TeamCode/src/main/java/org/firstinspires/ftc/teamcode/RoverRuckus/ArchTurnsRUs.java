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
import org.firstinspires.ftc.teamcode.RelicRecovery.RRHardwarePresets;
import org.openftc.revextensions2.RevExtensions2;

import java.util.Locale;
import java.util.Timer;

@Autonomous(name = "ArchTurn", group = "Autonomous")
public class ArchTurnsRUs extends LinearOpMode {
//    RRHardwarePresets robot = new RRHardwarePresets();
//
//    public void runOpMode() {
//
//        waitForStart();
//
//        startTurn();
//    }
//    void startTurn(int counts, double speed, double turnFraction, boolean turnRight) {
//
//        double reducedSpeed = speed * (1 - turnFraction);
//        int reducedCounts = counts * (1- turnFraction);
//
//        if (turnRight) {
//            robot.left1.setTargetPosition(counts);
//            robot.left1.setPower(speed);
//            robot.right1.setTargetPosition(counts - 500);
//            robot.right1.setPower(reducedSpeed);
//        }
//        else {
//            robot.left1.setTargetPosition(reducedCounts);
//            robot.left1.setPower(reducedSpeed);
//            robot.right1.setTargetPosition(counts - );
//            robot.right1.setPower(speed);
//        }
//    }
public RoverHardware robot = new RoverHardware(); //Initialize our Hardware referenced in our hardware class

    private GoldAlignDetector detector; //Initialize detector used to search and find the golden block

    float angleTurn; //create a float variable used to store our current heading position while turning
    int blue; //create a integer variable used to store the amount of blue being registered from a color sensor
    int red; //create a integer variable used to store the amount of red being registered from a color sensor
    Timer waitTimer;
    int blockPosition;
    int timer;

    double POWER = 1.15;

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

        //detector.enable();

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
            // telemetry.addData("DetectorXPos", detector.getXPosition());
            telemetry.update();
//            if(detector.getXPosition() < 50){
//                blockPosition = 1;
//            }
        }
        waitForStart();

        detector.enable();

        robot.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.marker.setPosition(robot.DILBERT_UP );

        //Raise arm
        while (robot.upperLimit.red() > 150 && opModeIsActive()) {
            robot.hang.setPower(-1);
            telemetry.addData("Red Color", robot.upperLimit.red());
            telemetry.update();
        }
        robot.hang.setPower(0);
        sleep(200);

        telemetry.addData("BlockPosition", blockPosition);
        telemetry.update();

        //Drive forward slightly
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(450);//900
        robot.right1.setTargetPosition(450);//900
        robot.left1.setPower(0.4);
        robot.right1.setPower(0.4);
        while (robot.left1.isBusy() && robot.right1.isBusy() && opModeIsActive()) {
            telemetry.addData("XPos", detector.getXPosition());
            telemetry.update();

            if (opModeIsActive() == false) {
                break;
            }
        }

        detector.enable();

        telemetry.addData("detectorPosition", detector.getXPosition());
        telemetry.update();

        double Xpos = detector.getXPosition();
        telemetry.addData("XPos", Xpos);
        telemetry.update();

        if(detector.getXPosition() < 50 && Xpos < 50){
            blockPosition = 1;
        }
        else if(detector.getAligned() == true || detector.getAligned() == false) {
            //Block is located in the middle spot
            if (Xpos < 400 && Xpos > 160) {
                blockPosition = 2;//Block is located in the middle.
            } else if (Xpos > 400) {
                blockPosition = 3;//Block is located in the right spot
            }
        }

        telemetry.addData("blockPosition", blockPosition);
        telemetry.update();

        if (detector.getAligned() == true && Xpos >= 100|| detector.getAligned() == false && Xpos >= 100) {
            //Hunt for the Block
            if (blockPosition == 2){
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
            }
            else if(blockPosition == 3) {
                robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.left1.setTargetPosition(-200);//900
                robot.right1.setTargetPosition(-200);//900
                robot.left1.setPower(0.5);
                robot.right1.setPower(0.5);
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
            }
        } else if (blockPosition == 1){
            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("status", "entered block1");
            telemetry.update();
            while (detector.getXPosition() < 235 && opModeIsActive() || detector.getXPosition() > 345 && opModeIsActive()) {
                telemetry.addData("Status", "searching for angle");
                telemetry.addData("xpos", detector.getXPosition());
                telemetry.addData("IsAligned", detector.getAligned());
                telemetry.addData("status", "entered loop");
                telemetry.update();
                robot.left1.setPower(-.4);
                robot.right1.setPower(.4);
            }
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);
        sleep(100);

        telemetry.addData("heading", robot.angles.firstAngle);
        telemetry.update();

        if(blockPosition == 1){
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(1900);//3400
            robot.right1.setTargetPosition(1900);//3400
            robot.left1.setPower(0.8);
            robot.right1.setPower(0.8);
            while (robot.left1.isBusy() && opModeIsActive()) {}

            robot.left1.setTargetPosition(800);//3400
            robot.right1.setTargetPosition(800);//3400
            robot.left1.setPower(0.8);
            robot.right1.setPower(0.8);
            while (robot.left1.isBusy() && opModeIsActive()) {}

            robot.right1.setPower(0);
            robot.left1.setPower(0);

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle > -94 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(((-90 - angleTurn) / -48) * 0.3);
                robot.right1.setPower(((-90 - angleTurn) / -48) * -0.3);
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
            robot.left1.setTargetPosition(-2200);//5200
            robot.right1.setTargetPosition(-2200);//5200
            robot.left1.setPower(-0.9);
            robot.right1.setPower(-0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
            }
            robot.right1.setPower(0);
            robot.left1.setPower(0);

            //Turn Parallel with wall
            telemetry.update();
            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            timer = 0;
            while (robot.angles.firstAngle < -75 && opModeIsActive()) {
                robot.left1.setPower(Math.abs((-75 - robot.angles.firstAngle) / -40) * -0.4);
                robot.right1.setPower(Math.abs((-75 - robot.angles.firstAngle) / -40) * 0.4);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
                telemetry.addData("Timer", timer);
                telemetry.update();
            }
            robot.left1.setPower(0);
            robot.right1.setPower(0);
////
            // Drive forward and slightly into the wall
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(-1400);//-6000
            robot.right1.setTargetPosition(-1400);//-6000
            robot.left1.setPower(-0.9 * 1.1);
            robot.right1.setPower(-0.9);
            while (robot.right1.isBusy() && robot.left1.isBusy() && opModeIsActive()) {}
            robot.left1.setPower(-0.4);
            robot.right1.setPower(-0.4);
            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
            telemetry.update();
        }
        if (blockPosition == 2) {
            float currentAngle = robot.angles.firstAngle;
            telemetry.addData("heading", robot.angles.firstAngle);
            telemetry.update();
            while (robot.angles.firstAngle > (currentAngle - 1) && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                robot.left1.setPower(((-55 - angleTurn) / -8) * 0.05);
                robot.right1.setPower(((-55 - angleTurn) / -8) * -0.05);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
                telemetry.update();
            }

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

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle > -94 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(((-90 - angleTurn) / -48) * 0.3);
                robot.right1.setPower(((-90 - angleTurn) / -48) * -0.3);
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
            robot.left1.setTargetPosition(-1000);//5200
            robot.right1.setTargetPosition(-1000);//5200
            robot.left1.setPower(-0.9);
            robot.right1.setPower(-0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
            }

            //Turn Parallel with wall
            telemetry.update();
            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robot.right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            while (robot.angles.firstAngle < -75 && opModeIsActive()) {
                robot.left1.setPower(-0.9);
                robot.right1.setPower(-0.45);
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.update();
            }
            robot.left1.setPower(0);
            robot.right1.setPower(0);
////
            // Drive forward and slightly into the wall
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(-1600);//-6000
            robot.right1.setTargetPosition(-1600);//-6000
            robot.left1.setPower(-0.9 * 1.1);
            robot.right1.setPower(-0.9);
            while (robot.right1.isBusy() && robot.left1.isBusy() && opModeIsActive()) {
                robot.marker.setPosition(robot.DILBERT_DOWN);
            }
            robot.left1.setPower(-0.4);
            robot.right1.setPower(-0.4);
            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
            telemetry.update();
            robot.marker.setPosition(robot.DILBERT_UP);
        }
        if(blockPosition == 3) {

            double currentAngle = robot.angles.firstAngle;
            telemetry.addData("heading", robot.angles.firstAngle);
            telemetry.update();
            while (robot.angles.firstAngle > (currentAngle - 3) && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                robot.left1.setPower(((-55 - angleTurn) / -10) * 0.2);
                robot.right1.setPower(((-55 - angleTurn) / -10) * -0.2);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
                telemetry.update();
            }
//
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(1900);//2500
            robot.right1.setTargetPosition(1900);//2500
            robot.left1.setPower(0.9);
            robot.right1.setPower(0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
                telemetry.addData("left1", robot.left1.getCurrentPosition());
                telemetry.update();
            }

            robot.left1.setTargetPosition(650);//2500
            robot.right1.setTargetPosition(650);//2500
            robot.left1.setPower(0.9);
            robot.right1.setPower(0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
                telemetry.addData("left1", robot.left1.getCurrentPosition());
                telemetry.update();
            }

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle > -94 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(((-95 - angleTurn) / -40) * 0.3);
                robot.right1.setPower(((-95 - angleTurn) / -40) * -0.3);
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
            robot.left1.setTargetPosition(-2700);//5200
            robot.right1.setTargetPosition(-2700);//5200
            robot.left1.setPower(-0.9);
            robot.right1.setPower(-0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
            }
            robot.right1.setPower(0);
            robot.left1.setPower(0);

            //Turn Parallel with wall
            telemetry.update();
            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            timer = 0;
            while (robot.angles.firstAngle < -71 && opModeIsActive()) {
                robot.left1.setPower(Math.abs((-75 - robot.angles.firstAngle) / -40) * -0.4);
                robot.right1.setPower(Math.abs((-75 - robot.angles.firstAngle) / -40) * 0.4);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
                telemetry.addData("Timer", timer);
                telemetry.update();
            }
            robot.left1.setPower(0);
            robot.right1.setPower(0);
////
            // Drive forward and slightly into the wall
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(-2000);//-6000
            robot.right1.setTargetPosition(-2000);//-6000
            robot.left1.setPower(-0.9 * 1.1);
            robot.right1.setPower(-0.9);
            while (robot.right1.isBusy() && robot.left1.isBusy() && opModeIsActive()) {
            }
            robot.left1.setPower(-0.4);
            robot.right1.setPower(-0.4);
            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Color Sensor RED", robot.cornerSensor.red());
            telemetry.update();
        }

        robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
        while(robot.cornerSensor.blue() <= 33  && opModeIsActive()){
            robot.left1.setPower(-0.3);
            robot.right1.setPower(-0.3);
            telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

//        robot.marker.setPosition(robot.DILBERT_DOWN);
//        sleep(1000);
//
//        robot.marker.setPosition(robot.DILBERT_UP);

        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(4400);
        robot.right1.setTargetPosition(4400);
        robot.left1.setPower(0.9);
        robot.right1.setPower(0.9);
        while (robot.left1.isBusy() && opModeIsActive()) {}

        robot.hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.hang.setTargetPosition(-7900);
        robot.hang.setPower(1);
        while(robot.hang.isBusy()){
            telemetry.addData("hanger position", robot.hang.getCurrentPosition());
            telemetry.update();
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