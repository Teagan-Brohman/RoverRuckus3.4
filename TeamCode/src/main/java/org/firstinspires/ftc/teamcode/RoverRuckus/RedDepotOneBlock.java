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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.RevExtensions2;
import java.util.TimerTask;
import java.util.Timer;

import java.util.Locale;

@Autonomous(name = "RedDepotOneBlock", group = "Autonomous")
public class RedDepotOneBlock extends LinearOpMode {
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
        robot.hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.marker.setPosition(robot.DILBERT_UP);

        //Raise arm
        while (robot.upperLimit.red() > 110 && opModeIsActive()) {
            robot.hang.setPower(-1);
            telemetry.addData("Red Color", robot.upperLimit.red());
            telemetry.update();
        }
        robot.hang.setPower(0);
        sleep(200);
//
        telemetry.addData("BlockPosition", blockPosition);
        telemetry.update();

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
            if (Xpos < 400 && Xpos > 60){
                blockPosition = 2;//Block is located in the middle.
            } else if (Xpos > 400){
                blockPosition = 3;//Block is located in the right spot
            }
        }

        telemetry.addData("blockPosition", blockPosition);
        telemetry.update();

        if (detector.getAligned() == true && Xpos >= 100|| detector.getAligned() == false && Xpos >= 100) {
            //Hunt for the Block
//            if (blockPosition == 2){
//                robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                while (detector.getXPosition() < 235 && opModeIsActive() || detector.getXPosition() > 345 && opModeIsActive()) {
//                    telemetry.addData("Status", "searching for angle");
//                    telemetry.addData("xpos", detector.getXPosition());
//                    telemetry.addData("IsAligned", detector.getAligned());
//                    if (detector.getXPosition() < 235) {
//                        robot.left1.setPower(-.4);
//                        robot.right1.setPower(.4);
//                    } else if (detector.getXPosition() > 340) {
//                        robot.left1.setPower(.4);
//                        robot.right1.setPower(-.4);
//                    }
//                }
//            }
            if(blockPosition == 3) {
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
            while (robot.angles.firstAngle < 66.2 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(((69.2 - angleTurn) / 35) * -0.47);
                robot.right1.setPower(((69.2 - angleTurn) / 35) * 0.47);
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
            robot.left1.setTargetPosition(2700);//5200
            robot.right1.setTargetPosition(2700);//5200
            robot.left1.setPower(0.9);
            robot.right1.setPower(0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
            }
            robot.right1.setPower(0);
            robot.left1.setPower(0);

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle < 139 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(((144 - angleTurn) / 40) * -0.31);
                robot.right1.setPower(((144 - angleTurn) / 40) * 0.31);
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
//            float currentAngle = robot.angles.firstAngle;
//            telemetry.addData("heading", robot.angles.firstAngle);
//            telemetry.update();
//            while (robot.angles.firstAngle > (currentAngle - 1) && opModeIsActive()) {
//                angleTurn = robot.angles.firstAngle;
//                robot.left1.setPower(((-55 - angleTurn) / -8) * 0.05);
//                robot.right1.setPower(((-55 - angleTurn) / -8) * -0.05);
//                telemetry.addData("left1 power", robot.left1.getPower());
//                telemetry.addData("right1 power", robot.right1.getPower());
//                telemetry.addData("heading", robot.angles.firstAngle);
//                telemetry.addData("angle var:", angleTurn);
//                telemetry.update();
//            }

            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(1450);//2500
            robot.right1.setTargetPosition(1450);//2500
            robot.left1.setPower(0.9);
            robot.right1.setPower(0.9);
            while (robot.right1.isBusy() && opModeIsActive()) {
                telemetry.addData("left1", robot.left1.getCurrentPosition());
                telemetry.update();
            }

            robot.left1.setTargetPosition(400);
            robot.right1.setTargetPosition(400);
            robot.left1.setPower(-0.4);
            robot.right1.setPower(-0.4);
            while (robot.left1.isBusy() && opModeIsActive()){}
            robot.left1.setPower(0);
            robot.right1.setPower(0);

            sleep(500);

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle < 58 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(((66 - angleTurn) / 53) * -0.36);
                robot.right1.setPower(((66 - angleTurn) / 53) * 0.36);
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
            robot.left1.setTargetPosition(3500);//5200
            robot.right1.setTargetPosition(3500);//5200
            robot.left1.setPower(0.9);
            robot.right1.setPower(0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
            }
            robot.right1.setPower(0);
            robot.left1.setPower(0);

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle < 136 && opModeIsActive()) {
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
            robot.left1.setTargetPosition(-1875);//5200
            robot.right1.setTargetPosition(-1875);//5200
            robot.left1.setPower(-0.9);
            robot.right1.setPower(-0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
            }
            robot.right1.setPower(0);
            robot.left1.setPower(0);
        }

        if (blockPosition == 3) {
            float currentAngle = robot.angles.firstAngle;
            while (robot.angles.firstAngle > (currentAngle - 5) && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                robot.left1.setPower(((-45 - angleTurn) / -10) * 0.3);
                robot.right1.setPower(((-45 - angleTurn) / -10) * -0.3);
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
            robot.left1.setTargetPosition(2300);//2500
            robot.right1.setTargetPosition(2300);//2500
            robot.left1.setPower(0.9);
            robot.right1.setPower(0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
                telemetry.addData("left1", robot.left1.getCurrentPosition());
                telemetry.update();
            }

            robot.left1.setTargetPosition(550);//2500
            robot.right1.setTargetPosition(550);//2500
            robot.left1.setPower(0.9);
            robot.right1.setPower(0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
                telemetry.addData("left1", robot.left1.getCurrentPosition());
                telemetry.update();
            }

            while (robot.left1.isBusy() && opModeIsActive()) {}
            robot.right1.setPower(0);
            robot.left1.setPower(0);

            sleep(500);

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle > -97 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(((-95 - angleTurn) / -45) * 0.36);
                robot.right1.setPower(((-95 - angleTurn) / -45) * -0.36);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
                telemetry.update();
            }
//
            robot.right1.setPower(0);
            robot.left1.setPower(0);
            sleep(500);

            //Drive to the team marker area
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left1.setTargetPosition(-2700);//5200
            robot.right1.setTargetPosition(-2700);//5200
            robot.left1.setPower(-0.9);
            robot.right1.setPower(-0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {}
            robot.right1.setPower(0);
            robot.left1.setPower(0);

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle < -20 && opModeIsActive()){
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(Math.abs((0 - angleTurn) / 150) * 0.25);
                robot.right1.setPower(Math.abs((0 - angleTurn) / 150) * -0.25);
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
            robot.left1.setPower(-0.6 * 1.2);
            robot.right1.setPower(-0.6);
            while (robot.left1.isBusy() && opModeIsActive()) {}

        }

        robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Color Sensor RED", robot.cornerSensor.red());
        while(robot.cornerSensor.red() <= 40 && opModeIsActive()){
            robot.left1.setPower(-0.3);
            robot.right1.setPower(-0.3);
            telemetry.addData("Color Sensor RED", robot.cornerSensor.red());
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

        robot.marker.setPosition(robot.DILBERT_DOWN);
        sleep(1000);

        robot.marker.setPosition(robot.DILBERT_UP);

        int encoderticksR = 0;
        int encoderticksL = 0;
        robot.right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("range", String.format("%.01f cm", robot.frontDetect.getDistance(DistanceUnit.CM)));
        telemetry.update();
        while(robot.frontDetect.getDistance(DistanceUnit.CM) > 2 && opModeIsActive() && robot.left1.getCurrentPosition() < 4000){
            telemetry.addData("EncoderR", robot.right1.getCurrentPosition());
            telemetry.addData("EncoderL", robot.left1.getCurrentPosition());
            encoderticksL = robot.left1.getCurrentPosition();
            encoderticksR = robot.right1.getCurrentPosition();
            telemetry.addData("ticksR", encoderticksR);
            telemetry.addData("ticksL", encoderticksL);
            robot.left1.setPower(0.9 * 1.1 / ((encoderticksL / 1500.083)));
            robot.right1.setPower(0.9 / ((encoderticksR / 1500.083)));
            telemetry.addData("Distance", robot.frontDetect.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

//        //Hold position
//        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        int currentPosL = robot.left1.getCurrentPosition();
//        int currentPosR = robot.right1.getCurrentPosition();
//        robot.left1.setTargetPosition(currentPosL);
//        robot.right1.setTargetPosition(currentPosR);
//        robot.right1.setPower(0.2);
//        robot.left1.setPower(0.2);

//        robot.leftBop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.rightBop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.leftBop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.rightBop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.leftBop.setPower(-0.4);
//        robot.rightBop.setPower(-0.4);
//        robot.intake.setPower(0.9);
//        sleep(1250);
//
//        robot.rightBop.setPower(0);
//        robot.leftBop.setPower(0);
//        sleep(1000);

        robot.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("hang position", robot.hang.getCurrentPosition());
        telemetry.update();
        while(robot.hang.getCurrentPosition() > -4600 && opModeIsActive()){
            telemetry.addData("hang position", robot.hang.getCurrentPosition());
            telemetry.update();
            robot.hang.setPower(.99);
        }
        robot.hang.setPower(0);
    }

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