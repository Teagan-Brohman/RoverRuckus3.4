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

@Autonomous(name = "BlueCraterOneBlock", group = "Autonomous")
//@Disabled
public class CraterOneBlock extends BaseAuto {
    public RoverHardware robot = new RoverHardware(); //Initialize our Hardware referenced in our hardware class


    private GoldAlignDetector detector; //Initialize detector used to search and find the golden block

    float angleTurn; //create a float variable used to store our current heading position while turning
    int blue; //create a integer variable used to store the amount of blue being registered from a color sensor
    int red; //create a integer variable used to store the amount of red being registered from a color sensor
    Timer waitTimer;
    int blockPosition;

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
        while(!opModeIsActive()){
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


        double Xpos = detector.getXPosition();
        telemetry.addData("XPos", Xpos);
        telemetry.update();
        //Block is located in the middle spot
        if(Xpos < 400){
            blockPosition = 2;//Block is located in the middle.
        }
        else if(Xpos > 400){
            blockPosition = 3;//Block is located in the right spot
        }
        else{
            blockPosition = 1;//block is left
        }

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

            if (opModeIsActive() == false){
                break;
            }
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
                } else if (detector.getXPosition() > 340){
                    robot.left1.setPower(.4);
                    robot.right1.setPower(-.4);
                }
            }
        } else {
            detector.disable();

            telemetry.addData("Status:", "no Block Seen");
            telemetry.update();

            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (robot.angles.firstAngle > 25 && opModeIsActive() || robot.angles.firstAngle < 35 && opModeIsActive()) {
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

//        //Lower intake and extend arm out
        telemetry.addData("heading", robot.angles.firstAngle);
        telemetry.update();

//        blockPosition = 2;

//        if(blockPosition == 1){
////            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            robot.left1.setTargetPosition(1700);//3400
////            robot.right1.setTargetPosition(1700);//3400
////            robot.left1.setPower(0.8);
////            robot.right1.setPower(0.8);
////            while (robot.left1.isBusy() && opModeIsActive()) {}
////
////            telemetry.update();
////            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            while (robot.angles.firstAngle > -59 && opModeIsActive() || (robot.angles.firstAngle < -70 && robot.angles.firstAngle < 0) && opModeIsActive()) {
//                robot.left1.setPower(Math.abs((-80 - robot.angles.firstAngle) / -58) * -0.4);
//                robot.right1.setPower(Math.abs((-80 - robot.angles.firstAngle) / -45) * 0.8);
//                telemetry.addData("left1 power", robot.left1.getPower());
//                telemetry.addData("right1 power", robot.right1.getPower());
//                telemetry.addData("heading", robot.angles.firstAngle);
//                telemetry.addData("angle var:", angleTurn);
//                telemetry.update();
//            }
//            robot.left1.setPower(0);
//            //robot.drop.setPower(0);
//            robot.right1.setPower(0);
//
//            //Drive forward and slightly into the wall
//            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.left1.setTargetPosition(-3900);//-7800
//        robot.right1.setTargetPosition(-3900);//-7800
//        robot.left1.setPower(-0.9 * 1.1);
//        robot.right1.setPower(-0.9);
//        while (robot.right1.isBusy() && robot.left1.isBusy() && opModeIsActive()) {}
//        robot.left1.setPower(-0.4);
//        robot.right1.setPower(-0.4);
//        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
//        telemetry.update();
//    }
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

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (robot.angles.firstAngle > -77 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                //This is a right turn to 78 degrees
                robot.left1.setPower(((-80 - angleTurn) / -48) * 0.3);
                robot.right1.setPower(((-80 - angleTurn) / -48) * -0.3);
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
            robot.left1.setTargetPosition(-2600);//5200
            robot.right1.setTargetPosition(-2600);//5200
            robot.left1.setPower(-0.9);
            robot.right1.setPower(-0.9);
            while (robot.left1.isBusy() && opModeIsActive()) {
            }
            robot.right1.setPower(0);
            robot.left1.setPower(0);

            //Turn Parallel with wall
            telemetry.update();
            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (robot.angles.firstAngle < -62 && opModeIsActive() || robot.angles.firstAngle > -58 && opModeIsActive()) {
                robot.left1.setPower(Math.abs((-60 - robot.angles.firstAngle) / -25) * -0.4);
                robot.right1.setPower(Math.abs((-60 - robot.angles.firstAngle) / -25) * 0.4);
                telemetry.addData("left1 power", robot.left1.getPower());
                telemetry.addData("right1 power", robot.right1.getPower());
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.addData("angle var:", angleTurn);
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
            robot.left1.setTargetPosition(-1800);//-6000
            robot.right1.setTargetPosition(-1900);//-6000
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
//            if(blockPosition == 3) {
//                robot.leftBop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.rightBop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.leftBop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.rightBop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.leftBop.setTargetPosition(-400);
//                robot.rightBop.setTargetPosition(-400);
//                robot.leftBop.setPower(-0.9);
//                robot.rightBop.setPower(-0.9);
//                while (robot.leftBop.isBusy() && robot.rightBop.isBusy() && opModeIsActive()) {
//                    telemetry.addData("encoder L", robot.leftBop.getCurrentPosition());
//                    telemetry.addData("power L", robot.leftBop.getPower());
//                    telemetry.addData("mode L", robot.leftBop.getMode());
//                    telemetry.addData("encoder R", robot.rightBop.getCurrentPosition());
//                    telemetry.addData("power R", robot.rightBop.getPower());
//                    telemetry.addData("mode R", robot.rightBop.getMode());
//                    telemetry.update();
//                }
//                robot.leftBop.setPower(0);
//                robot.rightBop.setPower(0);
//                //}
//                //Change the arm angle so it can hit the block
//                robot.rotateMech.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                sleep(100);
//                robot.rotateMech.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.rotateMech.setTargetPosition(180);
//                robot.rotateMech.setPower(0.8);
//                while (robot.rotateMech.getCurrentPosition() < 160 && opModeIsActive()) {
//                    telemetry.addData("rotate mech position", robot.rotateMech.getCurrentPosition());
//                    telemetry.update();
//                }
//                robot.rotateMech.setPower(0);
//                //}
////
////            robot.drop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            robot.drop.setTargetPosition(robot.BOTTOM_INTAKE);
////            robot.drop.setPower(-0.8);
////            while(robot.drop.getCurrentPosition() <= 180 && opModeIsActive()){
////                telemetry.addData("Drop Motor Power", robot.drop.getPower());
////                telemetry.addData("Drop Motor Position", robot.drop.getCurrentPosition());
////            }
////                while (robot.bottomDrop.getState()) {
////                    robot.drop.setPower(-0.65);
////                    telemetry.addData("bottomDrop", robot.bottomDrop.getState());
////                    telemetry.update();
////                }
////                robot.drop.setPower(0);
////
//
//                robot.leftBop.setTargetPosition(-1800);
//                robot.rightBop.setTargetPosition(-1800);
//                robot.leftBop.setPower(-0.6);
//                robot.rightBop.setPower(-0.6);
//                while (robot.leftBop.isBusy() && robot.rightBop.isBusy() && opModeIsActive()) {
//                    robot.intake.setPower(0);
//                    //Drive forward slightly
//                }
//                robot.leftBop.setTargetPosition(-1400);
//                robot.rightBop.setTargetPosition(-1400);
//                robot.leftBop.setPower(-0.9);
//                robot.rightBop.setPower(-0.9);
//                while (robot.leftBop.isBusy() && robot.rightBop.isBusy() && opModeIsActive()) {
//                }
//
//                robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                robot.leftBop.setTargetPosition(-150);
//                robot.rightBop.setTargetPosition(-150);
//                robot.leftBop.setPower(0.9);
//                robot.rightBop.setPower(0.9);
//                while (robot.leftBop.isBusy() && robot.rightBop.isBusy() && opModeIsActive()) {
////                    if (robot.topDrop.getState()) {
////                        robot.drop.setPower(.7);
////                    }
//                    robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    robot.left1.setTargetPosition(450);//900
//                    robot.right1.setTargetPosition(450);//900
//                    robot.left1.setPower(0.8);
//                    robot.right1.setPower(0.8);
//                    while (robot.left1.isBusy() && robot.right1.isBusy() && opModeIsActive()) {
//                        telemetry.addData("right power", robot.right1.getPower());
//                        telemetry.addData("right position", robot.right1.getCurrentPosition());
//                        telemetry.addData("left power", robot.left1.getPower());
//                        telemetry.addData("left position", robot.left1.getCurrentPosition());
//                        telemetry.update();
//                        if (opModeIsActive() == false){
//                            break;
//                        }
//                    }
//                }
//
//                robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                while (robot.angles.firstAngle > 73 && opModeIsActive() || robot.angles.firstAngle < 65 && opModeIsActive()) {
//                    angleTurn = robot.angles.firstAngle;
//                    //This is a right turn to 78 degrees
//                    robot.left1.setPower(((80 - angleTurn) / 50) * 0.6);
//                    robot.right1.setPower(((80 - angleTurn) / 50) * -0.6);
//                    telemetry.addData("left1 power", robot.left1.getPower());
//                    telemetry.addData("right1 power", robot.right1.getPower());
//                    telemetry.addData("heading", robot.angles.firstAngle);
//                    telemetry.addData("angle var:", angleTurn);
//                    telemetry.update();
//                }
//
//                //Drive to the team marker area
//                robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.left1.setTargetPosition(2600);//5200
//                robot.right1.setTargetPosition(2600);//5200
//                robot.left1.setPower(0.9);
//                robot.right1.setPower(0.9);
//
//
//                while (robot.left1.isBusy() && opModeIsActive() || robot.right1.isBusy() && opModeIsActive()) {
//                }
//                robot.right1.setPower(0);
//                robot.left1.setPower(0);
//
//////
//                //Turn Parallel with wall
//                telemetry.update();
//                robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                while (robot.angles.firstAngle > -59 && opModeIsActive() || (robot.angles.firstAngle < -70 && robot.angles.firstAngle < 0) && opModeIsActive()) {
//                    robot.left1.setPower(Math.abs((-80 - robot.angles.firstAngle) / -58) * -0.4);
//                    robot.right1.setPower(Math.abs((-80 - robot.angles.firstAngle) / -45) * 0.8);
//                    telemetry.addData("left1 power", robot.left1.getPower());
//                    telemetry.addData("right1 power", robot.right1.getPower());
//                    telemetry.addData("heading", robot.angles.firstAngle);
//                    telemetry.addData("angle var:", angleTurn);
//                    telemetry.update();
//                }
//                robot.left1.setPower(0);
//                //robot.drop.setPower(0);
//                robot.right1.setPower(0);
//
//                //Drive forward and slightly into the wall
//                robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.left1.setTargetPosition(-3000);//-3000
//                robot.right1.setTargetPosition(-3000);//-3000
//                robot.left1.setPower(-0.9 * 1.1);
//                robot.right1.setPower(-0.9);
//                while (robot.right1.isBusy() && robot.left1.isBusy() && opModeIsActive()) {}
//                robot.left1.setPower(-0.4);
//                robot.right1.setPower(-0.4);
//                robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
//                telemetry.update();
//            }
//
//            while(robot.cornerSensor.blue() <= 45 &&opModeIsActive()){
//                blue = robot.cornerSensor.blue();
//
//                robot.left1.setPower(-0.4* 1.03);
//                robot.right1.setPower(-0.4);
//                telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
//                telemetry.addData("Alpha", robot.cornerSensor.alpha());
//                telemetry.addData("BLUE", blue);
//                telemetry.addData("Searching", "");
//                telemetry.update();
//            }
//            robot.left1.setPower(0);
//            robot.right1.setPower(0);
//
//            //Turn and prep to drop off marker
////            telemetry.addData("Found!", "");
////            telemetry.update();
////            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////            while (robot.angles.firstAngle > 6 && opModeIsActive() || (robot.angles.firstAngle < -4 && robot.angles.firstAngle < 0) && opModeIsActive()) {
////                robot.left1.setPower(0);
////                robot.right1.setPower(-0.9);
////                telemetry.update();
////            }
////            robot.left1.setPower(0);
////            robot.right1.setPower(0);
//
//            //Drop Icon thing
//            robot.marker.setPosition(0.7);
//            sleep(500);
//            robot.marker.setPosition(0);
//
//
//            telemetry.update();
//            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.left1.setPower(0);
//            robot.right1.setPower(0);
//
//            //Backup into crater
//            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            robot.left1.setPower(-0.9 * 1.05);
//            robot.right1.setPower(-0.9);
//            sleep(4500);
//
//
//            while (robot.left1.isBusy() || robot.right1.isBusy() && opModeIsActive()) {}
//            robot.left1.setPower(0);
//            robot.right1.setPower(0);
//
    }

//    class secondwait extends TimerTask {
//        public void run(){
//            robot.drop.setPower(0);
//            robot.drop.setTargetPosition(robot.drop.getCurrentPosition());
//            robot.intake.setPower(0);
//            waitTimer.cancel();
//        }
//    }

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