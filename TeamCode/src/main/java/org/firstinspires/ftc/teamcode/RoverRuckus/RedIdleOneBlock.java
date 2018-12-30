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
import java.util.TimerTask;
import java.util.Timer;

import java.util.Locale;

@Autonomous(name = "RedDepotOneBlock", group = "Autonomous")
public class RedIdleOneBlock extends LinearOpMode {
    public RoverHardware robot = new RoverHardware(); //Create a new instance of the

    //Create detector gateway
    private GoldAlignDetector detector;

    //Create Variables
    float angleTurn;
    int blue;
    Timer waitTimer;

    public void runOpMode() {robot.init(hardwareMap);

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
        while(!opModeIsActive()){//In the initializing phase we can look at the pitch heading and roll to setup
            telemetry.update();
        }
        waitForStart();//waits for the start button to be pressed

        //Raise arm
        while (robot.upperLimit.red() > 300 && opModeIsActive()) {
            robot.hang.setPower(1);
        }
        robot.hang.setPower(0);
        sleep(200);

        //Drive forward slightly
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(450);//900
        robot.right1.setTargetPosition(450);
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

        if (detector.getAligned() == true || detector.getAligned() == false) {
            //Hunt for the Block
            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (detector.getXPosition() < 235 && opModeIsActive() || detector.getXPosition() > 345 && opModeIsActive()) {
                telemetry.addData("Status", "searching for angle");
                telemetry.addData("xpos", detector.getXPosition());
                telemetry.addData("IsAligned", detector.getAligned());
                if (detector.getXPosition() < 235) {
                    robot.left1.setPower(.8);
                    robot.right1.setPower(-.8);
                } else if (detector.getXPosition() > 340) {
                    robot.left1.setPower(-.8);
                    robot.right1.setPower(.8);
                }
            }
        } else {
            robot.angles.firstAngle = angleTurn;

            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (robot.angles.firstAngle > 12 && opModeIsActive() || robot.angles.firstAngle < 23 && opModeIsActive()) {
                angleTurn = robot.angles.firstAngle;
                robot.left1.setPower(Math.abs((18 - angleTurn) / 47) * 0.5);
                robot.right1.setPower(Math.abs((18 - angleTurn) / 47) * -0.5);
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


//        //Lower intake and extend arm out
        telemetry.addData("heading", robot.angles.firstAngle);
        telemetry.update();
        if (robot.angles.firstAngle < 10 && robot.angles.firstAngle > -10) {
            robot.bop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bop.setTargetPosition(-600);
            robot.bop.setPower(-0.6);
            while (robot.bop.isBusy() && opModeIsActive()) {
                telemetry.addData("encoder", robot.bop.getCurrentPosition());
                telemetry.addData("power", robot.bop.getPower());
                telemetry.addData("mode", robot.bop.getMode());
                telemetry.update();
            }
            robot.bop.setPower(0);
            //}
            //Change the arm angle so it can hit the block
            //if (robot.angles.firstAngle > 2) {
//            robot.drop.setPower(0);
//            robot.drop.setTargetPosition(0);
//            if(robot.angles.firstAngle < 25) {
            robot.rotateMech.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rotateMech.setTargetPosition(180);
            robot.rotateMech.setPower(0.8);
//            while (robot.rotateMech.isBusy()){
//            }
            //}
//
            robot.drop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.drop.setTargetPosition(robot.BOTTOM_INTAKE);
            robot.drop.setPower(-0.8);
            while(robot.drop.getCurrentPosition() <= -180 && opModeIsActive()){
                telemetry.addData("Drop Motor Power", robot.drop.getPower());
                telemetry.addData("Drop Motor Position", robot.drop.getCurrentPosition( ));
            }
            sleep(300);
            robot.intake.setPower(0.9);
//
            robot.bop.setTargetPosition(-1800);
            robot.bop.setPower(-0.6);
            while (robot.bop.isBusy() && opModeIsActive()) {
                robot.intake.setPower(0.9);
            }

        }
        else{
            robot.bop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.bop.setTargetPosition(-600);
            robot.bop.setPower(-0.8);
            while (robot.bop.getCurrentPosition() < -580 && opModeIsActive()) {
                telemetry.addData("encoder", robot.bop.getCurrentPosition());
                telemetry.addData("power", robot.bop.getPower());
                telemetry.addData("mode", robot.bop.getMode());
                telemetry.update();
            }
            robot.bop.setPower(0);
            telemetry.update();
            //}
            //Change the arm angle so it can hit the block
            //if (robot.angles.firstAngle > 2) {
//            robot.drop.setPower(0);
//            robot.drop.setTargetPosition(0);

            if(robot.angles.firstAngle < 25){
                robot.rotateMech.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rotateMech.setTargetPosition(120);
                robot.rotateMech.setPower(0.8);
                while (robot.rotateMech.isBusy() && opModeIsActive()){
                }
            }
//
            robot.drop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.drop.setTargetPosition(robot.BOTTOM_INTAKE);
            robot.drop.setPower(-0.8);
            while(robot.drop.getCurrentPosition() <= -150 && opModeIsActive() ){
                telemetry.addData("Drop Motor Power", robot.drop.getPower());
                telemetry.addData("Drop Motor Position", robot.drop.getCurrentPosition( ));
                telemetry.update();
            }
            robot.intake.setPower(0.9);
//
            if(robot.angles.firstAngle < 40) {
                robot.bop.setTargetPosition(-1750);
                robot.bop.setPower(-0.8);
                while (robot.bop.isBusy() && opModeIsActive()) {
                    robot.intake.setPower(0.9);
                }
            } else{
                robot.bop.setTargetPosition(-2000);
                robot.bop.setPower(-0.8);
                while (robot.bop.isBusy() && opModeIsActive()) {
                    robot.intake.setPower(0.9);
                }
            }
        }

        //bring arm back in
//        robot.bop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.bop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.bop.setTargetPosition(-50);
//        robot.bop.setPower(0.9);
//        while (robot.bop.isBusy() && opModeIsActive()){
//            robot.intake.setPower(0.9);
//        }
        robot.bop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (robot.bopLimit.red() <= 130 && robot.bopLimit.alpha() < 300 && opModeIsActive()){
            robot.bop.setPower(0.7);
            robot.rotateMech.setPower(-0.2);
            robot.rotateMech.setTargetPosition(0);
        }
        robot.bop.setPower(0);
        robot.rotateMech.setPower(0);

        robot.drop.setTargetPosition(robot.TOP_INTAKE);
        robot.drop.setPower(0.8);



        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(600);//1200
        robot.right1.setTargetPosition(600);
        robot.left1.setPower(0.8);
        robot.right1.setPower(0.8);

//        robot.drop.setTargetPosition(robot.drop.getCurrentPosition());
//        robot.drop.setPower(0);
        waitTimer = new Timer();
        waitTimer.schedule(new secondwait(), 3000,100);

        while(robot.left1.getCurrentPosition() < 500 && opModeIsActive()){

        }

        //turn right
        telemetry.update();
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (robot.angles.firstAngle > -70 && opModeIsActive() || (robot.angles.firstAngle < -90 && robot.angles.firstAngle < 0) && opModeIsActive()) {
            robot.left1.setPower(Math.abs((-80 - robot.angles.firstAngle) / -40) * -0.8);
            robot.right1.setPower(Math.abs((-80 - robot.angles.firstAngle) / -40) * 0.8);
            telemetry.addData("left1 power", robot.left1.getPower());
            telemetry.addData("right1 power", robot.right1.getPower());
            telemetry.addData("heading", robot.angles.firstAngle);
            telemetry.addData("angle var:", angleTurn);
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

        //Drive to other blocks
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(2950);//5900
        robot.right1.setTargetPosition(2950);
        robot.left1.setPower(0.8);
        robot.right1.setPower(0.8);
        while (robot.left1.isBusy() && opModeIsActive() || robot.right1.isBusy() && opModeIsActive()) {
            robot.intake.setPower(0.9);
        }
        robot.right1.setPower(0);
        robot.left1.setPower(0);
        robot.intake.setPower(0);

        //turn right
        telemetry.update();
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (robot.angles.firstAngle > -150 && opModeIsActive() || (robot.angles.firstAngle < -170 && robot.angles.firstAngle < 0) && opModeIsActive()) {
            robot.left1.setPower(Math.abs((-160 - robot.angles.firstAngle) / -35) * -0.8);
            robot.right1.setPower(Math.abs((-160 - robot.angles.firstAngle) / -35) * 0.8);
            telemetry.addData("left1 power", robot.left1.getPower());
            telemetry.addData("right1 power", robot.right1.getPower());
            telemetry.addData("heading", robot.angles.firstAngle);
            telemetry.addData("angle var:", angleTurn);
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);
//
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(-800);//-1600
        robot.right1.setTargetPosition(-800);
        robot.left1.setPower(-0.8 * 1.1);
        robot.right1.setPower(-0.8);
        while (robot.left1.isBusy() && opModeIsActive()) {}
        robot.left1.setPower(-0.8 * 1.1);
        robot.right1.setPower(-0.8);
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
        telemetry.update();

        while(robot.cornerSensor.red() <= 60 &&opModeIsActive()){

            robot.left1.setPower(-0.6);
            robot.right1.setPower(-0.6);
            telemetry.addData("Color Sensor Red", robot.cornerSensor.red());
            telemetry.addData("Alpha", robot.cornerSensor.alpha());
            telemetry.addData("Red", blue);
            telemetry.addData("Searching", "");
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

        //Turn and prep to drop off marker
        telemetry.update();
        robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (robot.angles.firstAngle > -74 && opModeIsActive() || (robot.angles.firstAngle < -97 && robot.angles.firstAngle < 0) && opModeIsActive()) {
            robot.left1.setPower(Math.abs((-84 - robot.angles.firstAngle) / -38) * 0.8);
            robot.right1.setPower(Math.abs((-84 - robot.angles.firstAngle) / -38) * -0.8);
            telemetry.update();
        }
        robot.left1.setPower(0);
        robot.right1.setPower(0);

        robot.marker.setPosition(robot.DILBERT_DOWN);
        sleep(500);
        robot.marker.setPosition(robot.DILBERT_UP);


        while (robot.angles.firstAngle > -65 && opModeIsActive() || (robot.angles.firstAngle < -90 && robot.angles.firstAngle < 0) && opModeIsActive()) {
            robot.left1.setPower(Math.abs((-70 - robot.angles.firstAngle) / -40) * 0.8);
            robot.right1.setPower(Math.abs((-70 - robot.angles.firstAngle) / -40) * -0.8);
            telemetry.update();
        }
//        robot.left1.setPower(-0.9);
//        robot.right1.setPower(-0.75);
//
//        sleep(2000);

        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(-500);
        robot.right1.setTargetPosition(-500);//-1000
        robot.left1.setPower(-0.9);
        robot.right1.setPower(-0.9);


        robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.left1.setTargetPosition(-4650);//-9300
        robot.right1.setTargetPosition(-4650);
        robot.left1.setPower(-0.9 * 1.2);
        robot.right1.setPower(-0.9);
        while (robot.left1.isBusy() && opModeIsActive()) {}
    }

    class secondwait extends TimerTask{
        public void run(){
            robot.drop.setPower(0);
            robot.drop.setTargetPosition(robot.drop.getCurrentPosition());
            robot.intake.setPower(0);
            waitTimer.cancel();
        }
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
