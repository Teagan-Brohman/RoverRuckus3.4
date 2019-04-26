package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;

@TeleOp(name = "DepotTeleop", group = "Teleop")
public class DepotTeleop extends LinearOpMode {
    public RoverHardware robot = new RoverHardware();

    public float leftPower;
    public float rightPower;
    public float xValue;
    public float yValue;
    public double controlSpeed;
    float angleTurn;
    float offsetHeading;
    float targetHeading;

    ExpansionHubEx expansionHub;
    Timer stopMotor;
    Timer sorterOut;
    Timer intakeOut;
    Timer currentReg;
    Timer currentLim;
    Timer IntakeR;
    Timer SpinOut;
    Timer DoorDrop;

    ExpansionHubMotor left, right;
    RevBulkData bulkData;
    boolean buttonFlag;
    boolean limitFlag = false;
    boolean sorterFlag;
    boolean rotateFlag;
    boolean intakeFlag;
    boolean intakeReverse = true;
    boolean DoorFlag = true;
    boolean backUpFlag = true;
    public int currentDiv = 1;


    DistanceSensor lidar;

    public void runOpMode() {
        robot.init(hardwareMap);


        RevExtensions2.init();

        lidar = hardwareMap.get(DistanceSensor.class, "lidar");


        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        left = (ExpansionHubMotor) hardwareMap.dcMotor.get("left1");
        right = (ExpansionHubMotor) hardwareMap.dcMotor.get("right1");

////        //Initialize Gyro
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.sorter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        robot.imu = hardwareMap.get(BNO055IMU.class, "imu");
        robot.imu.initialize(parameters1);

        robot.initServoPositions();
        robot.rotateMech.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rotateMech.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.drop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        composeTelemetry();

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
                    }
                });

        waitForStart();


        while (opModeIsActive()) {
//            if (limitFlag = false) {
////                currentLim = new Timer();
////                currentLim.schedule(new SecondRobotTeleop.CurrentLim(), 20000, 10);
////            }


            yValue = gamepad1.right_stick_y;
            xValue = -gamepad1.right_stick_x;



//            if( lidar.getDistance(DistanceUnit.CM) > 800){
//                telemetry.addData("Is LIDAR mad/broken: Yes","");
//            }
//            else{
//                telemetry.addData("Is LIDAR mad/broken: No","");
//            }

            //GAMEPAD 1
            //Arcade Style Drive Motors
//            if(Math.abs(gamepad1.right_stick_y) > 0.2|| Math.abs(gamepad1.right_stick_x) > 0.2){
//                yValue = gamepad1.right_stick_y;
//                xValue = -gamepad1.right_stick_x;
//            }
//
//            else if(Math.abs(gamepad1.left_stick_y) > 0.2|| Math.abs(gamepad1.left_stick_x) > 0.2) {
//                yValue = gamepad1.left_stick_y;
//                xValue = -gamepad1.left_stick_x;
//            }
//            else{
//                yValue = 0;
//                xValue = 0;
//            }


            leftPower = yValue - xValue;
            rightPower = yValue + xValue;

            if(Math.abs(gamepad1.right_stick_y) > 0.2 || Math.abs(gamepad1.right_stick_x) > 0.2){
                robot.left1.setPower(Range.clip(leftPower, -1.0, 1.0));
                robot.right1.setPower(Range.clip(rightPower, -1.0, 1.0));
            }
            else{
                robot.left1.setPower(0);
                robot.right1.setPower(0);
            }

            //Arm that shoots blocks and balls
            if (Math.abs(gamepad2.right_stick_y) >= 0.2) {
                buttonFlag = true;
            }


            if (gamepad1.right_bumper) {
                sorterFlag = false;
                robot.sorter.setPower(.99);
            } else if (gamepad1.left_bumper) {
                sorterFlag = false;
//                robot.sorter.setPower(-1.0);
                robot.sorterFlip.setPosition(robot.SORTER_UP);
                if (robot.sorterLidar.getDistance(DistanceUnit.CM) > 70) {
                    robot.sorter.setPower(-1.0);
                } else if (robot.sorterLidar.getDistance(DistanceUnit.CM) <= 70 && robot.sorterLidar.getDistance(DistanceUnit.CM) > 20) {
                    robot.sorter.setPower(-0.75);
                } else if (robot.sorterLidar.getDistance(DistanceUnit.CM) <= 20 && robot.sorterLidar.getDistance(DistanceUnit.CM) > 4) {
                    robot.sorter.setPower(-0.6);
                } else if (robot.sorterLidar.getDistance(DistanceUnit.CM) < 2.5) {
                    robot.sorter.setPower(-0.1);
                }
                else if(robot.sorterLidar.getDistance(DistanceUnit.CM) > 500) {
                    if (gamepad1.right_bumper) {
                        sorterFlag = false;
                        robot.sorter.setPower(.99);
                    }
                    if(gamepad1.left_bumper){
                        sorterFlag = false;
                        robot.sorter.setPower(-.99);
                    }
                }
                else {
                    robot.sorter.setPower(0);
                }

            }
            else {
                if(robot.sorterLidar.getDistance(DistanceUnit.CM) < 15){
                    robot.sorter.setPower(-0.25);
                }
                else{
                    robot.sorter.setPower(0);
                }
            }

            if (gamepad1.a) {
                robot.sorterFlip.setPosition(robot.SORTER_DOWN);
            }
            if (gamepad1.b) {
                robot.sorterFlip.setPosition(robot.SORTER_UP);
            }

            //Hanging Mechanism
            if (gamepad1.dpad_up/* && robot.upperLimit.red() < 300 && robot.upperLimit.alpha() > 230*/) {
                robot.hang.setPower(-1);
            } else if (gamepad1.dpad_down /* && robot.bottomLimit.red() < 100*/) {
                robot.hang.setPower(1);
            } else {
                robot.hang.setPower(0);
            }

            if (gamepad1.x) {
                sorterFlag = true;
            }

            if (sorterFlag == true) {
                robot.sorter.setPower(-0.9);
                robot.sorterFlip.setPosition(robot.SORTER_UP);
                if (robot.sorterLidar.getDistance(DistanceUnit.CM) < 5) {
                    sorterFlag = false;
                    robot.sorter.setPower(0);
                } else if (robot.sorterLidar.getDistance(DistanceUnit.CM) <= 70 && robot.sorterLidar.getDistance(DistanceUnit.CM) > 5) {
                    robot.sorter.setPower(-0.65);
                }
                if (gamepad1.right_bumper) {
                    sorterFlag = false;
                    robot.sorter.setPower(.99);
                }

            }

            if(gamepad1.right_trigger > 0.5){ //target shuld be 45
                robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.update();
                while(robot.angles.firstAngle > -130) {//45
                    robot.left1.setPower(0.8);
                    robot.right1.setPower(-0.8);
                    telemetry.addData("heading", robot.angles.firstAngle);
                    telemetry.update();
                }
                robot.left1.setPower(-0.18);
                robot.right1.setPower(0.18);
                telemetry.addData("heading", robot.angles.firstAngle);
                telemetry.update();
                while(robot.angles.firstAngle < 140 && opModeIsActive() || robot.angles.firstAngle > -120 && opModeIsActive()){
                    telemetry.addData("heading", robot.angles.firstAngle);
                    telemetry.update();
                }
                robot.left1.setPower(0);
                robot.right1.setPower(0);
            }
            //TODO find correct angles, find correct powers

             if(gamepad1.left_trigger > 0.5){ //target should be -138
                 robot.left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                 robot.right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                 robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                 robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                 robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.left1.setTargetPosition(-500);
                 robot.right1.setTargetPosition(-500);
                 robot.left1.setPower(-1);
                 robot.right1.setPower(-1);
                 while(robot.left1.isBusy() && opModeIsActive()){
                 }
                 robot.left1.setPower(0);
                 robot.right1.setPower(0);

                 robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                 robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                 telemetry.addData("heading", robot.angles.firstAngle);
                 telemetry.update();
                 while(robot.angles.firstAngle < 20){
                     robot.left1.setPower(-0.8);
                     robot.right1.setPower(0.8);
                     telemetry.addData("heading", robot.angles.firstAngle);
                     telemetry.update();
                 }
                 robot.left1.setPower(0.18);
                 robot.right1.setPower(-0.18);
                 telemetry.addData("heading", robot.angles.firstAngle);
                 telemetry.update();
                 while(robot.angles.firstAngle < 25 && opModeIsActive() || robot.angles.firstAngle > 45 && opModeIsActive()){
                     telemetry.addData("heading", robot.angles.firstAngle);
                     telemetry.update();
                 }
                 robot.left1.setPower(0);
                 robot.right1.setPower(0);
            }

//            //Shoots blocks
//            if (gamepad1.b){
//                robot.launcher.setPower(-1);
//                stopMotor = new Timer();
//                stopMotor.schedule(new PennyslvaniaTeleop.RemindTask(),1,5000);
//            }

            //GAMEPAD 2
            //Sets Servo Position to Top or Bottom
//            if (gamepad2.dpad_up) {
//                robot.drop.setPower(0.7);
//                if(robot.intake.isBusy()){
//                    robot.intake.setPower(0);
//                }
//            }
//            else if (gamepad2.dpad_down) {
//                robot.drop.setPower(-0.65);
//            }else{
//                robot.drop.setPower(0);
//            }
//            if (!robot.bottomDrop.getState() && !gamepad2.dpad_up) {
//                robot.drop.setPower(0);
//            }
//            if (!robot.topDrop.getState() && !gamepad2.dpad_down) {
//                robot.drop.setPower(0);
//            }

            //Move intake in or out
            if (gamepad2.b) {
                robot.intake.setPower(-0.9);
            } else if (gamepad2.a) {
                robot.intake.setPower(0.9);
            } else if (gamepad2.y) {
                robot.intake.setPower(0.0);
            }

//                if(lidar.getDistance(DistanceUnit.CM) > 35 && lidar.getDistance(DistanceUnit.CM) < 75 && gamepad2.left_stick_y > 0.1){
//                    if(intakeReverse == true){
//                        TimerTask SpinOut = new TimerTask() {
//                            public void run() {
//                                robot.intake.setPower(0);
//                                cancel();
//                            }
//                        };
//
//                        intakeReverse = false;
//                        Timer intake = new Timer("IntakeReverse");
//                        robot.intake.setPower(-0.8);
//                        intake.schedule(SpinOut, 500);
//                    }
//                }
//                else if(lidar.getDistance(DistanceUnit.CM) < 35 || lidar.getDistance(DistanceUnit.CM) > 75){
//                intakeReverse = true;
//                }


            if ((gamepad2.right_bumper && robot.sorterLidar.getDistance(DistanceUnit.CM) < 6) || (gamepad2.right_bumper && robot.sorterLidar.getDistance(DistanceUnit.CM) > 700) || (gamepad2.left_bumper == true && gamepad2.right_bumper == true)) {
                robot.door.setPosition(robot.DOOR_DOWN);

            }
            else if(robot.sorterLidar.getDistance(DistanceUnit.CM) < 4.5 && lidar.getDistance(DistanceUnit.CM) < 8 && gamepad2.right_bumper == false){
                if(DoorFlag == true) {
                    TimerTask DoorRelease = new TimerTask() {
                        @Override
                        public void run() {
                            robot.door.setPosition(robot.DOOR_DOWN);
                            sleep(650);
                            robot.door.setPosition(robot.DOOR_UP);
                            DoorFlag = false;
                            cancel();
                        }
                    };
                    Timer Door_Drop = new Timer("Door");
                    Door_Drop.schedule(DoorRelease, 0);

                }
                else{
                    robot.door.setPosition(robot.DOOR_UP);
                }

            }
            else{
                robot.door.setPosition(robot.DOOR_UP);
            }


            if( DoorFlag == false && lidar.getDistance(DistanceUnit.CM) > 8) {
                DoorFlag = true;
            }






            //Moves intake arm in and out
//            robot.bop.setPower(gamepad2.right_stick_y / 1.25);

//            if (gamepad2.x) {
//                robot.bop.setPower(.9);
//                buttonFlag = false;
//            }

//            if (robot.bopLimit.red() >= 150 && robot.bopLimit.alpha() < 400) {
//                //if (robot.bopLimit.red() >= 150 && robot.bopLimit.alpha() < 400) {
//                if (!robot.topDrop.getState()/*robot.drop.getCurrentPosition() > -20 && robot.drop.getCurrentPosition() < 10*/) {
//                    if (robot.rotateMech.getCurrentPosition() < 0) {
//                        robot.rotateMech.setTargetPosition(0);
//                        robot.rotateMech.setPower(0.1);
//                    }
//
//                    if (robot.rotateMech.getCurrentPosition() > 0) {
//                        robot.rotateMech.setPower(-0.1);
//                    }
//                }
//                //}
//            }


            if(gamepad2.x){
                robot.rotateMech.setTargetPosition(0);
                rotateFlag = true;

//                if(robot.rotateMech.getCurrentPosition() < -5 && rotateFlag == true){
//                    robot.rotateMech.setPower(0.5);
//                    if (gamepad2.right_trigger >= 0.5) {
//                        rotateFlag = false;
//                    } else if (gamepad2.left_trigger >= 0.5) {
//                        rotateFlag = false;
//                    }
//                } else if (robot.rotateMech.getCurrentPosition() > 5 && rotateFlag == true){
//                    robot.rotateMech.setPower(-0.5);
//                    if (gamepad2.right_trigger >= 0.5) {
//                        rotateFlag = false;
//                    } else if (gamepad2.left_trigger >= 0.5) {
//                        rotateFlag = false;
//                    }
//                }

            }
//
//            if (lidar.getDistance(DistanceUnit.CM) < 8 && robot.sorterLidar.getDistance(DistanceUnit.CM) < 4) {
//                robot.door.setPosition(robot.DOOR_UP);
//            }
//            else if(gamepad1.right_bumper) {
//                robot.door.setPosition(robot.DOOR_UP);
//            }
//            else{
//                robot.door.setPosition(robot.DOOR_DOWN);
//            }
//




            if(lidar.getDistance(DistanceUnit.CM) <= 70){
                robot.rotateMech.setTargetPosition(0);
                if (robot.rotateMech.getCurrentPosition() < 5 && robot.rotateMech.getCurrentPosition() > -5){
                    robot.rotateMech.setPower(0);
                    robot.rotateMech.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rotateFlag = false;
                }
                else if(robot.rotateMech.getCurrentPosition() < -30){
                    robot.rotateMech.setPower(0.6);
                    if (gamepad2.right_trigger >= 0.5) {
                        rotateFlag = false;
                    } else if (gamepad2.left_trigger >= 0.5) {
                        rotateFlag = false;
                    }
                } else if (robot.rotateMech.getCurrentPosition() > 30){
                    robot.rotateMech.setPower(-0.6);
                    if (gamepad2.right_trigger >= 0.5) {
                        rotateFlag = false;
                    } else if (gamepad2.left_trigger >= 0.5) {
                        rotateFlag = false;
                    }
                }
            }
            if(gamepad2.dpad_up){
                robot.rotateMech.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rotateMech.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

//            if (robot.bopLimit.red() >= 110 && robot.bopLimit.alpha() < 300) {
//                if (gamepad2.right_stick_y <= -.1) {
//                    robot.leftBop.setPower(gamepad2.right_stick_y * 0.95);
//                    robot.rightBop.setPower(gamepad2.right_stick_y * 0.95);
//                }
//            }

//            if(gamepad2.left_stick_y > 0.1) {
//                if (lidar.getDistance(DistanceUnit.CM) > 40) {
//                    robot.leftBop.setPower(gamepad2.left_stick_y );
//                    robot.rightBop.setPower(gamepad2.left_stick_y );
//                    telemetry.addData("intake lidar", lidar.getDistance(DistanceUnit.CM));
//                    telemetry.update();
//
//                } else if (lidar.getDistance(DistanceUnit.CM) <= 40 && lidar.getDistance(DistanceUnit.CM) > 20) {
//
//                    robot.leftBop.setPower(gamepad2.left_stick_y * 0.7);
//                    robot.rightBop.setPower(gamepad2.left_stick_y * 0.7);
//
//                } else if (lidar.getDistance(DistanceUnit.CM) <= 20 && lidar.getDistance(DistanceUnit.CM) > 7.5){
//
//                            robot.leftBop.setPower(gamepad2.left_stick_y * 0.4);
//                            robot.rightBop.setPower(gamepad2.left_stick_y * 0.4);
//
//                    }
//                 else if (lidar.getDistance(DistanceUnit.CM) < 7.5) {
//                    robot.leftBop.setPower(0);
//                    robot.rightBop.setPower(0);
//                }
//            }.5
            if(gamepad2.left_stick_y > 0.1 && lidar.getDistance(DistanceUnit.CM) < 70 && lidar.getDistance(DistanceUnit.CM) > 8){
                controlSpeed = (lidar.getDistance(DistanceUnit.CM) * 0.01);
                robot.leftBop.setPower(gamepad2.left_stick_y * (controlSpeed*6));
                robot.rightBop.setPower(gamepad2.left_stick_y * (controlSpeed *6));
            }
            else if( gamepad2.left_stick_y > 0.1 && lidar.getDistance(DistanceUnit.CM) < 8){
                robot.leftBop.setPower(0);
                robot.rightBop.setPower(0);
            }
            else {

                robot.leftBop.setPower(gamepad2.left_stick_y * 0.88);
                robot.rightBop.setPower(gamepad2.left_stick_y * 0.88);
            }


//                if(gamepad2.left_stick_y > 0.5){
//                        if (gamepad2.a) {
//                            robot.intake.setPower(0.9);
//                        } else if (gamepad2.b) {
//                            robot.intake.setPower(-0.9);
//                        } else{
//                            robot.intake.setPower(0.0);
//                        }
//
//
//                    }
//                if(robot.topDrop.getState()) {
////                    //robot.drop.setTargetPosition(robot.TOP_INTAKE); // Top
////                    robot.drop.setPower(0.7);
////                }
//                else {
//                    robot.drop.setPower(0);
//                }
//                robot.leftBop.setPower(gamepad2.right_stick_y * 0.95);
//                robot.rightBop.setPower(gamepad2.right_stick_y * 0.95);

            //Rotates the Intake Arm
            if (gamepad2.left_trigger >= 0.5) {
                robot.rotateMech.setTargetPosition(-10);
                robot.rotateMech.setPower(-gamepad2.left_trigger * 0.9);
                rotateFlag = false;
            } else if (gamepad2.right_trigger >= 0.5) {
                robot.rotateMech.setTargetPosition(770);
                robot.rotateMech.setPower(gamepad2.right_trigger * 0.9);
                rotateFlag = false;
            } else {
                robot.rotateMech.setTargetPosition(0);
                robot.rotateMech.setPower(0);
            }

//            if(robot.drop.getPosition() != robot.BOTTOM_INTAKE && robot.drop.getPosition() != robot.TOP_INTAKE){
//                robot.intake.setPower(0);
//            }

            // bulkData = expansionHub.getBulkInputData();

            //Telemetry Section
//            telemetry.addData("Distance (cm)", //Checks what the distance sensor on the launcher sees
//            String.format(Locale.US, "%.02f", robot.senseOBJ.getDistance(DistanceUnit.CM)));

//            telemetry.addData("Left Power", leftPower);
//            telemetry.addData("Right Power", rightPower);
//            telemetry.addData("Gamepad Tigger", gamepad1.right_trigger);
//            telemetry.addData("upper red", robot.upperLimit.red());
//            telemetry.addData("upper blue", robot.upperLimit.blue());
//            telemetry.addData("upper Alpha", robot.upperLimit.alpha());
//            telemetry.addData("bottom red", robot.bottomLimit.red());
//            telemetry.addData("bottom blue", robot.bottomLimit.blue());

            //telemetry.addData("range", String.format("%.01f cm", lidar.getDistance(DistanceUnit.CM)));
            //telemetry.addData("sorter range", String.format("%.01f cm", robot.sorterLidar.getDistance(DistanceUnit.CM)));
            //            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
//            telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
//            telemetry.addData("Arm power", robot.bop.getPower());
//            telemetry.addData("Arm position", robot.bop.getCurrentPosition());
            //telemetry.addData("Color Sensor RED", robot.cornerSensor.red());
            //telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
//            telemetry.addData("right power", robot.right1.getPower());
//            telemetry.addData("right position", robot.right1.getCurrentPosition());
//            telemetry.addData("left power", robot.left1.getPower());
////            telemetry.addData("left position", robot.left1.getCurrentPosition());
//            telemetry.addData("turret position", robot.rotateMech.getCurrentPosition());
//            telemetry.addData("Turret Power", robot.rotateMech.getPower());
//                //telemetry.addData("heading", robot.angles.firstAngle);
////            telemetry.addData("Total current", expansionHub.getTotalModuleCurrentDraw());
////            telemetry.addData("I2C current", expansionHub.getI2cBusCurrentDraw());
////            telemetry.addData("GPIO current", expansionHub.getGpioBusCurrentDraw());
////            telemetry.addData("Left current", left.getCurrentDraw());
////            telemetry.addData("Right current", right.getCurrentDraw());
////            telemetry.addData("drop position", robot.drop.getCurrentPosition());
////            telemetry.addData("Drop Commanded Position", robot.drop.getTargetPosition());
////            telemetry.addData("Drop Power", robot.drop.getPower());
            //      telemetry.addData("Gamepad Right Y", gamepad2.left_stick_y);
////            telemetry.addData("Button Flag", buttonFlag);
////            telemetry.addData("Bop Alpha", robot.bopLimit.alpha());
////            telemetry.addData("Sorter red", robot.sorterLimit.red());
////            telemetry.addData("Bop red", robot.bopLimit.red());
////            telemetry.addData("topDrop", robot.topDrop.getState());
////            telemetry.addData("bottomDrop", robot.bottomDrop.getState());
////            telemetry.addData("dpad up", gamepad1.dpad_up);
////            telemetry.addData("dpad down", gamepad1.dpad_down);
////            telemetry.addData("frontDetect Distance in Inches:", robot.frontDetect.getDistance(DistanceUnit.INCH));
////            telemetry.addData("backDetect Distance in Inches:", robot.backDetect.getDistance(DistanceUnit.INCH));
//            //telemetry.addData("position", robot.sorterFlip.getPosition());
//            telemetry.addData("joystick", gamepad2.left_stick_y);
//           telemetry.addData("intake lidar", lidar.getDistance(DistanceUnit.CM));
//          telemetry.addData("sorter", robot.sorterLidar.getDistance(DistanceUnit.CM));
//           telemetry.addData("rotateMech", robot.rotateMech.getCurrentPosition());
            telemetry.addData("hang encoder", robot.hang.getCurrentPosition());

            //telemetry.addData("wallDetect Distance in CM", robot.wallDetect.getDistance(DistanceUnit.CM));
//            telemetry.addData("rotateFlag State", rotateFlag);
            telemetry.update();
        }
    }

//    class positionTimer extends TimerTask{
//        public void run(){
//         currentTarget += 5;
//         if (currentTarget > setTarget){
//             positionSet.cancel();
//         }
//        }
//
//    }

//    class RemindTask extends TimerTask {
//        public void run(){
//            robot.drop.setPower(0.05);
//            stopMotor.cancel();
//        }
//    }
//        class SpinOut extends TimerTask {
//        public void run() {
//            robot.intake.setPower(-0.8);
//            sleep(400);
//            robot.intake.setPower(0);
//            sorterOut.cancel();
//        }
//    }
//        class BopOut extends TimerTask {
//            public void run() {
//                robot.leftBop.setPower(-0.4);
//                robot.rightBop.setPower(-0.4);
//                intakeOut.cancel();
//            }
//        }
//        class CurrentReg extends TimerTask {
//            public void run() {
//                currentDiv = currentDiv + 7;
//                currentReg.cancel();
//            }
//        }
//
//        class CurrentLim extends TimerTask {
//            public void run() {
//                limitFlag = true;
//                currentLim.cancel();
//            }
//        }
void composeTelemetry () {


    telemetry.addAction(new Runnable() {
        @Override
        public void run() {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robot.gravity = robot.imu.getGravity();
        }
    });
//    telemetry.addLine()
//            .addData("status", new Func<String>() {
//                @Override
//                public String value() {
//                    return robot.imu.getSystemStatus().toShortString();
//                }
//            })
//            .addData("calib", new Func<String>() {
//                @Override
//                public String value() {
//                    return robot.imu.getCalibrationStatus().toString();
//                }
//            });
    telemetry.addLine()
            .addData("heading", new Func<String>() {
                @Override
                public String value() {
                    return robot.formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
                }
            });
//            .addData("roll", new Func<String>() {
//                @Override
//                public String value() {
//                    return robot.formatAngle(robot.angles.angleUnit, robot.angles.secondAngle);
//                }
//            })
//            .addData("pitch", new Func<String>() {
//                @Override
//                public String value() {
//                    return robot.formatAngle(robot.angles.angleUnit, robot.angles.thirdAngle);
//                }
//            });

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


//                telemetry.addData("heading", robot.angles.firstAngle);
//                telemetry.update();
//                offsetHeading = robot.angles.firstAngle;
//                targetHeading = -180 + Math.abs(offsetHeading);
//
//                robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                telemetry.addData("heading", robot.angles.firstAngle);
//                telemetry.update();
//                boolean exit = false;
//                while (robot.angles.firstAngle > (targetHeading + 3) && opModeIsActive() && exit == false){
//                    if(Math.abs(gamepad1.right_stick_y) > 0.5 || Math.abs(gamepad1.right_stick_x) > 0.5){
//                        exit = true;
//                    }
//                    if(robot.angles.firstAngle < 0){
//                        angleTurn = offsetHeading - robot.angles.firstAngle;
//                    } else if(robot.angles.firstAngle > 0){
//                        angleTurn = offsetHeading + robot.angles.firstAngle;
//                    }
//                    //This is a right turn to 78 degrees
//                    double leftPower = (((targetHeading - angleTurn) / 60) * 0.3);
//                    double rightPower = (((targetHeading - angleTurn) / 60) * -0.3);
//                    if(leftPower < 0.15){
//                        leftPower = 0.15;
//                    } if(rightPower > -0.15){
//                        rightPower = -0.15;
//                    }
//                    robot.left1.setPower(leftPower);
//                    robot.right1.setPower(rightPower);
//                    telemetry.addData("left1 power", robot.left1.getPower());
//                    telemetry.addData("right1 power", robot.right1.getPower());
//                    telemetry.addData("heading", robot.angles.firstAngle);
//                    telemetry.addData("angle var:", angleTurn);
//                    telemetry.update();
//                }