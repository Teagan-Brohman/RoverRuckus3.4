package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@TeleOp(name = "Teleop", group = "Teleop")
public class SecondRobotTeleop extends LinearOpMode {
    public RoverHardware robot = new RoverHardware();

    public float leftPower;
    public float rightPower;
    public float xValue;
    public float yValue;


    ExpansionHubEx expansionHub;
    Timer stopMotor;
    Timer sorterOut;
    Timer intakeOut;
    Timer currentReg;
    Timer currentLim;
    ExpansionHubMotor left, right;
    RevBulkData bulkData;
    boolean buttonFlag;
    Boolean limitFlag = false;
    public int currentDiv = 1;


    public void runOpMode() {
        robot.init(hardwareMap);

        RevExtensions2.init();


        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        left = (ExpansionHubMotor) hardwareMap.dcMotor.get("left1");
        right = (ExpansionHubMotor) hardwareMap.dcMotor.get("right1");

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
        telemetry.update();
        robot.initServoPositions();
        //robot.drop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();


        while (opModeIsActive()) {
            if (limitFlag = false) {
                currentLim = new Timer();
                currentLim.schedule(new SecondRobotTeleop.CurrentLim(), 20000, 10);
            }


            robot.rotateMech.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //GAMEPAD 1
            //Arcade Style Drive Motors
            yValue = gamepad1.left_stick_y;
            xValue = gamepad1.left_stick_x;

            leftPower = yValue - xValue;
            rightPower = yValue + xValue;

            robot.left1.setPower(Range.clip(leftPower, -1.0, 1.0));
            robot.right1.setPower(Range.clip(rightPower, -1.0, 1.0));

            //Tank Drive (if driver prefers it
//            robot.left1.setPower(gamepad1.left_stick_y);
//            robot.right1.setPower(gamepad1.right_stick_y);

            //Arm that shoots blocks and balls

            if (Math.abs(gamepad2.right_stick_y) >= 0.2 /*|| gamepad1.right_stick_y <= -0.5*/) {
                buttonFlag = true;
            }


            if (gamepad1.right_bumper) {
                robot.sorter.setPower(0.9);
            } else if (gamepad1.left_bumper) {
                robot.sorter.setPower(-1.0);
                robot.sorterFlip.setPosition(robot.SORTER_UP);
                // if(robot.bopLimit.red() >= 200){
//                    sorterOut = new Timer();
//                    sorterOut.schedule(new MoveOut(), 0, 1000);
                //}
            } else {
                robot.sorter.setPower(0);
            }

            if (gamepad1.a && robot.sorterLimit.red() < 100 && robot.sorterLimit.alpha() < 300) {
                robot.sorterFlip.setPosition(robot.SORTER_DOWN);
            } else if (gamepad1.b) {
                robot.sorterFlip.setPosition(robot.SORTER_UP);
            }

            //Hanging Mechanism
            if (gamepad1.dpad_up && robot.upperLimit.red() < 300) {
                robot.hang.setPower(-1);
            } else if (gamepad1.dpad_down && robot.bottomLimit.red() < 100) {
                robot.hang.setPower(1);
            } else {
                robot.hang.setPower(0);
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
                robot.intake.setPower(0.9);
            } else if (gamepad2.a) {
                robot.intake.setPower(-0.9);
            } else if (gamepad2.y) {
                robot.intake.setPower(0.0);
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
                robot.leftBop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightBop.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (robot.bopLimit.red() >= 110 && robot.bopLimit.alpha() < 300) {
                if (gamepad2.right_stick_y <= -.1) {
                    robot.leftBop.setPower(gamepad2.right_stick_y * 0.95);
                    robot.rightBop.setPower(gamepad2.right_stick_y * 0.95);
                } else {
                    robot.leftBop.setPower(0);
                    robot.rightBop.setPower(0);
                }
//                if(robot.topDrop.getState()) {
////                    //robot.drop.setTargetPosition(robot.TOP_INTAKE); // Top
////                    robot.drop.setPower(0.7);
////                }
//                else {
//                    robot.drop.setPower(0);
//                }
            } else if (buttonFlag == true) {
                robot.leftBop.setPower(gamepad2.right_stick_y * 0.95);
                robot.rightBop.setPower(gamepad2.right_stick_y * 0.95);
            }

            //Rotates the Intake Arm
            if (gamepad2.right_trigger >= 0.5) {
                robot.rotateMech.setPower(-gamepad2.right_trigger * 0.5);
            } else if (gamepad2.left_trigger >= 0.5) {
                robot.rotateMech.setPower(gamepad2.left_trigger * 0.5);
            } else {
                robot.rotateMech.setPower(0);
            }

//            if(robot.drop.getPosition() != robot.BOTTOM_INTAKE && robot.drop.getPosition() != robot.TOP_INTAKE){
//                robot.intake.setPower(0);
//            }

            bulkData = expansionHub.getBulkInputData();

            //Telemetry Section
//            telemetry.addData("Distance (cm)", //Checks what the distance sensor on the launcher sees
//            String.format(Locale.US, "%.02f", robot.senseOBJ.getDistance(DistanceUnit.CM)));

//            telemetry.addData("Left Power", leftPower);
//            telemetry.addData("Right Power", rightPower);
//            telemetry.addData("Gamepad Tigger", gamepad1.right_trigger);
            telemetry.addData("upper red", robot.upperLimit.red());
            telemetry.addData("upper blue", robot.upperLimit.blue());
            telemetry.addData("bottom red", robot.bottomLimit.red());
            telemetry.addData("bottom blue", robot.bottomLimit.blue());
//            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
//            telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
//            telemetry.addData("Arm power", robot.bop.getPower());
//            telemetry.addData("Arm position", robot.bop.getCurrentPosition());
            //telemetry.addData("Color Sensor RED", robot.cornerSensor.red());
            //telemetry.addData("Color Sensor BLUE", robot.cornerSensor.blue());
//            telemetry.addData("right power", robot.right1.getPower());
//            telemetry.addData("right position", robot.right1.getCurrentPosition());
//            telemetry.addData("left power", robot.left1.getPower());
//            telemetry.addData("left position", robot.left1.getCurrentPosition());
//            telemetry.addData("turret position", robot.rotateMech.getCurrentPosition());
//            telemetry.addData("Turret Power", robot.rotateMech.getPower());
            //telemetry.addData("heading", robot.angles.firstAngle);
//            telemetry.addData("Total current", expansionHub.getTotalModuleCurrentDraw());
//            telemetry.addData("I2C current", expansionHub.getI2cBusCurrentDraw());
//            telemetry.addData("GPIO current", expansionHub.getGpioBusCurrentDraw());
//            telemetry.addData("Left current", left.getCurrentDraw());
//            telemetry.addData("Right current", right.getCurrentDraw());
//            telemetry.addData("drop position", robot.drop.getCurrentPosition());
//            telemetry.addData("Drop Commanded Position", robot.drop.getTargetPosition());
//            telemetry.addData("Drop Power", robot.drop.getPower());
////            telemetry.addData("Gamepad Right Y", gamepad1.right_stick_y);
//            telemetry.addData("Button Flag", buttonFlag);
//            telemetry.addData("Bop Alpha", robot.bopLimit.alpha());
//            telemetry.addData("Sorter red", robot.sorterLimit.red());
//            telemetry.addData("Bop red", robot.bopLimit.red());
//            telemetry.addData("topDrop", robot.topDrop.getState());
//            telemetry.addData("bottomDrop", robot.bottomDrop.getState());
            telemetry.addData("dpad up", gamepad1.dpad_up);
            telemetry.addData("dpad down", gamepad1.dpad_down);
//            telemetry.addData("frontDetect Distance in Inches:", robot.frontDetect.getDistance(DistanceUnit.INCH));
//            telemetry.addData("backDetect Distance in Inches:", robot.backDetect.getDistance(DistanceUnit.INCH));
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
    class MoveOut extends TimerTask{
        public void run(){
            robot.sorter.setPower(0.8);
            sorterOut.cancel();
        }
    }
    class BopOut extends TimerTask{
        public void  run(){
            robot.leftBop.setPower(-0.4);
            robot.rightBop.setPower(-0.4);
            intakeOut.cancel();
        }
    }
    class CurrentReg extends TimerTask{
        public void run(){
            currentDiv = currentDiv + 7;
            currentReg.cancel();
        }
    }

    class CurrentLim extends TimerTask{
        public void run(){
            limitFlag = true;
            currentLim.cancel();
        }
    }
    void composeTelemetry() {


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
