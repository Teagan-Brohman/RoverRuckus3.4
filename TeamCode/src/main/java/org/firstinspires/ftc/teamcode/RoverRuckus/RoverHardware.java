package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.TimerTask;
import java.util.Timer;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

import java.util.Locale;

public class RoverHardware {

    //Create hardware in code
    HardwareMap HwMap;

    //Drive Motors
    public DcMotor left1;
    // DcMotor left2;
    public DcMotor right1;
   //DcMotor right2;
//    public DcMotor launcher;
    //Hanging Mechanism
    public DcMotor hang;
    //bopper
    public DcMotor bop;
    //Rotation Mechanism
    public DcMotor rotateMech;
    // Ball Catcher???
    public DcMotor sorter;
    //dropper
    public DcMotor drop;
    //marker dropper
    public Servo marker;
    //Sorter Flipper
    public Servo sorterFlip;
    //Intake Servo
    public DcMotor intake;


    //Create Sensors
    //DistanceSensor senseOBJ;
    public DistanceSensor sensorRange;

    //Color Sensor 'Limit Switches'
    public ColorSensor bottomLimit;
    public ColorSensor upperLimit;
    public ColorSensor cornerSensor;
    public ColorSensor bopLimit;
    public ColorSensor sorterLimit;

    //Create Gyro
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //Create Limit Switches
    DigitalChannel topDrop;
    DigitalChannel bottomDrop;

    RevBulkData bulkData;
    AnalogInput a0, a1, a2, a3;
    DigitalChannel d0, d1, d2, d3, d4, d5, d6, d7;

    public final int BOTTOM_INTAKE = -195;
    public final int MIDDLE_INTAKE = -96;
    public final int TOP_INTAKE = 0;
    public final double DILBERT_DOWN = 0.7;
    public final double DILBERT_UP = 0.0;
    public final double SORTER_DOWN = 0.2;
    public final double SORTER_UP = 0.45;

    public RoverHardware() {
        System.out.println("Created new RRHardwarePresets Object!");
    }


    public void init(HardwareMap hwm) {

        //Give mappings for each hardware device do the phone knows what goes to which port
        HwMap = hwm;
        left1 = HwMap.dcMotor.get("left1");
        right1 = HwMap.dcMotor.get("right1");
        //Hanging Motor
        hang = HwMap.dcMotor.get("hang");
        //bopper
        bop = HwMap.dcMotor.get("bop");
        //Rotation Mechanism
        rotateMech = HwMap.dcMotor.get("rotate");
        //Ball Catcher
        sorter = HwMap.dcMotor.get("sorter");
        //dropper
        drop = HwMap.dcMotor.get("drop");
        //marker dropper
        marker = HwMap.servo.get("marker");
        //sorterFlips
        sorterFlip = HwMap.servo.get("sorterFlip");
        //Intake servo
        intake = HwMap.dcMotor.get("intake");

        //ColorSensors
        bottomLimit = HwMap.colorSensor.get("bottomLimit");
        upperLimit = HwMap.colorSensor.get("upperLimit");
        cornerSensor = HwMap.colorSensor.get("cornerSensor");
        bopLimit = HwMap.colorSensor.get("bopLimit");
        sorterLimit = HwMap.colorSensor.get("sorterLimit");

        //Gyro
        imu = HwMap.get(BNO055IMU.class, "imu");

        //Limit Switches
        topDrop = HwMap.get(DigitalChannel.class, "topDrop");
        bottomDrop = HwMap.get(DigitalChannel.class, "bottomDrop");

        //Set DcMotor Directions and Behaviors
        left1.setDirection(DcMotorSimple.Direction.FORWARD);
        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        hang.setDirection(DcMotorSimple.Direction.FORWARD);
        bop.setDirection(DcMotorSimple.Direction.FORWARD);
        rotateMech.setDirection(DcMotorSimple.Direction.FORWARD);
        drop.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotateMech.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

//
    public void initServoPositions() {
        sorterFlip.setPosition(SORTER_UP);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void setRunMode(String input) {
        if (input.equals("STOP_AND_RESET_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (input.equals("RUN_WITHOUT_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (input.equals("RUN_USING_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (input.equals("RUN_TO_POSITION")) {
            this.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    //Returns TRUE if any drive motors are busy and FALSE if not.
    public boolean anyMotorsBusy() {
        if (this.left1.isBusy() || this.right1.isBusy()) {
            return (true);
        } else {
            return (false);
        }
    }

    //Sets all drive motor power.
    public void setMotorPower(double power) {
        this.left1.setPower(power);
        this.right1.setPower(power);
    }

    //Sets all motors target position.
    public void setAllTargetPositions(int distance) {
        left1.setTargetPosition(distance);
        right1.setTargetPosition(distance);
    }

    public void driveForwardSetDistance(double power, int distance) {
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        //Sets target distance. Set to negative distance because motor was running backwards.
        setAllTargetPositions(distance);
        //Sets to RUN_TO_POSITION mode
        setRunMode("RUN_TO_POSITION");
        //Sets power for DC Motors.
        setMotorPower(power);
        //Waits while driving to position.
        while (left1.isBusy() & right1.isBusy()) {
            // Spinning.
            // Waiting for robot to arrive at destination.
        }
    }
}

