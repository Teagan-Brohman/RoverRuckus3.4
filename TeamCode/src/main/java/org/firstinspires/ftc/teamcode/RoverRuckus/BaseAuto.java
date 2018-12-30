package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class BaseAuto extends LinearOpMode {
    RoverHardware robot = new RoverHardware();

    public void preventCollision(int limitingInches, String input){
        if(input == "frontDetect") {
            while (robot.frontDetect.getDistance(DistanceUnit.INCH) < limitingInches && opModeIsActive()) {
                robot.left1.setPower(0);
                robot.right1.setPower(0);
            }
            robot.right1.setPower(robot.savedMotorPowerRight);
            robot.left1.setPower(robot.savedMotorPowerLeft);
        } else if(input == "backDetect"){
            while (robot.backDetect.getDistance(DistanceUnit.INCH) < limitingInches && opModeIsActive()){
                robot.left1.setPower(0);
                robot.right1.setPower(0);
            }
            robot.right1.setPower(robot.savedMotorPowerRight);
            robot.left1.setPower(robot.savedMotorPowerLeft);
        }
    }
    public void searchingForCollision(int limitingInches, String input){
        if(input == "frontDetect" && robot.collisionFlag == false){
            if(robot.frontDetect.getDistance(DistanceUnit.INCH) < limitingInches){
                robot.savedMotorPowerRight = robot.right1.getPower();
                robot.savedMotorPowerLeft = robot.left1.getPower();
                robot.collisionFlag = true;
                preventCollision(limitingInches, input);
            }
        }
        else if(input == "backDetector" && robot.collisionFlag == false){
            if(robot.backDetect.getDistance(DistanceUnit.INCH) < limitingInches){
                robot.savedMotorPowerRight = robot.right1.getPower();
                robot.savedMotorPowerLeft = robot.left1.getPower();
                robot.collisionFlag = true;
                preventCollision(limitingInches, input);
            }
        }
    }

    //Sets all drive motor power.
    public void setMotorPower(double power) {
        robot.left1.setPower(power);
        robot.right1.setPower(power);
    }

    //Sets all motors target position.
    public void setAllTargetPositions(int distance) {
        robot.left1.setTargetPosition(distance);
        robot.right1.setTargetPosition(distance);
    }

    public void driveForwardSetDistance(double power, int distance) {
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        robot.setRunMode("STOP_AND_RESET_ENCODER");
        //Sets target distance. Set to negative distance because motor was running backwards.
        setAllTargetPositions(distance);
        //Sets to RUN_TO_POSITION mode
        robot.setRunMode("RUN_TO_POSITION");
        //Sets power for DC Motors.
        setMotorPower(power);
        //Waits while driving to position.
        while (robot.left1.isBusy() & robot.right1.isBusy()) {
            // Spinning.
            // Waiting for robot to arrive at destination.
        }
    }
}
