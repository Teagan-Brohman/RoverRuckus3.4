package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;
import org.openftc.revextensions2.RevExtensions2;

@TeleOp(name = "Test", group = "Teleop")
public class idk extends LinearOpMode {

    public RoverHardware robot = new RoverHardware();
    public boolean intakeReleaseFlag = false;
    public double controlSpeed = 0;

    DistanceSensor lidar;

    public void runOpMode() {
        robot.init(hardwareMap);
        lidar = hardwareMap.get(DistanceSensor.class, "lidar");

        waitForStart();
        while(opModeIsActive()){

            waitForStart();

            if(gamepad1.x){
                robot.leftBop.setPower(-.6);
                robot.rightBop.setPower(-.6);
            }
            else if(gamepad1.y){
                robot.leftBop.setPower(0);
                robot.rightBop.setPower(0);
            }


            if (lidar.getDistance(DistanceUnit.CM) > 50 && !gamepad1.b) {

                robot.leftBop.setPower(0);
                robot.rightBop.setPower(0);
                robot.leftBop.setTargetPosition(robot.leftBop.getCurrentPosition());
                robot.rightBop.setTargetPosition(robot.rightBop.getCurrentPosition());
            }
            if(gamepad1.b && lidar.getDistance(DistanceUnit.CM) < 70 && lidar.getDistance(DistanceUnit.CM) > 8){
                controlSpeed = (lidar.getDistance(DistanceUnit.CM) * 0.01);
                robot.leftBop.setPower(controlSpeed*4);
                robot.rightBop.setPower((controlSpeed *4));
            }
            else if( gamepad1.b && lidar.getDistance(DistanceUnit.CM) < 8){
                robot.leftBop.setPower(0);
                robot.rightBop.setPower(0);
            }
            else if(gamepad1.b && lidar.getDistance(DistanceUnit.CM) > 70){
            robot.rightBop.setPower(1);
            robot.leftBop.setPower(1);
            }
            else{

            }

        }




    }
}
