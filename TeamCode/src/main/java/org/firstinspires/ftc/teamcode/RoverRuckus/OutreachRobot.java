package org.firstinspires.ftc.teamcode.RoverRuckus;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Outreach bot", group = "Teleop")
@Disabled
public class OutreachRobot extends LinearOpMode {

    DcMotor Shooter;
    DcMotor Driveleft;
    DcMotor Driveright;
    Servo BallStop;
    public void runOpMode(){
        Driveright = hardwareMap.dcMotor.get("Driveright");
        Driveleft = hardwareMap.dcMotor.get("Driverleft");
        Shooter = hardwareMap.dcMotor.get("Shooter");
        BallStop = hardwareMap.servo.get("BallStop");

        Driveright.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            Driveright.setPower(gamepad1.right_stick_y/2);
            Driveleft.setPower(gamepad1.left_stick_y/2);

            if (gamepad1.a){
                Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Shooter.setTargetPosition(-5250);
                Shooter.setPower(-.8);
                while (Shooter.isBusy()){}
                Shooter.setPower(0);
                BallStop.setPosition(.45);
                sleep(500);
                BallStop.setPosition(1);
            }
//            if (gamepad1.b){
//                Shooter.setPower(0);
//            }
//            if (gamepad1.x) {
//                BallStop.setPosition(.5);
//            }
//            if (gamepad1.y) {
//                BallStop.setPosition(1);
//            }

            telemetry.addData("Servo Position", BallStop.getPosition());
            telemetry.update();
            idle();
        }

    }
}
