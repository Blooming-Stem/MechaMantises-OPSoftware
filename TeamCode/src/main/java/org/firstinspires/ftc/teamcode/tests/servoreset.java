package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@TeleOp
public class servoreset extends OpMode {


    Servo servo1;
    Servo servo2;




    public void init() {
        servo1 = hardwareMap.servo.get("wrist");


        servo1.setPosition(0);

        servo2 = hardwareMap.servo.get("claw");
        servo2.setPosition(1);






    }


    public void loop() {
        if(gamepad1.a){
            servo1.setPosition(1);

        }
        if(gamepad1.b){
            servo2.setPosition(0);
        }




    }
}
