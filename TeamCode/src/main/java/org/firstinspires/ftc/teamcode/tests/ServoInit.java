package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ServoInit extends LinearOpMode {
    DcMotor slidesLeft;
    DcMotor slidesRight;
    Servo armright;
    Servo armleft;
    Servo claw;
    Servo wrist;
    ElapsedTime arm_state_timer = new ElapsedTime();
    public enum ArmState{
        ARM_IDLE,
        CLAW_CLOSE,
        WRIST_TURN,
        ARM_UP,
        DEPOSIT_REST,


    }
    ArmState arm_state = ArmState.ARM_IDLE;



    public void runOpMode(){
        slidesLeft = hardwareMap.dcMotor.get("slidesLeft");
        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slidesRight = hardwareMap.dcMotor.get("slidesRight");
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armright = hardwareMap.servo.get("rightArm");
        armleft = hardwareMap.servo.get("leftArm");
        armright.setPosition(0.28);
        armleft.setPosition(0.72);
        claw = hardwareMap.servo.get("claw");
        claw.setPosition(0.9);
        wrist = hardwareMap.servo.get("wrist");
        wrist.setPosition(0.05);



        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("SlidesLeft", slidesLeft.getCurrentPosition());
            telemetry.addData("SlidesRight", slidesRight.getCurrentPosition());

            telemetry.update();
            if(gamepad1.y){
                slidesLeft.setTargetPosition(2000);
                slidesRight.setTargetPosition(2000);
                slidesLeft.setPower(0.5);
                slidesRight.setPower(0.5);
                slidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.a){
                slidesLeft.setTargetPosition(0);
                slidesRight.setTargetPosition(0);
                slidesLeft.setPower(0.5);
                slidesRight.setPower(0.5);
                slidesLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slidesRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.x){





            }
            switch (arm_state){
                case ARM_IDLE:
                    if(gamepad1.b){
                        claw.setPosition(1);
                        arm_state_timer.reset();
                        arm_state = ArmState.CLAW_CLOSE;

                    }
                case CLAW_CLOSE:
                    if(arm_state_timer.seconds()>0.5){
                        wrist.setPosition(0.7);
                        arm_state_timer.reset();
                        arm_state = ArmState.WRIST_TURN;
                    }
                case WRIST_TURN:
                    if(arm_state_timer.seconds()>0.5){

                    }
            }






        }
    }
}
