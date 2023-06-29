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
public class ServoInit extends OpMode {
    SampleMecanumDrive drive;
    private PIDController slidescontroller;
    public static double ps = 0.002, is = 0, ds = 0;
    public static double fs = 0.01;
    public int targets = 0;
    private final double ticks_in_degrees = 384 / 360.0;
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
        LIFT_UP,
        DROP,
        LIFT_RESET,
        LIFT_DOWN,
        DOWN,


    }
    public String pole = "";
    ArmState arm_state = ArmState.ARM_IDLE;



    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        slidescontroller = new PIDController(ps, is, ds);
        slidesLeft = hardwareMap.dcMotor.get("slidesLeft");
        slidesLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slidesRight = hardwareMap.dcMotor.get("slidesRight");
        slidesRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armright = hardwareMap.servo.get("rightArm");
        armleft = hardwareMap.servo.get("leftArm");
        armright.setPosition(0);
        armleft.setPosition(1);
        claw = hardwareMap.servo.get("claw");
        claw.setPosition(0.9);
        wrist = hardwareMap.servo.get("wrist");
        wrist.setPosition(0.025);


    }


        public void loop() {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            telemetry.addData("State", arm_state);

            telemetry.update();


            switch (arm_state) {
                case ARM_IDLE:
                    if (gamepad1.a) {
                        claw.setPosition(1);
                        pole = "low";
                        arm_state_timer.reset();
                        arm_state = ArmState.CLAW_CLOSE;

                    }
                    if(gamepad1.left_bumper){
                        claw.setPosition(1);
                    }
                    else if(gamepad1.b){
                        claw.setPosition(1);
                        pole = "medium";
                        arm_state_timer.reset();
                        arm_state = ArmState.CLAW_CLOSE;
                    }
                    else if(gamepad1.y){
                        claw.setPosition(1);
                        pole = "high";
                        arm_state_timer.reset();
                        arm_state = ArmState.CLAW_CLOSE;
                    }
                    break;
                case CLAW_CLOSE:
                    if (arm_state_timer.seconds() > 0.5) {
                        if(pole == "high"){
                            targets = 1800;
                        }
                        else if(pole == "medium"){
                            targets = 1200;
                        }
                        else if(pole == "low"){
                            targets = 0;
                        }

                        wrist.setPosition(0.7+0.025);
                        arm_state_timer.reset();
                        arm_state = ArmState.WRIST_TURN;
                    }
                    break;
                case WRIST_TURN:
                    if (arm_state_timer.seconds() > 1) {
                        armright.setPosition(0.75);
                        armleft.setPosition(0.25);
                        arm_state_timer.reset();
                        arm_state = ArmState.LIFT_UP;
                    }
                    break;


                case LIFT_UP:
                    if(pole != "low"){
                    if(slidesRight.getCurrentPosition()>1000){
                        armright.setPosition(0.8);
                        armleft.setPosition(0.2);
                        arm_state = ArmState.DROP;
                    }
                    }else {
                        arm_state = ArmState.DROP;
                    }
                    break;
                case DROP:
                    if(targets < slidesRight.getCurrentPosition()+20){
                        if(gamepad1.right_bumper){
                            claw.setPosition(0.9);
                            if(pole != "low") {
                                armright.setPosition(1);
                                armleft.setPosition(0);
                            }
                            arm_state_timer.reset();
                            arm_state = ArmState.LIFT_DOWN;
                        }
                    }
                    break;
                case LIFT_DOWN:
                    if(arm_state_timer.seconds()>1){
                        wrist.setPosition(0.025);
                        armright.setPosition(0);
                        armleft.setPosition(1);
                        targets = 0;
                        arm_state = ArmState.ARM_IDLE;
                    }
                    break;

            }
            slidescontroller.setPID(ps, is, ds);
            int slidesleftpos = slidesLeft.getCurrentPosition();
            double pidleft = slidescontroller.calculate(slidesleftpos, targets);
            double ffleft = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

            int slidesrightpos = slidesRight.getCurrentPosition();
            double pidright = slidescontroller.calculate(slidesrightpos, targets);
            double ffright = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

            double powerleft = pidleft + ffleft;

            double powerright = pidright + ffright;

            slidesLeft.setPower(powerleft);
            slidesRight.setPower(powerright);



    }
}
