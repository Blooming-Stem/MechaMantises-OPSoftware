package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveCancelable;

@TeleOp
public class TeleOPFINAL extends OpMode {
    final int lift_high = 2950;
    final int lift_mid = 2200;
    final int lift_low = 1400;
    final int turret_center = 0;
    final int turret_right = 360;
    final int turret_left = -360;
    final double claw_open = 0.8;
    final double claw_close = 0;

    private DistanceSensor distance;
    private PIDController slidescontroller;
    public static double ps = 0.002, is = 0, ds = 0;
    public static double fs = 0.01;
    public int targets = 0;
    private final double ticks_in_degrees = 384 / 360.0;
    private DcMotorEx slidesleft;
    private DcMotorEx slidesright;
    private Servo turret;
    ElapsedTime liftTimer = new ElapsedTime();
    SampleMecanumDriveCancelable drive;
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor rightRear;
    DcMotor leftRear;
    Servo claw;
    public static int currentposleft;
    public static int currentposright;
    public static int currentposturret;
    public enum LiftState{
        LIFTSTART,
        LIFTCLAWCLOSE,
        LIFTFORWARD,
        LIFTEXTEND,
        LIFTTURN,
        LIFTDROP,
        LIFTARMRESET,
        LIFTRETRACT,
        LIFTBACK,
        LIFTRETRACTED,
    }
    public enum Mode{
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL,
    }
    public enum Pole{
        LOW,
        MEDIUM,
        HIGH,
        FAR_HIGH,
    }
    public enum DriveMode{
        AUTOMATIC,
        MANUAL,
    }
    public enum Side{
        LEFT,
        RIGHT,
    }
    public enum Drop{
        LIFTIDLE,
        LIFTSTART,
        LIFTUP,
        LIFTDROP,
        LIFTDOWN,

    }
    public String direction = "forward";
    public boolean automaticpickup = true;
    Drop drop = Drop.LIFTIDLE;
    LiftState liftstate = LiftState.LIFTSTART;
    Mode mode = Mode.DRIVER_CONTROL;
    Pole pole = Pole.HIGH;
    DriveMode drivemode = DriveMode.MANUAL;
    Side side = Side.LEFT;

    ElapsedTime toggletime = new ElapsedTime();
    ElapsedTime droptime = new ElapsedTime();
    ElapsedTime slacktime = new ElapsedTime();
    @Override
    public void init() {


        liftTimer.reset();
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        leftRear = hardwareMap.dcMotor.get("leftRear");

        slidescontroller = new PIDController(ps, is, ds);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slidesleft = hardwareMap.get(DcMotorEx.class, "slidesLeft");
        slidesright = hardwareMap.get(DcMotorEx.class, "slidesRight");

        slidesleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidesright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesright.setDirection(DcMotorSimple.Direction.REVERSE);
        drive = new SampleMecanumDriveCancelable(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details



        claw = hardwareMap.servo.get("claw");
        turret = hardwareMap.servo.get("turret");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        toggletime.reset();
        droptime.reset();


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        claw.setPosition(0.6);
        turret.setPosition(0.5);
        distance = hardwareMap.get(DistanceSensor.class, "distance");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distance;



    }
    @Override
    public void loop() {
        telemetry.addData("DROPSTATE", drop);
        if(gamepad1.dpad_left||gamepad1.dpad_right){
            if(gamepad1.dpad_left){
                side = Side.LEFT;
            }
            else if(gamepad1.dpad_right){
                side = Side.RIGHT;
            }
        }
        if(gamepad1.start|| gamepad1.back){
            if(gamepad1.start){
                drivemode = DriveMode.AUTOMATIC;
            }
            if(gamepad1.back){
                drivemode = DriveMode.MANUAL;
                liftstate= LiftState.LIFTSTART;
            }
        }
        telemetry.addData("DRIVERMODE:",drivemode);
        telemetry.addData("AUTOMATICPICKUP",automaticpickup);
        telemetry.addData("SIDE", side);
        telemetry.addData("Mode:",mode);
        telemetry.addData("LiftState:",liftstate);
        switch (drivemode){
            case MANUAL:
                switch (drop){
                    case LIFTSTART:
                        targets-=500;
                        drop = Drop.LIFTUP;
                        break;
                    case LIFTUP:
                        if(slidesleft.getCurrentPosition()>targets-30&&slidesright.getCurrentPosition()>targets-30){
                            claw.setPosition(0.6);
                            droptime.reset();
                            drop = Drop.LIFTDROP;
                        }
                        break;
                    case LIFTDROP:
                        if(droptime.seconds()>0.5){
                            targets+=500;
                            drop = Drop.LIFTDOWN;
                        }
                        break;
                    case LIFTDOWN:
                        if(slidesleft.getCurrentPosition()<targets+30&&slidesright.getCurrentPosition()<targets+30) {
                            drop = Drop.LIFTIDLE;
                        }
                        break;



                }

                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
                if(gamepad1.left_bumper&&toggletime.seconds() >=0.5){
                    if(automaticpickup==true){
                        automaticpickup=false;
                        toggletime.reset();

                    }
                    else{
                        automaticpickup=true;
                        toggletime.reset();
                    }
                }

                if(automaticpickup == true) {
                    if (distance.getDistance(DistanceUnit.INCH) < 5) {
                        claw.setPosition(1);
                    }
                }
                if(gamepad1.b){
                    claw.setPosition(1);
                }
                else if(gamepad1.right_bumper){
                    if(slidesleft.getCurrentPosition()>1430&&slidesright.getCurrentPosition()>1430&&drop == Drop.LIFTIDLE){
                        drop = Drop.LIFTSTART;
                    }
                    else {
                        claw.setPosition(0.6);
                    }
                }




                if(gamepad2.y) {

                    targets = 2950;

                }else if(gamepad2.a) {
                    targets = 0;

                }
                else if(gamepad2.dpad_up){
                    targets = 2200;
                }
                else if(gamepad2.dpad_right){
                    targets = 1400;
                }
                if(gamepad2.x){
                    if(slidesleft.getCurrentPosition()>50||slidesright.getCurrentPosition()>50) {
                        turret.setPosition(0.1);
                    }
                }else if(gamepad2.b){
                    if(slidesleft.getCurrentPosition()>50||slidesright.getCurrentPosition()>50) {
                        turret.setPosition(0.9);
                    }

                }
                else if(gamepad2.right_bumper){
                    turret.setPosition(0.5);
                }
                break;
            case AUTOMATIC:
                switch (mode){
                    case DRIVER_CONTROL:
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y,
                                        -gamepad1.left_stick_x,
                                        -gamepad1.right_stick_x
                                )
                        );
                        switch (liftstate) {
                            case LIFTSTART:
                                if (distance.getDistance(DistanceUnit.INCH) < 5) {
                                    claw.setPosition(1);
                                    liftTimer.reset();
                                    liftstate = LiftState.LIFTCLAWCLOSE;

                                }
                                break;

                            case LIFTCLAWCLOSE:
                                if(gamepad2.x){
                                    claw.setPosition(0.6);
                                    //liftstate = LiftState.LIFTSTART;
                                }
                                if(gamepad2.b){
                                    claw.setPosition(1);
                                }
                                if (liftTimer.seconds()>=0.2) {

                                    if(gamepad1.a||gamepad1.b||gamepad1.y|| gamepad1.x){
                                        if(gamepad1.a){
                                            pole = Pole.LOW;
                                            targets = 1400;
                                        }
                                        else if(gamepad1.x){
                                            pole = Pole.MEDIUM;
                                            targets = 2200;
                                        }
                                        else if(gamepad1.b){
                                            pole = Pole.HIGH;
                                            targets = 2950;
                                        }
                                        else if(gamepad1.y){
                                            pole = Pole.FAR_HIGH;
                                            targets = 2950;
                                        }

                                        liftstate = LiftState.LIFTFORWARD;
                                    }


                                }
                                break;
                            case LIFTFORWARD:
                                direction = "forward";
                                switch (pole){
                                    case LOW:
                                        Trajectory trajgo1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                                .back(10)
                                                .build();

                                        drive.followTrajectoryAsync(trajgo1);
                                        mode = Mode.AUTOMATIC_CONTROL;
                                        liftstate = LiftState.LIFTDROP;
                                        break;
                                    case MEDIUM:
                                        Trajectory trajgo2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                                .back(33.5)
                                                .build();

                                        drive.followTrajectoryAsync(trajgo2);
                                        mode = Mode.AUTOMATIC_CONTROL;
                                        liftstate = LiftState.LIFTDROP;
                                        break;
                                    case HIGH:
                                        Trajectory trajgo3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                                .back(33.5)
                                                .build();

                                        drive.followTrajectoryAsync(trajgo3);
                                        mode = Mode.AUTOMATIC_CONTROL;
                                        liftstate = LiftState.LIFTDROP;
                                        break;
                                    case FAR_HIGH:
                                        Trajectory trajgo4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                                .back(58)
                                                .build();

                                        drive.followTrajectoryAsync(trajgo4);
                                        mode = Mode.AUTOMATIC_CONTROL;
                                        liftstate = LiftState.LIFTDROP;

                                        break;
                                }


                                break;


                            case LIFTDROP:
                                if (gamepad1.right_bumper) {
                                    claw.setPosition(0.6);
                                    targets-=500;
                                    //droptime.reset();

                                    liftTimer.reset();
                                    liftstate = LiftState.LIFTARMRESET;
                                }
                                break;

                            case LIFTARMRESET:
                                if (liftTimer.seconds() >= 0.5) {
                                    turret.setPosition(0.5);
                                    liftTimer.reset();
                                    liftstate = LiftState.LIFTRETRACT;
                                }
                                break;
                            case LIFTRETRACT:
                                if(turret.getPosition()==0.5&&liftTimer.seconds()>=0.5){
                                    targets= 0;
                                    liftstate = LiftState.LIFTBACK;
                                }
                                break;
                            case LIFTBACK:
                                direction = "backward";
                                switch (pole){
                                    case LOW:
                                        Trajectory trajgo1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                                .forward(10)
                                                .build();

                                        drive.followTrajectoryAsync(trajgo1);
                                        mode = Mode.AUTOMATIC_CONTROL;
                                        liftstate = LiftState.LIFTRETRACTED;
                                        break;
                                    case MEDIUM:
                                        Trajectory trajgo2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                                .forward(33.5)
                                                .build();

                                        drive.followTrajectoryAsync(trajgo2);
                                        mode = Mode.AUTOMATIC_CONTROL;
                                        liftstate = LiftState.LIFTRETRACTED;
                                        break;
                                    case HIGH:
                                        Trajectory trajgo3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                                .forward(33.5)
                                                .build();

                                        drive.followTrajectoryAsync(trajgo3);
                                        mode = Mode.AUTOMATIC_CONTROL;
                                        liftstate = LiftState.LIFTRETRACTED;
                                        break;
                                    case FAR_HIGH:
                                        Trajectory trajgo4 = drive.trajectoryBuilder(drive.getPoseEstimate())
                                                .forward(58)
                                                .build();

                                        drive.followTrajectoryAsync(trajgo4);
                                        mode = Mode.AUTOMATIC_CONTROL;
                                        liftstate = LiftState.LIFTRETRACTED;

                                        break;
                                }
                                break;

                            case LIFTRETRACTED:
                                if (slidesleft.getCurrentPosition() < 10 && slidesright.getCurrentPosition() < 10 ) {
                                    liftstate = LiftState.LIFTSTART;
                                }
                                break;
                            default:
                                liftstate = LiftState.LIFTSTART;


                        }
                        break;



                    case AUTOMATIC_CONTROL:
                        // If x is pressed, we break out of the automatic following
                        if (gamepad1.left_bumper) {
                            drive.breakFollowing();
                            liftstate = LiftState.LIFTSTART;
                            mode = Mode.DRIVER_CONTROL;
                            drivemode = DriveMode.MANUAL;
                        }
                        if (slidesleft.getCurrentPosition() > 1400 && slidesright.getCurrentPosition() > 1400&& direction=="forward") {
                            switch(side) {

                                case LEFT:
                                    switch (pole) {
                                        case LOW:
                                            turret.setPosition(0.9);
                                            break;
                                        case MEDIUM:
                                            turret.setPosition(0.9);
                                            break;
                                        case HIGH:
                                            turret.setPosition(0.1);
                                            break;
                                        case FAR_HIGH:
                                            turret.setPosition(0.9);
                                            break;
                                    }
                                    break;
                                case RIGHT:
                                    switch (pole) {
                                        case LOW:
                                            turret.setPosition(0.1);
                                            break;
                                        case MEDIUM:
                                            turret.setPosition(0.1);
                                            break;
                                        case HIGH:
                                            turret.setPosition(0.9);
                                            break;
                                        case FAR_HIGH:
                                            turret.setPosition(0.1);
                                            break;
                                    }
                                    break;
                            }


                        }

                        // If drive finishes its task, cede control to the driver
                        if (!drive.isBusy()) {
                            mode = Mode.DRIVER_CONTROL;
                        }
                        break;
                }
                break;

        }


        slidescontroller.setPID(ps, is, ds);
        int slidesleftpos = slidesleft.getCurrentPosition();
        double pidleft = slidescontroller.calculate(slidesleftpos, targets);
        double ffleft = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

        int slidesrightpos = slidesright.getCurrentPosition();
        double pidright = slidescontroller.calculate(slidesrightpos, targets);
        double ffright = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

        double powerleft = pidleft + ffleft;

        double powerright = pidright + ffright;

        slidesleft.setPower(powerleft);
        slidesright.setPower(powerright);

        telemetry.addData("posslides", slidesleftpos);
        telemetry.addData("target", targets);


        currentposleft = slidesleft.getCurrentPosition();
        currentposright = slidesright.getCurrentPosition();

        telemetry.addData("Left Spool", currentposleft);
        telemetry.addData("Right Spool", currentposright);

        telemetry.update();
        drive.update();




    }
}
