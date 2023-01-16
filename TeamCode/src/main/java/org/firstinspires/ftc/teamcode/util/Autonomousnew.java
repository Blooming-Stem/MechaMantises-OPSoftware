//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
//@Autonomous
//public class Autosnomousnew extends OpMode {
//    private PIDController slidescontroller;
//    public static double ps = 0.002, is = 0, ds = 0;
////    public static double fs = 0.01;
//    public static int targets = 0;
//    private final double ticks_in_degrees = 384 / 360.0;
//    private DcMotorEx slidesleft;
//    private DcMotorEx slidesright;
//    private Servo turret;
//    private Servo claw;
//
//
//
//    SampleMecanumDrive drive;
//    public void init() {
//        slidescontroller = new PIDController(ps, is, ds);
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        slidesleft = hardwareMap.get(DcMotorEx.class, "slidesLeft");
//        slidesright = hardwareMap.get(DcMotorEx.class, "slidesRight");
//
//        slidesleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slidesleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        slidesright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slidesright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slidesright.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        claw = hardwareMap.servo.get("claw");
//        turret = hardwareMap.servo.get("turret");
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        drive = new SampleMecanumDrive(hardwareMap);
//
//
//
//
//
//
//        claw.setPosition(1);
//        turret.setPosition(0.5);
//        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
//
//
//
//                .addDisplacementMarker(()->{
//                    targets = 2950;
//                })
//                .back(56)
//                .UNSTABLE_addTemporalMarkerOffset(-1.5,()->{
//                    turret.setPosition(0.1);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{
//                    claw.setPosition(0.3);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(()->{
//                    turret.setPosition(0.5);
//                    targets=500;
//                })
//                .forward(9)
//                .turn(Math.toRadians(-90))
//                .forward(25)
//                .addTemporalMarker(()->{
//                    claw.setPosition(1);
//                })
////                .waitSeconds(0.3)
////                .addTemporalMarker(()->{
////                    targets=700;
////                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{
//                    targets=2950;
//                })
//                .waitSeconds(0.7)
//                .back(31)
//                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
//                    turret.setPosition(0.9);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{
//                    claw.setPosition(0.3);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(()->{
//                    turret.setPosition(0.5);
//                    targets=400;
//                })
//                .forward(31)
//                .addTemporalMarker(()->{
//                    claw.setPosition(1);
//                })
////                .waitSeconds(0.3)
////                .addTemporalMarker(()->{
////                    targets=700;
////                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{
//                    targets=2950;
//                })
//                .waitSeconds(0.7)
//                .back(31)
//                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
//                    turret.setPosition(0.9);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{
//                    claw.setPosition(0.3);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(()->{
//                    turret.setPosition(0.5);
//                    targets=300;
//                })
//                .forward(31)
//                .addTemporalMarker(()->{
//                    claw.setPosition(1);
//                })
////                .waitSeconds(0.3)
////                .addTemporalMarker(()->{
////                    targets=700;
////                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{
//                    targets=2950;
//                })
//                .waitSeconds(0.7)
//                .back(31)
//                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
//                    turret.setPosition(0.9);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{
//                    claw.setPosition(0.3);
//                }).waitSeconds(0.5)
//                .addTemporalMarker(()->{
//                    turret.setPosition(0.5);
//                    targets=200;
//                })
//                .forward(31)
//                .addTemporalMarker(()->{
//                    claw.setPosition(1);
//                })
////                .waitSeconds(0.3)
////                .addTemporalMarker(()->{
////                    targets=700;
////                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{
//                    targets=2950;
//                })
//                .waitSeconds(0.7)
//                .back(31)
//                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
//                    turret.setPosition(0.9);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{
//                    claw.setPosition(0.3);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(()->{
//                    turret.setPosition(0.5);
//                    targets=10;
//
//                })
//
//
//
//
//                .build();
//        drive.followTrajectorySequenceAsync(traj1);
//
//    }
//    public void loop(){
//        drive.update();
//
//        slidescontroller.setPID(ps, is, ds);
//        int slidesleftpos = slidesleft.getCurrentPosition();
//        double pidleft = slidescontroller.calculate(slidesleftpos, targets);
//        double ffleft = Math.cos(Math.toRadians(targets/ticks_in_degrees))* fs;
//
//        int slidesrightpos = slidesright.getCurrentPosition();
//        double pidright = slidescontroller.calculate(slidesrightpos, targets);
//        double ffright = Math.cos(Math.toRadians(targets/ticks_in_degrees))* fs;
//
//        double powerleft = pidleft + ffleft;
//
//        double powerright = pidright + ffright;
//
//        slidesleft.setPower(powerleft);
//        slidesright.setPower(powerright);
//
//
//
//
//
//
//
//
//
//
//    }
//}
