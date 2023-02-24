/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;

@Autonomous
public class ParkAuto extends OpMode {


    boolean done=false;
    int state=0;


    //TW
    private PIDController twController;
    public static double Tp=0.03, Ti = 0, Td = 0.0014;
    public static double Tf = 0.27;

    public static int twTarget = 0;
    private final double ticksPerMM = 1.503876;
    private DcMotorEx towerRight;
    private DcMotorEx towerLeft;
    //ARM
    private PIDController armController;

    public static double Ap = 0.006, Ai = 0, Ad = 0.0006;
    public static double Af = 0.13;

    public static int armTarget = 0;
    private final double ticksPerRadian = 28 * (2.89655) * (3.61905) * (5.23077) * (2.4) / (2 * Math.PI);
    private DcMotorEx armMotor;

    private double targetX=0;
    private double targetY=0;

    //GR
    private Servo gripperRotationServo;
    private Servo alignmentBarServo;
    private CRServo frontRollerServo;
    private CRServo backRollerServo;
    private int retractAlignmentBar = 0;
    private final double alignmentBarDownPos = 0;
    private final double alignmentBarUpPos = 0.65;
    private InverseKinematics inverseKinematics;
    public static double gripperRotationServoPosition=1;


    //camera stuff
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag IDs of sleeve
    int left = 1;
    int middle = 2;
    int right = 3;
    int signal = 2;

    AprilTagDetection tagOfInterest = null;

    //roadrunner setup
    SampleMecanumDrive drive = null;

    //points of interest

    Pose2d startPose=null;
    Pose2d leftParkPose=null;
    Pose2d middleParkPose=null;
    Pose2d rightParkPose=null;
    Trajectory middlePark=null;
    Trajectory leftPark=null;
    Trajectory rightPark=null;

    ElapsedTime timer = null;


    public void init() {

        //bulk read stuff
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //setUpServos
        frontRollerServo = hardwareMap.get(CRServo.class, "frontRollerServo");
        backRollerServo = hardwareMap.get(CRServo.class, "backRollerServo");
        alignmentBarServo = hardwareMap.get(Servo.class, "alignmentBarServo");
        gripperRotationServo = hardwareMap.get(Servo.class, "gripperRotationServo");
        gripperRotationServoPosition=1;
        //setUp tower
        twController = new PIDController(Tp, Ti, Td);
        towerRight = hardwareMap.get(DcMotorEx.class, "towerRight");
        towerLeft = hardwareMap.get(DcMotorEx.class, "towerLeft");

        towerRight.setDirection(DcMotorEx.Direction.REVERSE);
        towerLeft.setDirection(DcMotorEx.Direction.FORWARD);
        towerRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        towerLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        //setUp arm
        armController = new PIDController(Ap, Ai, Ad);
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        inverseKinematics = new InverseKinematics(ticksPerRadian, ticksPerMM);

        towerLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        towerRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        towerRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        towerLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        towerRight.setPower(0);
        towerLeft.setPower(0);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(0);

        //ftc dashboard stuff
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        //camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        //roadrunner stuff

        //roadrunner setup
        drive = new SampleMecanumDrive(hardwareMap);


        startPose = new Pose2d(0, 0, Math.toRadians(0));
        leftParkPose = new Pose2d( 27, 28);
        middleParkPose = new Pose2d (27, 0);
        rightParkPose = new Pose2d (27, -28);
        middlePark = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(middleParkPose)
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();
        leftPark = drive.trajectoryBuilder(middlePark.end())
                .lineToLinearHeading(leftParkPose)
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();
        rightPark = drive.trajectoryBuilder(middlePark.end())
                .lineToLinearHeading(rightParkPose)
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();
        drive.setPoseEstimate(startPose);


    }


    public void init_loop() {
        //inverse kinematics
        int towerPos = towerRight.getCurrentPosition();
        int armPos = armMotor.getCurrentPosition();
        twController.setPID(Tp, Ti, Td);
        armController.setPID(Ap, Ai, Ad);
        //set targets
        targetX=288.43871245;
        targetY=-340.67571868;

        //calculate with invserse kinematics here
        int armTargetLast=armTarget;
        int twTargetLast=twTarget;
        if (inverseKinematics.calculate(targetX, targetY, armPos, towerPos)){
            armTarget=inverseKinematics.armTarget;
            twTarget=inverseKinematics.towerTarget;
        }

        //tower controller

        double towerPid = twController.calculate(towerPos, twTarget);
        double towerFf = Tf;

        double towerPower = towerPid + towerFf;
        towerRight.setPower(towerPower);
        towerLeft.setPower(towerPower);

        //arm controller
        armController.setPID(Ap, Ai, Ad);

        double armPid = armController.calculate(armPos, armTarget);
        double armFf = -Math.sin((armPos / ticksPerRadian)-0.236) * Af;

        double armPower = armPid + armFf;
        armMotor.setPower(armPower);

        //set servos
        gripperRotationServo.setPosition(gripperRotationServoPosition);
        alignmentBarServo.setPosition(0.35);

        telemetry.update();



        //iterate the april tags
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            boolean tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == left || tag.id == middle || tag.id == right) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }



        }
        if (tagOfInterest != null) {
            if (tagOfInterest.id == left) {
                telemetry.addLine("will go left");
                signal = left;
            } else if (tagOfInterest.id == right) {
                telemetry.addLine("will go right");
                signal = right;
                //tagToTelemetry(tagOfInterest);
            } else if (tagOfInterest.id == middle) {
                telemetry.addLine("will go middle");
                signal = middle;
            }
        } else {
            telemetry.addLine("will go middle Bc didn't find anything");
            signal = middle;
        }
        telemetry.update();
    }

    /*
     * The START command just came in: now work off the latest snapshot acquired
     * during the init loop.
     */

    public void start() {
        /* Update the telemetry */
        gripperRotationServoPosition=0.35;
        alignmentBarServo.setPosition(alignmentBarUpPos);
        frontRollerServo.setPower(0.1);
        backRollerServo.setPower(0.1);

        //set final parking zone from the camera
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            //tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }



        //finite state machine reset
    }


    public void loop(){
        if (state==0) {
            targetX = 0;
            targetY = 542.75598018;
            state+=1;
        }
        else if (state==1){
            if (withinTolerance(armController.getPositionError(), 100, twController.getPositionError(), 20)){
                state+=1;
            }
        }
        else if (state==2){
            drive.followTrajectoryAsync(middlePark);
            state+=1;
        }
        else if (state==4){
            if (signal == left) {
                drive.followTrajectoryAsync(leftPark);
            }
            else if (signal == right) {
                drive.followTrajectoryAsync(rightPark);

            }
            else{
                state+=1;
            }
            state+=1;
        }
        else if (state==6){
            targetX=388.43871245;
            targetY=-320.67571868;
            state+=1;
        }
        else if (state==7){
            if (withinTolerance(armController.getPositionError(), 100, twController.getPositionError(), 20)){
                state+=1;
            }
        }
        else if  (state==8){
            done=true;
        }

        telemetry.addData("currentFSMState", state);
        drive.update();


        //inverse kinemetics
        int towerPos = towerRight.getCurrentPosition();
        int armPos = armMotor.getCurrentPosition();
        double targetXLast=targetX;
        double targetYLast=targetY;

        //calculate with inverse kinematics
        if (inverseKinematics.calculate(targetX, targetY, armPos, towerPos)){
            armTarget=inverseKinematics.armTarget;
            twTarget=inverseKinematics.towerTarget;
        }
        else {
            targetX=targetXLast;
            targetY=targetYLast;
        }


        //tower controller
        twController.setPID(Tp, Ti, Td);
        double towerPid = twController.calculate(towerPos, twTarget);
        double towerFf = Tf;

        double towerPower = towerPid + towerFf;
        if (!done){
            towerRight.setPower(towerPower);
            towerLeft.setPower(towerPower);
        }
        else{
            towerRight.setPower(0);
            towerLeft.setPower(0);
        }


        //arm controller
        armController.setPID(Ap, Ai, Ad);

        double armPid = armController.calculate(armPos, armTarget);
        double armFf = -Math.sin((armPos / ticksPerRadian)-0.236) * Af;

        double armPower = armPid + armFf;
        if (!done){
            armMotor.setPower(armPower);
        }
        else{
            armMotor.setPower(0);
        }


        if (retractAlignmentBar > 0) {
            if (retractAlignmentBar == 1) {
                alignmentBarServo.setPosition(alignmentBarUpPos);
            }
            retractAlignmentBar -= 1;
        }
        gripperRotationServo.setPosition(gripperRotationServoPosition);





        telemetry.update();
    }
    public Boolean withinTolerance(double armError, int armTolerance, double towerError, int towerTolerance){
        if (Math.abs(armError)<=armTolerance && Math.abs(towerError)<=towerTolerance){
            return true;
        }
        else{
            telemetry.addLine("im not within tolerance yet");
            return false;
        }


    }
}