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
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;


@Autonomous
public class rightAuto extends OpMode {




    boolean done=false;
    String phase="toCycle";
    boolean depositing=false;
    boolean collecting=false;
    int state=0;
    double collectX=0;
    double collectY=0;




    //TW
    private PIDController twController;
    public static double Tp=0.035, Ti = 0, Td = 0.0014;
    public static double Tf = 0.06;



    public static int twTarget = 0;
    public static double towerMaxPower = 1;
    private final double ticksPerMM = 1.503876;
    private DcMotorEx towerRight;
    private DcMotorEx towerLeft;
    //ARM
    private PIDController armController;


    public static double Ap = 0.001, Ai = 0, Ad = 0.0008;
    public static double Af = 0.13;

    public static int armTarget = 0;
    public static double armMaxPower = 1;
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
    Vector2d cycle_positionVector=null;
    Trajectory middlePark=null;
    Trajectory leftPark=null;
    Trajectory rightPark=null;
    Trajectory middleParkFinal=null;
    Trajectory leftParkFinal=null;
    Trajectory rightParkFinal=null;
    Trajectory cycle_position=null;
    Trajectory transition=null;






    ElapsedTime timer = new ElapsedTime();




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




        startPose = new Pose2d(38.465, -61.8125, Math.toRadians(90));
        cycle_positionVector = new Vector2d(56, -8);



        cycle_position = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(31,-36), Math.toRadians(90))
                .splineTo(new Vector2d(50, -14), Math.toRadians(-14.0362))
                .splineToConstantHeading(cycle_positionVector, Math.toRadians(90+14.0362))
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();
        transition = drive.trajectoryBuilder(cycle_position.end())
                .lineToSplineHeading(new Pose2d(59, -24, Math.toRadians(-14.0362)))
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();
        leftPark = drive.trajectoryBuilder(transition.end())
                .lineToSplineHeading(new Pose2d(60, -34, Math.toRadians(270)))
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();
        middlePark = drive.trajectoryBuilder(leftPark.end())
                .lineToSplineHeading(new Pose2d(36, -34, Math.toRadians(270)) )
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();
        rightPark = drive.trajectoryBuilder(middlePark.end())
                .lineToSplineHeading(new Pose2d(12, -34, Math.toRadians(270)) )
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();
        leftParkFinal = drive.trajectoryBuilder(leftPark.end())
                .lineToConstantHeading(new Vector2d(60, -24))
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();
        middleParkFinal = drive.trajectoryBuilder(middlePark.end())
                .lineToConstantHeading(new Vector2d(36, -24) )
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();
        rightParkFinal = drive.trajectoryBuilder(rightPark.end())
                .lineToConstantHeading(new Vector2d(12, -24))
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();





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
        drive.setPoseEstimate(startPose);






        //finite state machine reset
    }




    public void loop() {
        if (Objects.equals(phase, "toCycle")) {
            if (state == 0) {
                targetX = 0;
                targetY = 542.75598018;
                state += 1;
            } else if (state == 1) {
                if (withinTolerance(armController.getPositionError(), 100, twController.getPositionError(), 20)) {
                    state += 1;
                }
            } else if (state == 2) {
                drive.followTrajectoryAsync(cycle_position);
                state += 1;
            } else if (state == 4) {
                phase = "deposit1";
                state = 0;
            }
        } else if (Objects.equals(phase, "deposit1")) { //deposit cone #1
            if (state != 0 && depositing == false) {
                phase = "collect2";
                state = 0;
            } else {
                depositing = true;
            }
        } else if (Objects.equals(phase, "collect2")){ //collect cone #2
            if (state != 0 && collecting == false) {
                phase = "deposit 2";
                state = 0;
            } else {
                collecting = true;
                collectX=485;
                collectY=-113;
            }
        } else if (Objects.equals(phase, "deposit2")) { //deposit cone #2
            if (state != 0 && depositing == false) {
                phase = "collect3";
                state = 0;
            } else {
                depositing = true;
            }
        } else if (Objects.equals(phase, "collect3")){ //collect cone #3
            if (state != 0 && collecting == false) {
                phase = "deposit 3";
                state = 0;
            } else {
                collecting = true;
                collectX=485;
                collectY=-160;
            }
        } else if (Objects.equals(phase, "deposit3")) { //deposit cone #3
            if (state != 0 && depositing == false) {
                phase = "collect4";
                state = 0;
            } else {
                depositing = true;
            }
        } else if (Objects.equals(phase, "collect4")){ //collect cone #4
            if (state != 0 && collecting == false) {
                phase = "deposit 4";
                state = 0;
            } else {
                collecting = true;
                collectX=485;
                collectY=-180;
            }
        }else if (Objects.equals(phase, "deposit4")) { //deposit cone #4
            if (state != 0 && depositing == false) {
                phase = "collect5";
                state = 0;
            } else {
                depositing = true;
            }
        } else if (Objects.equals(phase, "collect5")){ //collect cone #5
            if (state != 0 && collecting == false) {
                phase = "deposit 5";
                state = 0;
            } else {
                collecting = true;
                collectX=485;
                collectY=-200;
            }
        }else if (Objects.equals(phase, "deposit5")) { //deposit cone #5
            if (state != 0 && depositing == false) {
                phase = "collect6";
                state = 0;
            } else {
                depositing = true;
            }
        } else if (Objects.equals(phase, "collect6")){ //collect cone #6
            if (state != 0 && collecting == false) {
                phase = "deposit 6";
                state = 0;
            } else {
                collecting = true;
                collectX=485;
                collectY=-220;
            }
        }else if (Objects.equals(phase, "deposit6")) { //deposit cone #6
            if (state != 0 && depositing == false) {
                phase = "park";
                state = 0;
            } else {
                depositing = true;
            }
        }else if (Objects.equals(phase, "park")) {
            if (state == 0) {
                targetX = 0;
                targetY = 542.75598018;
                state += 1;
            } else if (state == 1) {
                if (withinTolerance(armController.getPositionError(), 100, twController.getPositionError(), 20)) {
                    state += 1;
                }
            } else if (state == 2) {
                drive.followTrajectoryAsync(transition);
                state+=1;
            } else if (state == 4) {
                drive.followTrajectoryAsync(leftPark);
                state+=1;
            } else if (state == 6) {
                if (signal==left){
                    state+=2;
                } else if (signal==middle){
                    drive.followTrajectoryAsync(middlePark);
                    state+=1;
                } else if (signal==right){
                    drive.followTrajectoryAsync(rightPark);
                    state+=1;
                }
            } else if (state == 8){
                if (signal==left){
                    drive.followTrajectoryAsync(leftParkFinal);
                } else if (signal==middle){
                    drive.followTrajectoryAsync(middleParkFinal);
                } else if (signal==right){
                    drive.followTrajectoryAsync(rightParkFinal);
                }
                state+=1;
            } else if (state == 10) {
                phase = "finish";
                state = 0;
            }


        } else if (Objects.equals(phase, "finish")) {
            if (state == 0) {
                targetX=308.43871245;
                targetY=-300.67571868;
                gripperRotationServoPosition=1;
                alignmentBarServo.setPosition(0.35);
                state += 1;
            } else if (state == 1) {
                if (withinTolerance(armController.getPositionError(), 100, twController.getPositionError(), 20)) {
                    state += 1;
                }
            } else if (state == 2) {
                done = true;
            }
        }


        if (depositing == true) { //deposit macro if depsiting is set to true it will caryout nesecary deposting steps then when finish sets depositng to false
            if (state == 0) { //to above junction
                targetX = -396;
                targetY = 848;
                state += 1;
            } else if (state == 1) {
                if (withinTolerance(armController.getPositionError(), 50, twController.getPositionError(), 10)) {
                    state += 1;
                    alignmentBarServo.setPosition(alignmentBarDownPos);
                    timer.reset();
                }
            } else if (state == 2) {
                if (timer.milliseconds() >= 500) {//move down
                    targetX = -398;
                    targetY = 741;
                    state+=1;
                }
            } else if (state == 3) {
                if (timer.milliseconds() >= 1000) {//deposit
                    frontRollerServo.setPower(-1);
                    backRollerServo.setPower(-1);
                    retractAlignmentBar=5;
                    state += 1;
                }
            } else if (state == 4) {
                if (timer.milliseconds() >= 1500) {
                    frontRollerServo.setPower(0);
                    backRollerServo.setPower(0);
                    targetX = -396;
                    targetY = 848;
                }
                if (timer.milliseconds() >= 1700) {
                    depositing = false;
                    state=0;
                }
            }

        }
        if (collecting == true) { //collect macro if collecting is set to true it will caryout nesecary collecting steps then when finish sets colecting to false
            if (state == 0) { //to above junction
                targetX = 486;
                targetY = -48;
                state += 1;
                alignmentBarServo.setPosition(alignmentBarUpPos);
            } else if (state == 1) {
                if (withinTolerance(armController.getPositionError(), 50, twController.getPositionError(), 10)) {
                    state += 1;
                    targetX = collectX;
                    targetY = collectY;
                    frontRollerServo.setPower(1);
                    backRollerServo.setPower(1);
                    timer.reset();
                }
            } else if (state == 2) {
                if (timer.milliseconds() >= 1000) {
                    targetX = 486;
                    targetY = -48;
                    frontRollerServo.setPower(0.1);
                    backRollerServo.setPower(0.1);
                    state+=1;
                }
                if (timer.milliseconds() >= 1300) {
                    collecting = false;
                    state=0;
                }
            }

        }


        telemetry.addData("currentFSMState", state);
        telemetry.addData("phase", phase);
        drive.update();


        //inverse kinemetics
        int towerPos = towerRight.getCurrentPosition();
        int armPos = armMotor.getCurrentPosition();
        double targetXLast = targetX;
        double targetYLast = targetY;


        //calculate with inverse kinematics
        if (inverseKinematics.calculate(targetX, targetY, armPos, towerPos)) {
            armTarget = inverseKinematics.armTarget;
            twTarget = inverseKinematics.towerTarget;
        } else {
            targetX = targetXLast;
            targetY = targetYLast;
        }


        //tower controller
        twController.setPID(Tp, Ti, Td);
        double towerPid = twController.calculate(towerPos, twTarget);
        if (towerPid > towerMaxPower) {
            towerPid = towerMaxPower;
        }
        if (towerPid < -towerMaxPower) {
            towerPid = -towerMaxPower;
        }
        double towerFf = Tf;


        double towerPower = towerPid + towerFf;
        if (!done) {
            towerRight.setPower(towerPower);
            towerLeft.setPower(towerPower);
        } else {
            towerRight.setPower(0);
            towerLeft.setPower(0);
        }


        //arm controller
        armController.setPID(Ap, Ai, Ad);


        double armPid = armController.calculate(armPos, armTarget);
        if (armPid > armMaxPower) {
            armPid = armMaxPower;
        }
        if (armPid < -armMaxPower) {
            armPid = -armMaxPower;
        }
        double armFf = -Math.sin((armPos / ticksPerRadian) - 0.236) * Af;


        double armPower = armPid + armFf;
        if (!done) {
            armMotor.setPower(armPower);
        } else {
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
    public Boolean withinTolerance(double armError, int armTolerance, double towerError, int towerTolerance){ //arm position checking function to allow only moving on if it is in pos
        if (Math.abs(armError)<=armTolerance && Math.abs(towerError)<=towerTolerance){
            return true;
        }
        else{
            telemetry.addLine("im not within tolerance yet");
            return false;
        }




    }
}





