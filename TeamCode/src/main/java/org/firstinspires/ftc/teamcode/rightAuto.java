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

//your code is updated

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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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
import java.util.Objects;
import java.util.Timer;


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
    public static double Tp=0.014, Ti = 0, Td = 0.00055;
    public static double Tf = 0.175, Ts=0.175, towerTolerance=6;//0.27;

    public static int twTarget = 0;
    private final double ticksPerMM = 2.02696328861;
    private DcMotorEx towerRight;
    private DcMotorEx towerLeft;

    public static double towerMaxPower = 1.5;

    //ARM
    private PIDController armController;

    public static double Ap = 0.002, Ai = 0, Ad = 0.00013;
    public static double Af = 0.11, As = 0, armTolerance= 6;

    public static int armTarget = 0;
    private final double ticksPerRadian = 8192 / (2 * Math.PI);
    private static int armOffset=3025;
    private DcMotorEx armMotor;

    public static double armMaxPower = 1.5;



    private double targetX=0;
    private double targetY=0;

    //GR
    private Servo gripperRotationServo;
    private Servo alignmentBarServo;
    private Servo leftClaw;
    private Servo rightClaw;
    private Servo wheelieBarServo;
    public static double wheelieBarPosition=.56;
    private int retractAlignmentBar = 0;
    public static int retractAlignmentBarDelay=5;
    public static double alignmentBarDownPos = 0.8;
    public static double alignmentBarMidPos = 0.35;
    public static double alignmentBarUpPos = 0.05;
    private InverseKinematics inverseKinematics;
    public static double gripperRotationServoPosition=0.5;
    public static double gripperOpenPos=0.2;
    public static double gripperHalfOpenPos=0.03;
    public static double gripperContainmentpos=0.06;
    public static double gripperClosePos=-0.07;
    public static double gripperCloseTight=-.15;
    public static double gripperClosePrep=0.06;
    public String gripperState="open";



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
    int signal = 3;


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
    Pose2d preCyclePose=null;






    ElapsedTime timer = new ElapsedTime();
    ElapsedTime loopTime = new ElapsedTime();
    ElapsedTime timeSinceStart = new ElapsedTime();




    public void init() {


        //bulk read stuff
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);


        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        //setUpServos
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        alignmentBarServo = hardwareMap.get(Servo.class, "alignmentBarServo");
        gripperRotationServo = hardwareMap.get(Servo.class, "gripperRotationServo");
        wheelieBarServo = hardwareMap.get(Servo.class, "wheelieBarServo");
        wheelieBarServo.setPosition(wheelieBarPosition);
        gripperRotationServoPosition=0.5;
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




        startPose = new Pose2d(36.895, -61.3125, Math.toRadians(90));
        cycle_positionVector = new Vector2d(57.5, -6.75);



        cycle_position = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(36,-24))
                .splineTo(new Vector2d(49.5, -14), Math.toRadians(-14.0392))//og angle is 194.0362
                .splineToConstantHeading(cycle_positionVector, Math.toRadians(90-14.0362))
                .addDisplacementMarker(() -> {
                    state+=1;
                    wheelieBarPosition=0.03;
                    wheelieBarServo.setPosition(wheelieBarPosition);
                    preCyclePose=drive.getPoseEstimate();
                    drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    drive.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    drive.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                })
                .build();
        rightPark = drive.trajectoryBuilder(cycle_position.end())
                .addDisplacementMarker(() -> {
                    wheelieBarPosition=0.56;
                    wheelieBarServo.setPosition(wheelieBarPosition);
                    drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    drive.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    drive.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                })
                .lineToSplineHeading(new Pose2d(61, -11.75, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();
        middlePark = drive.trajectoryBuilder(rightPark.end())
                .lineToConstantHeading(new Vector2d(36.5, -11.75),
                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();
        leftPark = drive.trajectoryBuilder(rightPark.end())
                .lineToConstantHeading(new Vector2d(13, -11.75),
                        SampleMecanumDrive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(25))
                .addDisplacementMarker(() -> {
                    state+=1;
                })
                .build();


    }




    public void init_loop() {
        armMotor.setPower(0);
        towerRight.setPower(0);
        towerLeft.setPower(0);

        //set servos
        gripperRotationServo.setPosition(gripperRotationServoPosition);
        alignmentBarServo.setPosition(alignmentBarUpPos);
        wheelieBarPosition=0.56;
        wheelieBarServo.setPosition(wheelieBarPosition);

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
            telemetry.addLine("will go right Bc didn't find anything");
            signal = right;
        }
        telemetry.update();
    }


    /*
     * The START command just came in: now work off the latest snapshot acquired
     * during the init loop.
     */


    public void start() {
        /* Update the telemetry */
        timeSinceStart.reset();
        gripperRotationServoPosition=0.5;
        alignmentBarServo.setPosition(alignmentBarUpPos);
        gripperState="contain";
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
        double targetXLast = targetX;
        double targetYLast = targetY;
        if (Objects.equals(phase, "toCycle")) {
            if (state == 0) {
                armMaxPower=0;
                towerMaxPower=0;
                gripperState = "containment";

                targetX = 330;
                targetY = -296; //10 HIGHER THAN NORMAL INTAKEPOS
                state += 1;
                timer.reset();
            } else if (state == 1) {
                if (timer.milliseconds() > 60 && gamepad2.a) {
                    armMaxPower=1.5;
                    towerMaxPower=1.5;
                    targetX = 330;
                    targetY = -296; //10 HIGHER THAN NORMAL INTAKEPOS
                    state += 1;
                }
            } else if (state == 2) {
                if (timer.milliseconds()>300 && gamepad2.a) {
                    gripperState = "closeTight";
                    state+=1;
                }
            } else if (state == 3) {
                if (timer.milliseconds()>500 && gamepad2.b){
                    state+=1;
                    targetX=231;
                    targetY=515;
                    towerMaxPower=0;
                }
            } else if (state == 4) {
                if (withinTolerance(armController.getPositionError(), 300, twController.getPositionError(), 10000)) {
                    state += 1;
                }
            } else if (state == 5 && gamepad2.a) {
                drive.followTrajectoryAsync(cycle_position);
                state += 1;
            } else if (state == 7 && gamepad2.b) { //found bug caused used to be 4 not 5
                phase = "deposit1";
                state = 0;
            }
        } else if (Objects.equals(phase, "deposit1")) { //deposit cone #1
            if (state != 0 && depositing == false && gamepad2.a) {
                phase = "collect2";
                state = 0;
            } else {
                depositing = true;
            }
        } else if (Objects.equals(phase, "collect2")){ //collect cone #2
            if (state != 0 && collecting == false && gamepad2.a) {
                phase = "deposit2";
                state = 0;
            } else {
                collecting = true;
                gripperRotationServoPosition=.5;
                collectX=403-10; //og is +10 on x
                collectY=-165; // og is +20 on y
            }
        } else if (Objects.equals(phase, "deposit2")) { //deposit cone #2
            if (state != 0 && depositing == false && gamepad2.a) {
                phase = "collect3";
                state = 0;
            } else {
                depositing = true;
            }
        } else if (Objects.equals(phase, "collect3")){ //collect cone #3
            if (state != 0 && collecting == false && gamepad2.a) {
                phase = "deposit3";
                state = 0;
            } else {
                collecting = true;
                gripperRotationServoPosition=.5;
                collectX=397-10;
                collectY=-197;
            }
        } else if (Objects.equals(phase, "deposit3")) { //deposit cone #3
            if (state != 0 && depositing == false && gamepad2.a) {
                phase = "collect4";
                state = 0;
            } else {
                depositing = true;
            }
        } else if (Objects.equals(phase, "collect4")){ //collect cone #4
            if (state != 0 && collecting == false && gamepad2.a) {
                phase = "deposit4";
                state = 0;
            } else {
                collecting = true;
                gripperRotationServoPosition=.5;
                collectX=393-10;
                collectY=-229;
            }
        }else if (Objects.equals(phase, "deposit4")) { //deposit cone #4
            if (state != 0 && depositing == false && gamepad2.a) {
                phase = "collect5";
                state = 0;
            } else {
                depositing = true;
            }
        } else if (Objects.equals(phase, "collect5")){ //collect cone #5
            if (state != 0 && collecting == false && gamepad2.a) {
                phase = "deposit5";
                state = 0;
            } else {
                collecting = true;
                gripperRotationServoPosition=0.5;
                collectX=385-10;
                collectY=-261;
            }
        }else if (Objects.equals(phase, "deposit5")) { //deposit cone #5
            if (state != 0 && depositing == false && gamepad2.a) {
                phase = "collect6";
                state = 0;
            } else {
                depositing = true;
            }
        } else if (Objects.equals(phase, "collect6")){ //collect cone #6
            if (state != 0 && collecting == false && gamepad2.a) {
                phase = "deposit6";
                state = 0;
            } else {
                collecting = true;
                gripperRotationServoPosition=0.5;
                collectX=379-10;
                collectY=-293;
            }
        }else if (Objects.equals(phase, "deposit6")) { //deposit cone #6
            if (state != 0 && depositing == false && gamepad2.a) {
                phase = "park";
                state = 0;
                wheelieBarPosition=0.56;
                wheelieBarServo.setPosition(wheelieBarPosition);
            } else {
                depositing = true;
            }
        }else if (Objects.equals(phase, "park")) {
            if (state == 0) {
                targetX=247;
                targetY=-13;
                gripperState="close";
                gripperRotationServoPosition=.56;
                alignmentBarServo.setPosition(alignmentBarUpPos);
                state += 1;
            } else if (state == 1) {
                drive.setPoseEstimate(preCyclePose);
                drive.followTrajectoryAsync(rightPark);
                state+=1;
            } else if (state == 3) {
                if (signal==right){
                    state+=2;
                } else if (signal==middle){
                    drive.followTrajectoryAsync(middlePark);
                    state+=1;
                } else if (signal==left){
                    drive.followTrajectoryAsync(leftPark);
                    state+=1;
                }
            } else if (state == 5) {
                phase = "finish";
                state = 0;
            }


        } else if (Objects.equals(phase, "finish")) {
            if (state == 0) {
                gripperRotationServoPosition=.5;
                alignmentBarServo.setPosition(alignmentBarUpPos);
                state += 1;
            } else if (state == 1) {
                if (withinTolerance(armController.getPositionError(), 100, twController.getPositionError(), 100)) {
                    state += 1;
                }
            } else if (state == 2) {
                done = true;
            }
        }


        if (depositing == true) { //deposit macro if depsiting is set to true it will caryout nesecary deposting steps then when finish sets depositng to false
            if (state == 0) { //to above junction
                if (timeSinceStart.milliseconds()>=400000000){
                    state=50;
                    depositing = false;
                } else {
                    towerMaxPower=1.5;
                    targetX = -433;//og -483
                    targetY = 780;//og 825
                    state += 1;
                    timer.reset();
                }
            } else if (state == 1) {
                if (timer.milliseconds()>500){
                    gripperRotationServoPosition=0.5;
                    alignmentBarServo.setPosition(alignmentBarMidPos);
                }
                if (withinTolerance(armController.getPositionError(), 20, twController.getPositionError(), 30) || timer.milliseconds()>5000)  {
                    state += 1;
                    alignmentBarServo.setPosition(alignmentBarDownPos);
                    gripperRotationServoPosition=0.5;
                    timer.reset();
                }
            } else if (state == 2 && gamepad2.a) {
                if (timer.milliseconds() >= 450) {//drop
                    targetX = -433;
                    targetY = 700;
                    state+=1;
                    timer.reset();
                }
            } else if (state == 3) {
                if (timer.milliseconds() >= 100 & gamepad2.a) {
                    gripperState="open";
                    retractAlignmentBar=retractAlignmentBarDelay;
                    timer.reset();
                    state+=1;
                }
            } else if (state == 4) {
                if (timer.milliseconds() >= 80) {
                    depositing = false;
                }
            }

        }
        if (collecting == true) { //collect macro if collecting is set to true it will caryout nesecary collecting steps then when finish sets colecting to false
            if (state == 0) { //to above junction
                if (timeSinceStart.milliseconds()>=400000000){
                    state=50;
                    collecting = false;
                }
                else {
                    targetX = 361; //og x=401
                    targetY = -36;
                    state += 1;
                    alignmentBarServo.setPosition(alignmentBarUpPos);
                    timer.reset();
                }

            } else if (state == 1 && gamepad2.b) {
                if (withinTolerance(armController.getPositionError(), 40, twController.getPositionError(), 40) || timer.milliseconds()>=5000) {
                    state += 1;
                    targetX = collectX;
                    targetY = collectY;
                    gripperState = "closePrep";
                    timer.reset();
                }
            } else if (state == 2 && gamepad2.b) {
                if (timer.milliseconds() >= 500) {
                    gripperState = "closeTight";
                    state += 1;
                    timer.reset();
                }
            } else if(state == 3 && gamepad2.b){
                if (timer.milliseconds() >= 250) {
                    targetX=411;
                    targetY=-36;
                    state+=1;
                    timer.reset();
                }
            } else if (state == 4 && gamepad2.b) {
                if (timer.milliseconds() >= 150) {
                    collecting = false;
                }
            }

        }
        telemetry.addData("loopTime", loopTime.milliseconds());
        loopTime.reset();
        telemetry.addData("dtError", drive.getPoseEstimate());
        telemetry.addData("cyclePos", cycle_position.end());
        telemetry.addData("currentFSMState", state);
        telemetry.addData("phase", phase);
        telemetry.addData("depositing?", depositing);
        telemetry.addData("collecting", collecting);
        telemetry.addData("targetX", targetX);
        telemetry.addData("targetY", targetY);
        telemetry.addData("armTarget", armTarget);
        telemetry.addData("armError", armController.getPositionError());
        telemetry.addData("towerTarget", twTarget);
        telemetry.addData("towerError", twController.getPositionError());
        drive.update();





        //inverse kinemetics

        //calculate with inverse kinematics
        int towerPos = (int)Math.round((towerLeft.getCurrentPosition()+towerRight.getCurrentPosition())/2);
        int armPos = armMotor.getCurrentPosition()+armOffset;

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
        double towerFf = Tf;
        double towerFs=0;
        if (twController.getPositionError()>towerTolerance){ //static friction code
            towerFs=Ts;
        } else if(twController.getPositionError()<-towerTolerance) {
            towerFs = -Ts;
        }
        double towerPower = towerPid + towerFs;
        towerPower+=towerFf;
        if (towerPower>towerMaxPower){
            towerPower=towerMaxPower;
        }
        if (towerPower<-towerMaxPower){
            towerPower=-towerMaxPower;
        }

        //arm controller
        armController.setPID(Ap, Ai, Ad);
        double armPid = armController.calculate(armPos, armTarget);
        double armFf = -Math.sin((armPos / ticksPerRadian)-0.236) * Af;
        double armFs = 0;
        if (armController.getPositionError()>armTolerance){
            armFs=As;
        } else if(armController.getPositionError()<-armTolerance) {
            armFs=-As;
        }
        double armPower = armPid + armFs;
        if (armPower>armMaxPower){
            armPid=armMaxPower;
        }
        if (armPid<-armMaxPower){
            armPid=-armMaxPower;
        }
        armPower+=armFf;
        if (!done){
            towerRight.setPower(towerPower);
            towerLeft.setPower(towerPower);
            armMotor.setPower(armPower);
        }
        else{
            armMotor.setPower(0);
            towerRight.setPower(0);
            towerLeft.setPower(0);
        }


        //gripper controller
        if (gripperState=="containment"){
            leftClaw.setPosition(0.5+gripperContainmentpos);
            rightClaw.setPosition(0.5-gripperContainmentpos);
        } else if (gripperState=="open"){
            if (armPos > 1000){
                leftClaw.setPosition(0.5 + gripperOpenPos);
                rightClaw.setPosition(0.5 - gripperOpenPos);
                telemetry.addData("gripperState", 1);
            } else {
                leftClaw.setPosition(0.5 + gripperHalfOpenPos);
                rightClaw.setPosition(0.5 - gripperHalfOpenPos);
                telemetry.addData("gripperState", 2);
            }
        } else if (gripperState=="close"){
            leftClaw.setPosition(0.5+gripperClosePos);
            rightClaw.setPosition(0.5-gripperClosePos);
        } else if (gripperState=="closeTight"){
            leftClaw.setPosition(0.5+gripperCloseTight);
            rightClaw.setPosition(0.5-gripperCloseTight);
        } else if (gripperState=="closePrep"){
            leftClaw.setPosition(0.5+gripperClosePrep);
            rightClaw.setPosition(0.5+gripperClosePrep);
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





