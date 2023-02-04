package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")

public class TeleOp2023V1 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorBackRight = null;
    private DcMotor motorFrontRight = null;
    private DcMotor motorBackLeft = null;
    private DcMotor motorFrontLeft = null;
    private DcMotor armMotor = null;
    private DcMotor towerMain = null;
    private CRServo gripperServo1 = null;
    private CRServo gripperServo2 = null;
    private Servo alignmentBarServo = null;
    private Servo gripperRotationServo = null;
    double towerTarget;
    double towerTargetLast;
    double armTarget;
    double armTargetLast;
    double targetX;
    double targetXLast;
    double targetY;
    double targetYLast;
    double retractAlignmentBar;
    Boolean auto;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        //set true if auto set false is not
        auto=false;
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorFrontRight  = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackLeft  = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        towerMain = hardwareMap.get(DcMotor.class, "towerMain");
        gripperServo1 = hardwareMap.get(CRServo.class, "gripperServo1");
        gripperServo2 = hardwareMap.get(CRServo.class, "gripperServo2");
        alignmentBarServo = hardwareMap.get(Servo.class, "alignmentBarServo");
        gripperRotationServo = hardwareMap.get(Servo.class, "gripperRotationServo");
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        towerMain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        towerMain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(0);
        armMotor.setPower(0.7);


        towerMain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        towerMain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        towerTarget = 0;
        armTarget = 0;
        targetY=542.75588018;
        targetX=0;
        retractAlignmentBar=-5;
        towerMain.setTargetPosition((int)(towerTarget*-5.4794));
        towerMain.setPower(1);



        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Im even more different");
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        if (auto){
            towerMain.setTargetPosition(0);
            armMotor.setTargetPosition(0);
        }
        else{

            towerMain.setTargetPosition(-670);
            armMotor.setTargetPosition(2057);
        }
        gripperRotationServo.setPosition(1);
        alignmentBarServo.setPosition(0.5);
    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        telemetry.addData("targetX", targetX);
        telemetry.addData("targetY", targetY);
        //telemetry.addData("tower1MotorPower", alignmentBarServo.getPosition());

        //alignmentBarServo.setPosition(alignmentBarServo.getPosition()+gamepad2.right_stick_y/200);

        gripperRotationServo.setPosition(0.35);
        if (gamepad2.x){
            alignmentBarServo.setPosition(0.01222);
        }
        if (gamepad2.y){
            alignmentBarServo.setPosition(0.73111);

        }
        mechanum();
        armInputs();
        gripper();
        if (gamepad1.left_bumper){
            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (gamepad1.right_bumper){
            motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        //towerMain.setPower(gamepad2.right_stick_y);


        towerMain.setTargetPosition((int)(towerTarget*-5.4794));
        armMotor.setTargetPosition((int)(armTarget*848.1303));
        if (retractAlignmentBar>1){
            retractAlignmentBar-=1;
        }
        if (retractAlignmentBar<=1&&retractAlignmentBar>0){
            retractAlignmentBar-=2;
            alignmentBarServo.setPosition(0.73111);
        }

    }
    private void mechanum() {
        double mechanumY = Math.pow(gamepad1.left_stick_y, 3) + Math.pow(-gamepad2.right_stick_y, 3)*0.5;
        double mechanumX = Math.pow(-gamepad1.left_stick_x, 3) * 1.1+Math.pow(-gamepad2.right_stick_x, 3)*0.5;
        double mechanumRX = Math.pow(-gamepad1.right_stick_x, 3);
        double denominator = Math.max(Math.abs(mechanumY) + Math.abs(mechanumX) + Math.abs(mechanumRX), 1);
        double frontLeftPower = (mechanumY + mechanumX + mechanumRX) / denominator;
        double backLeftPower = (mechanumY - mechanumX + mechanumRX) / denominator;
        double frontRightPower = (mechanumY - mechanumX - mechanumRX) / denominator;
        double backRightPower = (mechanumY + mechanumX - mechanumRX) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
    }
    private void armInputs(){
        towerTargetLast=towerTarget;
        armTargetLast=armTarget;
        targetXLast=targetX;
        targetYLast=targetY;
        //intakePosition
        if (gamepad2.right_bumper){
            targetX=414.6;
            targetY=-140.9;
            alignmentBarServo.setPosition(0.73111);
        }
        //groundJunction
        if (gamepad2.dpad_down){
            targetX=-133.9373;
            targetY=-157.6818;
            alignmentBarServo.setPosition(0.73111);
        }
        //lowJunction
        if (gamepad2.dpad_left){
            targetX=-483.1;
            targetY=773.17;
            alignmentBarServo.setPosition(0.73111);
        }
        //midJunction
        if (gamepad2.dpad_right){
            targetX=447.9;
            targetY=-248.1;
            alignmentBarServo.setPosition(0.73111);

        }
        //highJunction
        if (gamepad2.dpad_up){
            targetX=15;
            targetY=722.4;
            alignmentBarServo.setPosition(0.73111);
        }

        targetX-=Math.pow(gamepad2.left_stick_x,3)*4;
        targetY-=Math.pow(gamepad2.left_stick_y,3)*4;
        inverseKinematics();
    }
    private void inverseKinematics(){
        if (targetXLast!=targetX || targetYLast!=targetX){
            double prelimCompute=targetX/542.75588018;
            if (Math.abs(prelimCompute)>1){
                targetY=targetYLast;
                targetX=targetXLast;
                towerTarget=towerTargetLast;
                armTarget=armTargetLast;
            }
            else {
                double armP1=Math.asin(prelimCompute);
                double armP2=Math.PI-armP1;
                if (armP2>Math.PI){
                    armP2-=2*Math.PI;
                }
                if (armP2<-1*Math.PI){
                    armP2+=2*Math.PI;
                }
                double towerP1=targetY-Math.cos(armP1)*542.75588018;
                double towerP2=targetY-Math.cos(armP2)*542.75588018;
                boolean positionValid1=axisTravelCheck(armP1, towerP1);
                boolean positionValid2=axisTravelCheck(armP2, towerP2);

                if (positionValid1 && positionValid2){
                    //best solution is chosen based on which will require minimum change of arm angle and the other will be made invalid
                    double armChangeP1=Math.abs(armP1-armTarget);
                    double armChangeP2=Math.abs(armP2-armTarget);
                    if (armChangeP1<=armChangeP2){
                        positionValid2=false;
                    }
                    else{
                        positionValid1=false;
                    }
                }
                //if only one solution will work need to set targets to that solution
                if (positionValid1){
                    towerTarget=towerP1;
                    armTarget=armP1;

                }
                else {
                    if (positionValid2){
                        towerTarget=towerP2;
                        armTarget=armP2;
                    }
                    else{
                        targetY=targetYLast;
                        targetX=targetXLast;
                        towerTarget=towerTargetLast;
                        armTarget=armTargetLast;
                    }
                }
            }
        }
    }
    private void gripper(){
        if (gamepad2.a){
            gripperServo1.setPower(0.3);
            gripperServo2.setPower(1);
            alignmentBarServo.setPosition(0.73111);

        }
        else{
            if (gamepad2.b){
                gripperServo1.setPower(-0.3);
                gripperServo2.setPower(-1);
                retractAlignmentBar=4;
            }
            else{
                gripperServo1.setPower(0.1);
                gripperServo2.setPower(0.1);
            }
        }
    }
    public boolean axisTravelCheck(double armAngle, double towerPosition){
        if (towerPosition<0 || towerPosition>630 || armAngle<-2.889 || armAngle>2.889){
            return false;
        }
        else{
            return true;
        }
    }
    /*
     * Code to run ONCE after the driver hits STOP hehehehehe
     */
    @Override
    public void stop() {
    }
}