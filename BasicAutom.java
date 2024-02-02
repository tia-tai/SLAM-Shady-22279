package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class BasicAutom extends LinearOpMode {
    public Blinker control_Hub;
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    private Servo RightFinger;
    private Servo LeftFinger;
    private Servo axisX;
    
    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");

        /*Drive Motor Setup*/
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        
        axisX = hardwareMap.get(Servo.class, "AxisX");
        
        RightFinger = hardwareMap.get(Servo.class, "FingerRight");
        LeftFinger = hardwareMap.get(Servo.class, "FingerLeft");
        
        double defaultpos = 0.5;
        axisX.scaleRange(0.12, 0.75);
        
        waitForStart();
        
        
        if (opModeIsActive()) {
            RightFinger.setPosition(defaultpos);
            LeftFinger.setPosition(defaultpos);
            axisX.setPosition(0.08);
            
            int currFL = FrontLeft.getCurrentPosition();
            int currFR = FrontRight.getCurrentPosition();
            int currBL = BackLeft.getCurrentPosition();
            int currBR = BackRight.getCurrentPosition();

            int driveForward = -2000;

            FrontLeft.setTargetPosition(currFL + driveForward);
            FrontRight.setTargetPosition(currFR + driveForward);
            BackLeft.setTargetPosition(currBL + driveForward);
            BackRight.setTargetPosition(currBR + driveForward);
            
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double speed = 0.25;
            FrontLeft.setPower(speed);
            FrontRight.setPower(speed);
            BackLeft.setPower(speed);
            BackRight.setPower(speed);

            while (opModeIsActive() && (FrontLeft.isBusy() || FrontLeft.isBusy() || BackLeft.isBusy() || BackRight.isBusy())) {}

            speed = 0;
            FrontLeft.setPower(speed);
            FrontRight.setPower(speed);
            BackLeft.setPower(speed);
            BackRight.setPower(speed);
            
            RightFinger.setPosition(defaultpos - 0.13);
            LeftFinger.setPosition(defaultpos + 0.13);
        }
    }
}
