/*
Copyright 2023 FIRST Tech Challenge Team 22282

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp

public class Practice extends LinearOpMode {
    public Blinker control_Hub;
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    private DcMotorEx ArmLeft;
    private DcMotorEx ArmRight;
    private PIDController armPID;
    private DcMotor FlyWheel;
    private Servo RightFinger;
    private Servo LeftFinger;
    private Servo axisY;
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

        /*Arm Motor Setup*/
        ArmLeft = hardwareMap.get(DcMotorEx.class, "ArmLeft");
        ArmRight = hardwareMap.get(DcMotorEx.class, "ArmRight");

        ArmLeft.setDirection(DcMotor.Direction.REVERSE);
        ArmRight.setDirection(DcMotor.Direction.FORWARD);

        ArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ArmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ArmLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armPID = new PIDController(0.0003, 0, 0);

        RightFinger = hardwareMap.get(Servo.class, "FingerRight");
        LeftFinger = hardwareMap.get(Servo.class, "FingerLeft");

        axisY = hardwareMap.get(Servo.class, "AxisY");
        axisX = hardwareMap.get(Servo.class, "AxisX");

        axisX.scaleRange(0.12, 0.75);

        /* Fly Wheel Setup*/
        FlyWheel = hardwareMap.get(DcMotor.class, "FlyWheel");
        FlyWheel.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        double defaultAxis = 0.71;
        double axisXPosition = defaultAxis;
        double Rdefaultpos = 0.7;
        double Ldefaultpos = 0.;
        int ARM_RAISED_POSITION = 290;
        int ARM_DEFAULT_POSITION = 50;
        int ARM_TARGET_POSITION = ARM_DEFAULT_POSITION;

        axisX.setPosition(defaultAxis);
        axisY.setPosition(0);

        int armTarget = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*Precision Drive*/
            double Multiplier = 1.0;
            if (gamepad1.right_trigger == 1.0) {
                Multiplier = 0.2;
            }

            /*Mecanum Wheel Drive*/
            double max;
            double twist = -gamepad1.right_stick_x * Multiplier;
            double strafe = -gamepad1.left_stick_x * 2 * Multiplier;
            double drive = gamepad1.left_stick_y * Multiplier;

            double[] speeds = {
                    (drive + strafe + twist),
                    (drive - strafe - twist),
                    (drive - strafe + twist),
                    (drive + strafe - twist)
            };

            max = Math.max(Math.abs(speeds[0]), Math.abs(speeds[1]));
            max = Math.max(max, Math.abs(speeds[2]));
            max = Math.max(max, Math.abs(speeds[3]));

            if (max > 1.0) {
                speeds[0] /= max;
                speeds[1] /= max;
                speeds[2] /= max;
                speeds[3] /= max;
            }

            FrontLeft.setPower(speeds[0]);
            FrontRight.setPower(speeds[1]);
            BackLeft.setPower(speeds[2]);
            BackRight.setPower(speeds[3]);

            /*Arm Control*/

            if (gamepad1.x) {
                armTarget = 1;
            }
            if (gamepad1.y) {
                armTarget = -1;
            }
            telemetry.addData("arm target", armTarget);
            double armPower = 0;
            if(armTarget != 0) {
                if (armTarget == 1) {ARM_TARGET_POSITION = ARM_RAISED_POSITION;}
                if (armTarget == -1) {ARM_TARGET_POSITION = ARM_DEFAULT_POSITION;}
//                ArmRight.setTargetPosition(ARM_TARGET_POSITION);
//                ArmLeft.setTargetPosition(ARM_TARGET_POSITION);
                telemetry.addData("arm target", ARM_TARGET_POSITION);
                telemetry.addData("arm curr pos", ArmRight.getCurrentPosition());
                armPower = armPID.update(ARM_TARGET_POSITION, ArmRight.getCurrentPosition());
                ArmRight.setPower(armPower);
                ArmLeft.setPower(armPower);
//
//                ArmRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                ArmLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//                ArmRight.setVelocity(200);
//                ArmLeft.setVelocity(200);
//                if (armTarget == 1 && ArmRight.getCurrentPosition() >= ARM_RAISED_POSITION) {
//                    armTarget = 0;
//                }
//                if (armTarget == -1 && ArmRight.getCurrentPosition() <= ARM_DEFAULT_POSITION) {
//                    armTarget = 0;
//                }
            } else {
                ArmRight.setVelocity(0);
                ArmRight.setPower(0);
                ArmLeft.setVelocity(0);
                ArmLeft.setPower(0);
            }

            /*Claw Control*/
            if (gamepad1.left_trigger == 1.0) {
                RightFinger.setPosition(Rdefaultpos - 0.4);
                LeftFinger.setPosition(Ldefaultpos + 0.4);
            }
            else if (gamepad1.left_bumper) {
                LeftFinger.setPosition(Ldefaultpos + 0.4);
            }
            else if (gamepad1.right_bumper) {
                RightFinger.setPosition(Rdefaultpos - 0.4);
            }
            else {
                RightFinger.setPosition(Rdefaultpos);
                LeftFinger.setPosition(Ldefaultpos);
            }

            /*Wrist Rotation*/
            if (gamepad1.dpad_right) {
                axisY.setPosition(0.7);
            }
            if (gamepad1.dpad_left) {
                axisY.setPosition(0.0);
            }

            if (gamepad1.dpad_up) {
                axisXPosition += 0.001;
                axisX.setPosition(axisXPosition);
            }
            else if (gamepad1.dpad_down) {
                axisXPosition -= 0.001;
                axisX.setPosition(axisXPosition);
            }
            else if (gamepad1.b) {
                axisXPosition = defaultAxis;
                axisX.setPosition(axisXPosition);
            }

            /*Fly Wheel Control*/
            FlyWheel.setPower(0);
            if (gamepad1.a) {
                FlyWheel.setPower(1);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Servo Pos", axisX.getPosition());
            telemetry.addData("Arm Pos", ArmRight.getCurrentPosition());
            telemetry.addData("Arm Pow", ArmRight.getPower());
            telemetry.addData("Arm Pow2", armPower);
            telemetry.addData("p", armPID.getP());
            telemetry.addData("i", armPID.getI());
            telemetry.addData("d", armPID.getD());

            telemetry.addLine();
            telemetry.addData("Controls", null);
            telemetry.addData("Drive/Strafe/Rotation", "Joy Stick/Axis - Left/Y, Left/X, Right/X");
            telemetry.addData("Precision Drive", "HOLD Right Trigger");
            telemetry.addData("Raise Arm", "PRESS X Button");
            telemetry.addData("Lower Arm", "PRESS Y Button");
            telemetry.addData("Open Both Finger", "HOLD Left Trigger");
            telemetry.addData("Open Small Claw", "HOLD Left Bumper");
            telemetry.addData("Open Large Claw", "HOLD Right Bumper");
            telemetry.addData("Wrist Rotation Y-axis", "PRESS Dpad Right (180) & Left (OG)");
            telemetry.addData("Wrist Rotation X-axis", "PRESS Dpad Up & Down");
            telemetry.update();
        }
    }
}