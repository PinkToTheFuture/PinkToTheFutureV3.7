package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MatrixConstants;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Main Drive Project", group="PinktotheFuture")
public class AAMainDriveProject extends LinearOpMode {
    bno055driver imu;

    @Override
    public void runOpMode() throws InterruptedException {
        double LFpower = 0;
        double LBpower = 0;
        double RFpower = 0;
        double RBpower = 0;
        double speed = 1;
        double relicpos = 0.6;
        double relicanglepos = 0;
        double timeforbak1 = 0;
        double timeforbak2 = 0;
        double jewelextendertimeheen = 0;
        double jewelextendertimeterug = 0;
        double jewelextenderpos = 0;
        boolean jewelextenderbuttonheen = false;
        boolean jewelextenderbuttonterug = false;
        boolean bakjebovenpos = false;
        boolean bakjedichtpos = false;
        boolean timeforbakbool1 = false;
        boolean timeforbakbool2 = false;
        boolean endbottombakbool;
        boolean endtopbakbool;


        DigitalChannel endbottombak = hardwareMap.get(DigitalChannel.class, "endbottombak");
        DigitalChannel endtopbak = hardwareMap.get(DigitalChannel.class, "endtopbak");
        endbottombak.setMode(DigitalChannel.Mode.INPUT);
        endtopbak.setMode(DigitalChannel.Mode.INPUT);

        Servo moverelic = hardwareMap.servo.get("moverelic");
        Servo grabrelic = hardwareMap.servo.get("grabrelic");
        Servo bakjeturn = hardwareMap.servo.get("bakjeturn");
        Servo bakjedicht = hardwareMap.servo.get("bakjedicht");
        Servo anglefishingrod = hardwareMap.servo.get("anglefishingrod");
        Servo filamentmanipulator = hardwareMap.servo.get("filamentmanipulator");
        filamentmanipulator.setPosition(1);
        Servo jewelextender = hardwareMap.servo.get("jewelextender");
        Servo jewelchooser = hardwareMap.servo.get("jewelchooser");

        DcMotor intakeR = hardwareMap.dcMotor.get("intaker");
        DcMotor intakeL = hardwareMap.dcMotor.get("intakel");

        DcMotor relic = hardwareMap.dcMotor.get("relic");
        relic.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        relic.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relic.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor bak = hardwareMap.dcMotor.get("bak");

        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");

        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        RFdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new bno055driver("IMU", hardwareMap);

        Double[] imuArray;
        imuArray = new Double[1];
        imuArray[0] = 0.0;

        waitForStart();


        while (opModeIsActive()) {
            filamentmanipulator.setPosition(0.5 );
            if (gamepad1.dpad_up)     speed = 1;
            if (gamepad1.dpad_down)   speed = 0.3;
            if (gamepad1.dpad_left)   speed = 0.6;

            double temp;

            double theta = imu.getAngles()[0];

            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rcw = -gamepad1.right_stick_x;

            if (Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.right_stick_x) > 0 || Math.abs(gamepad1.right_stick_y) > 0 ){
                imuArray[0] = theta;
            }

            if (theta > 0) {
                temp = forward*Math.cos(theta)-strafe*Math.sin(theta);
                strafe = forward*Math.sin(theta)+strafe*Math.cos(theta);
                forward = temp;
            }

            if (theta <= 0) {
                temp = forward*Math.cos(theta)-strafe*Math.sin(theta);
                strafe = forward*Math.sin(theta)+strafe*Math.cos(theta);
                forward = temp;
            }

            LFpower = forward+rcw+strafe;
            RFpower = forward-rcw-strafe;
            LBpower = forward+rcw-strafe;
            RBpower = forward-rcw+strafe;

            Range.clip(RFpower, -1, 1);
            Range.clip(RBpower, -1, 1);
            Range.clip(LFpower, -1, 1);
            Range.clip(LBpower, -1, 1);

            LFdrive.setPower(-LFpower * speed);
            RBdrive.setPower(-RBpower * speed);
            LBdrive.setPower(-LBpower * speed);
            RFdrive.setPower(-RFpower * speed);

            if (gamepad1.left_bumper) {
                double Kp1 = 2;
                double Ki1 = 2;
                double Kd1 = 2;

                double Kp2 = 4;
                double Ki2 = 4;
                double Kd2 = 4;

                double error1;
                double error2;

                double Sp = 0;
                double Pv1 = imu.getAngles()[1];
                double Pv2 = imu.getAngles()[2];

                double integral1 = 0;
                double integral2 = 0;
                double derivative1;
                double derivative2;

                error1 = Sp - Pv1;
                error2 = Sp - Pv2;

                integral1 = error1 + integral1;
                integral2 = error2 + integral2;

                double lasterror1 = error1;
                double lasterror2 = error2;

                derivative1 = error1 - lasterror1;
                derivative2 = error2 - lasterror2;

                double correction1; //value for the motors from the pitch value
                double correction2; //value for the motors from the roll value

                correction1 = Kp1*error1 + Ki1*integral1 + Kd1*derivative1;
                correction2 = Kp2*error2 + Ki2*integral2 + Kd2*derivative2;

                //End of PID controller

                RFpower = ((((correction1) + (-correction2)) / 2));
                RBpower = ((((correction1) - (-correction2)) / 2));
                LFpower = ((((correction1) - (-correction2)) / 2));
                LBpower = ((((correction1) + (-correction2)) / 2));

                Range.clip(RFpower, -1, 1);
                Range.clip(RBpower, -1, 1);
                Range.clip(LFpower, -1, 1);
                Range.clip(LBpower, -1, 1);

                LFdrive.setPower(LFpower);
                RFdrive.setPower(RFpower);
                LBdrive.setPower(LBpower);
                RBdrive.setPower(RBpower);
            }

            if (gamepad1.y && gamepad1.right_bumper && !jewelextenderbuttonterug) {
                jewelextendertimeterug = getRuntime() + 3.5;
                jewelextenderbuttonterug = true;
            }
            if (gamepad1.x && gamepad1.right_bumper && !jewelextenderbuttonheen) {
                jewelextendertimeheen = getRuntime() + 3.5;
                jewelextenderbuttonheen = true;
            }
            if (jewelextendertimeterug >= getRuntime()){
                jewelextenderpos = jewelextenderpos - 0.03;
            } else { jewelextenderbuttonterug = false; }

            if (jewelextendertimeheen >= getRuntime()){
                jewelextenderpos = jewelextenderpos + 0.03;
            } else { jewelextenderbuttonheen = false; }
            if (jewelextenderpos < 0.1) jewelextenderpos = 0.1;
            if (jewelextenderpos > 0.7) jewelextenderpos = 0.7;
            jewelextender.setPosition(jewelextenderpos);


            if (gamepad1.a && gamepad1.right_bumper && !gamepad1.start) {
                jewelchooser.setPosition(0.1);
            } else {
                if (gamepad1.b && gamepad1.right_bumper && !gamepad1.start){
                    jewelchooser.setPosition(0.7);
                } else {
                    jewelchooser.setPosition(0.45);
                }

            }
            if (gamepad2.right_bumper && !(gamepad2.right_trigger>0.1))  grabrelic.setPosition(0.13);
            if (gamepad2.left_bumper)   grabrelic.setPosition(0.73);


            if (gamepad2.dpad_up)  relicpos = 0.9;
            if (gamepad2.dpad_down)  relicpos =.3;
            if (gamepad2.dpad_right) relicpos = 0.6;
            moverelic.setPosition(relicpos);



            if (gamepad2.b && !gamepad2.start && !timeforbakbool1 && !timeforbakbool2) {
                timeforbak1 = getRuntime() + 0.2;
                timeforbak2 = getRuntime() + 0.7;
                bakjedicht.setPosition(0.25);
                bakjedichtpos = true;
                timeforbakbool1 = true;
                timeforbakbool2 = true;

            }
            telemetry.addData("boven:", bakjeturn.getPosition());
            telemetry.addData("boven:", bakjebovenpos);
            telemetry.addData("dicht:", bakjedicht.getPosition());
            telemetry.addData("dicht:", bakjedichtpos);

            if (timeforbak1<getRuntime() && timeforbakbool1){
                timeforbakbool1 = false;
                if (!bakjebovenpos) {
                    bakjeturn.setPosition(0.2);
                    bakjebovenpos = true;
                } else {
                    bakjeturn.setPosition(0.7);
                    bakjebovenpos = false;
                }
                timeforbakbool2 = true;
            }
            if (timeforbak2<getRuntime() && timeforbakbool2){
                timeforbakbool2 = false;
                if (!bakjebovenpos){
                    bakjedicht.setPosition(0.1);
                    bakjedichtpos = false;
                }
            }

            if (gamepad1.a && !gamepad1.start) {
                bakjedicht.setPosition(0.1);
                bakjedichtpos = false;
            }
            if (gamepad2.a && !gamepad2.start) {
                bakjedicht.setPosition(0.1);
                bakjedichtpos = false;
            }

            relicanglepos = relicanglepos + gamepad2.right_stick_x/3;
            if (relicanglepos > 0.5){
                relicanglepos = 0.5;
            }
            if (relicanglepos < 0){
                relicanglepos = 0;
            }
            anglefishingrod.setPosition(relicanglepos);

            if ((gamepad2.right_trigger > 0.1) && gamepad2.right_bumper){
                intakeL.setDirection(DcMotorSimple.Direction.FORWARD);
                intakeR.setDirection(DcMotorSimple.Direction.REVERSE);
            } else {
                intakeL.setDirection(DcMotorSimple.Direction.FORWARD);
                intakeR.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            if (gamepad2.right_trigger > 0.1) {
                intakeL.setPower(-gamepad2.right_trigger);
                intakeR.setPower(gamepad2.right_trigger);

            } else {
                if (gamepad2.left_trigger > 0.1) {
                    intakeL.setPower(gamepad2.left_trigger);
                    intakeR.setPower(-gamepad2.left_trigger);
                } else {
                    intakeR.setPower(0);
                    intakeL.setPower(0);
                }
            }



            endbottombakbool = endbottombak.getState();
            endtopbakbool = endtopbak.getState();

            if (endbottombakbool && endtopbakbool){
                bak.setPower(-gamepad2.left_stick_y);
            } else {
                if (!endbottombakbool && endtopbakbool) bak.setPower(0.4);
                if (endbottombakbool && !endtopbakbool) bak.setPower(-0.4);
                if (!endbottombakbool && !endtopbakbool) bak.setPower(0);
            }
            relic.setPower(gamepad2.right_stick_y);

            telemetry.addData("endstopbottom", endbottombak.getState());
            telemetry.addData("endstoptop", endtopbak.getState());
            telemetry.addData("motor", bak.getPower());
            telemetry.addData("LB",LBpower);
            telemetry.addData("LF",LFpower);
            telemetry.addData("RB",RBpower);
            telemetry.addData("RF",RFpower);
            telemetry.addData("relic extruder", relic.getCurrentPosition());
            telemetry.update();


        }
    }
}
