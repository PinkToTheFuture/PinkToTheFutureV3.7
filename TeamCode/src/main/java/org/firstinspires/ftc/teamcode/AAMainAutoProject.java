package org.firstinspires.ftc.teamcode;

import android.net.UrlQuerySanitizer;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.disnodeteam.dogecv.detectors.GlyphDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.disnodeteam.dogecv.detectors.JewelDetector.JewelDetectionMode.PERFECT_AREA;


@Autonomous(name="Main Auto Project", group ="Concept")

public class AAMainAutoProject extends LinearOpMode {

    private JewelDetector jewelDetector = null;
    public Pictographs pictographs;
    public static final String TAG = "Vuforia VuMark Sample";
    ClosableVuforiaLocalizer vuforia;

    AutonomousVoids Autovoids = new AutonomousVoids();

    VuforiaTrackable relicTemplate;
    GlyphScoreCenter glyphScoreCenter = new GlyphScoreCenter();

    private CryptoboxDetector cryptoboxDetector = null;
    private GlyphDetector glyphDetector = null;

    public enum Pictographs {
        LEFT, CENTER, RIGHT
    }

    bno055driver imu;



    public void Vumark() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //delete cameraMonitorViewId to disable preview

        parameters.vuforiaLicenseKey = "AdQe/E//////AAAAGdUKVJKmNUyWi5sJUoS1G+d+ENxg28Ca11lGwDD6yFPE9hFgVC2x4O0CCFjPJzamT67NyeIzQYo4q0A3z4rJs6h76WVGT8Urwoi2AXXo/awgby8sTLQs8GXzvIg8WuS+7MvCiIKSvEwzv9FBsX8N8trXTsHsdfA7B3LB9C/rScSqDKulPKFTzbdgJvNRGJ8a6S1udF1q6FSZ5UPSFeEYsbQPpC7KBVuFbQAdtxikzobiBfkcHVWkPBJ77dvKkH8bi2tRPpWxqDDo0ZgQH5pTMI7NpKESokFWo8bNFbwvsVv9sK2QPDY8zd2l0Bo+ZOFypY4gdBpFhEiaX9TS/60Ee+LTL/5ExbkahObffUjnCb9X";

        parameters.cameraDirection = ClosableVuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = new ClosableVuforiaLocalizer(parameters);
        //change closable back (null object reference)


        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


        relicTrackables.activate();

        boolean loop = true;

        while (opModeIsActive()&& loop) {
            waitOneFullHardwareCycle();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                pictographs = Pictographs.LEFT;
                vuforia.close();
                loop = false;
            }
            if (vuMark == RelicRecoveryVuMark.CENTER) {
                pictographs = Pictographs.CENTER;
                vuforia.close();
                loop = false;
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                pictographs = Pictographs.RIGHT;
                vuforia.close();
                loop = false;
            }
            telemetry.update();
        }
    }

    public void Initialisation() throws InterruptedException {
        Servo Jewelservo = hardwareMap.servo.get("Jewelservo");
        Jewelservo.setPosition(1);
    }

    public void Jewels() throws InterruptedException {

        Servo jewelextender = hardwareMap.servo.get("jewelextender");
        Servo jewelchooser = hardwareMap.servo.get("jewelchooser");

        double jewelextenderpos = 0;
        double jewelextendertimedown = 0;
        double jewelextendertimeback = 0;
        boolean jewelextenderdown = false;
        boolean jewelextenderback = false;


        jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        //
        //Jewel Detector Settings
        jewelDetector.areaWeight = 0.05;
        jewelDetector.downScaleFactor = 1;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        //jewelDetector.perfectArea = 6500; //<- Needed for PERFECT_AREA
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 10;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;
        jewelDetector.rotateMat = false;
        jewelDetector.enable();


        /**
        CameraBridgeViewBase -> deliverAndDrawFrame (rotate view)
        CameraBridgeViewBase -> front/rear
        CameraGLRenderBase -> front/rear
        JavaCameraView -> front/rear
        */

        boolean loop = true;
        while (opModeIsActive()&&loop) {
            /*telemetry.addData("Current Order", "Jewel Order: " + jewelDetector.getCurrentOrder().toString()); // Current Result
            telemetry.addData("Last Order", "Jewel Order: " + jewelDetector.getLastOrder().toString()); // Last Known Result */
            waitOneFullHardwareCycle();
            JewelDetector.JewelOrder jewl = jewelDetector.getCurrentOrder();
            jewelextenderdown = true;

            if (jewelextenderdown == true) {
                jewelextendertimedown = getRuntime() + 3.5;
            } else jewelextenderdown = false;
            if (jewelextenderback == true) {
                jewelextendertimeback = getRuntime() + 3.5;
            } else jewelextenderback = false;

           if (jewelextendertimedown>= getRuntime()) {
               jewelextenderpos = jewelextenderpos - 0.03;
           }
           if (jewelextendertimeback>= getRuntime()) {
               jewelextenderpos = jewelextenderpos + .03;
           }
           if (jewelextenderpos < 0.1) jewelextenderpos = 0.1;
           if (jewelextenderpos > 0.7) jewelextenderpos = 0.7;
           jewelextender.setPosition(jewelextenderpos);


           if (jewl == JewelDetector.JewelOrder.RED_BLUE){
                jewelchooser.setPosition(0.1);
                sleep(500);
                telemetry.addData("DETECTED: ", jewelDetector.getCurrentOrder());
                telemetry.addData("jewelextender pos", jewelextender.getPosition());
                telemetry.update();
                jewelchooser.setPosition(0.45);
                sleep(500);
                loop = false;
                jewelDetector.disable();
                jewelextenderback = true;

            }
            if (jewl == JewelDetector.JewelOrder.BLUE_RED){
                jewelchooser.setPosition(0.7);
                sleep(500);
                telemetry.addData("DETECTED: ", jewelDetector.getCurrentOrder());
                telemetry.addData("jewelextender pos", jewelextender.getPosition());
                telemetry.update();
                jewelchooser.setPosition(0.45);
                sleep(500);
                loop = false;
                jewelDetector.disable();
                jewelextenderback = true;

            }
        }

    }

    public void Cryptobox() throws InterruptedException {

        boolean loop = true;
        while (opModeIsActive()&& loop) {

                if (pictographs == Pictographs.LEFT) {
                    //StrafeRight(x, .3);
                    StrafeRight(1, .3);
                }
                if (pictographs == Pictographs.CENTER) {
                    //StrafeRight(x, .3);
                    StrafeRight(2, .3);
                }
                if (pictographs == Pictographs.RIGHT) {
                    //StrafeRight(x, .3);
                    StrafeRight(3, .3);
                }
        }
    }

    public void Teleop() throws InterruptedException{
        double LFpower = 0;
        double LBpower = 0;
        double RFpower = 0;
        double RBpower = 0;
        double speed = 1;
        double relicpos = 0.6;
        double relicanglepos = 0.5;
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
            if (jewelextenderpos > 0.65) jewelextenderpos = 0.65;
            jewelextender.setPosition(jewelextenderpos);


            if (gamepad1.a && gamepad1.right_bumper && !gamepad1.start) {
                jewelchooser.setPosition(0.3);
            } else {
                if (gamepad1.b && gamepad1.right_bumper && !gamepad1.start){
                    jewelchooser.setPosition(0.7);
                } else {
                    jewelchooser.setPosition(0.45);
                }

            }
            if (gamepad2.right_bumper && !(gamepad2.right_trigger>0.1))  grabrelic.setPosition(0.13);
            if (gamepad2.left_bumper)   grabrelic.setPosition(0.7);


            if (gamepad2.dpad_up)  relicpos += 0.01;
            if (gamepad2.dpad_down)  relicpos -= 0.01;
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

            relicanglepos = relicanglepos + gamepad2.left_stick_x/30;
            if (relicanglepos > 0.8){
                relicanglepos = 0.8;
            }
            if (relicanglepos < 0.2){
                relicanglepos = 0.2;
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
                bak.setPower(-gamepad2.right_stick_y);
            } else {
                if (!endbottombakbool && endtopbakbool) bak.setPower(0.4);
                if (endbottombakbool && !endtopbakbool) bak.setPower(-0.4);
                if (!endbottombakbool && !endtopbakbool) bak.setPower(0);
            }
            relic.setPower(gamepad2.left_stick_y);

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

    public void TurnLeft(double rot, double pwr) throws InterruptedException {
        boolean loop = true;
        int encv = (int) Math.round(rot*53.75);
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (loop && opModeIsActive()){

            LFdrive.setPower(pwr);
            LBdrive.setPower(pwr);
            RFdrive.setPower(pwr);
            RBdrive.setPower(pwr);

            LFdrive.setTargetPosition(encv);     //537.6
            LBdrive.setTargetPosition(encv);
            RFdrive.setTargetPosition(encv);
            RBdrive.setTargetPosition(encv);
            //sleep(5000);

            waitOneFullHardwareCycle();
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (LFdrive.getCurrentPosition() > (encv - 10) && LBdrive.getCurrentPosition() > (encv - 10) && RFdrive.getCurrentPosition() > (encv - 40) && RBdrive.getCurrentPosition() > (encv - 10)) {
                loop = false;
            }

        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }

    public void TurnRight(double rot, double pwr) throws InterruptedException {
        boolean loop = true;
        int encv = (int) ((int) rot*53.75);
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        LBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (loop && opModeIsActive()){

            LFdrive.setPower(pwr);
            LBdrive.setPower(pwr);
            RFdrive.setPower(pwr);
            RBdrive.setPower(pwr);

            LFdrive.setTargetPosition(encv);     //537.6
            LBdrive.setTargetPosition(encv);
            RFdrive.setTargetPosition(encv);
            RBdrive.setTargetPosition(encv);
            //sleep(5000);

            waitOneFullHardwareCycle();
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();


            if (LFdrive.getCurrentPosition() > (encv - 10) && LBdrive.getCurrentPosition() > (encv - 10) && RFdrive.getCurrentPosition() > (encv - 40) && RBdrive.getCurrentPosition() > (encv - 10)) {
                loop = false;
            }

        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }

    public void StrafeRight(double rot, double pwr) throws InterruptedException {
        boolean loop = true;
        int encv = (int) ((int) rot*53.75);
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (loop && opModeIsActive()){

            LFdrive.setPower(pwr);
            LBdrive.setPower(pwr);
            RFdrive.setPower(pwr);
            RBdrive.setPower(pwr);

            LFdrive.setTargetPosition(encv);     //537.6
            LBdrive.setTargetPosition(encv);
            RFdrive.setTargetPosition(encv);
            RBdrive.setTargetPosition(encv);
            //sleep(5000);


            waitOneFullHardwareCycle();
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();



            if (LFdrive.getCurrentPosition() > (encv - 10) && LBdrive.getCurrentPosition() > (encv - 10) && RFdrive.getCurrentPosition() > (encv - 40) && RBdrive.getCurrentPosition() > (encv - 10)) {
                loop = false;
            }

        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }

    public void StrafeLeft (double rot, double pwr) throws InterruptedException {
        boolean loop = true;
        int encv = (int) ((int) rot*53.75);
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (loop && opModeIsActive()){

            LFdrive.setPower(pwr);
            LBdrive.setPower(pwr);
            RFdrive.setPower(pwr);
            RBdrive.setPower(pwr);

            LFdrive.setTargetPosition(encv);     //537.6
            LBdrive.setTargetPosition(encv);
            RFdrive.setTargetPosition(encv);
            RBdrive.setTargetPosition(encv);

            waitOneFullHardwareCycle();
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (LFdrive.getCurrentPosition() > (encv - 10) && LBdrive.getCurrentPosition() > (encv - 10) && RFdrive.getCurrentPosition() > (encv - 40) && RBdrive.getCurrentPosition() > (encv - 10)) {
                loop = false;
            }

        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }

    public void Forward(double rot, double pwr) throws InterruptedException {
        boolean loop = true;
        int encv = ((int) Math.round(rot*53.75));
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (loop && opModeIsActive()){

            LFdrive.setPower(pwr);
            LBdrive.setPower(pwr);
            RFdrive.setPower(pwr);
            RBdrive.setPower(pwr);

            LFdrive.setTargetPosition(encv);     //537.6
            LBdrive.setTargetPosition(encv);
            RFdrive.setTargetPosition(encv);
            RBdrive.setTargetPosition(encv);
            //sleep(5000);

            waitOneFullHardwareCycle();
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (LFdrive.getCurrentPosition() > (encv - 10) && LBdrive.getCurrentPosition() > (encv - 10) && RFdrive.getCurrentPosition() > (encv - 40) && RBdrive.getCurrentPosition() > (encv - 10)) {
                loop = false;
            }

        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }

    public void Reverse (double rot, double pwr) throws InterruptedException {
        boolean loop = true;
        int encv = (int) ((int) rot*53.75);
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        LBdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RBdrive.setDirection(DcMotorSimple.Direction.REVERSE);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (loop && opModeIsActive()){

            LFdrive.setPower(pwr);
            LBdrive.setPower(pwr);
            RFdrive.setPower(pwr);
            RBdrive.setPower(pwr);

            LFdrive.setTargetPosition(encv);     //537.6
            LBdrive.setTargetPosition(encv);
            RFdrive.setTargetPosition(encv);
            RBdrive.setTargetPosition(encv);
            //sleep(5000);

            waitOneFullHardwareCycle();
            telemetry.addData("LFdrive", LFdrive.getCurrentPosition());
            telemetry.addData("LBdrive", LBdrive.getCurrentPosition());
            telemetry.addData("RFdrive", RFdrive.getCurrentPosition());
            telemetry.addData("RBdrive", RBdrive.getCurrentPosition());
            telemetry.update();

            if (LFdrive.getCurrentPosition() > (encv - 10) && LBdrive.getCurrentPosition() > (encv - 10) && RFdrive.getCurrentPosition() > (encv - 40) && RBdrive.getCurrentPosition() > (encv - 10)) {
                loop = false;
            }

        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }

    public void Shake(double rot ) throws InterruptedException {
        boolean loop = true;
        int encv = ((int) Math.round(rot*53.75));
        DcMotor LFdrive = hardwareMap.dcMotor.get("LFdrive");
        DcMotor RBdrive = hardwareMap.dcMotor.get("RBdrive");
        DcMotor LBdrive = hardwareMap.dcMotor.get("LBdrive");
        DcMotor RFdrive = hardwareMap.dcMotor.get("RFdrive");
        LFdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LBdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        RFdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        RBdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        LFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        LFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (loop && opModeIsActive()){



        }
        LFdrive.setPower(0);
        LBdrive.setPower(0);
        RFdrive.setPower(0);
        RBdrive.setPower(0);
    }



    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * Change voids depending on alliance and starting position:
         * */

        //Initialisation();
        waitForStart();

        boolean bakjebovenpos = false;
        boolean bakjedichtpos = false;
        boolean timeforbakbool1 = false;
        boolean timeforbakbool2 = false;
        double timeforbak1 = 0;
        double timeforbak2 = 0;

        Servo bakjedicht = hardwareMap.servo.get("bakjedicht");
        Servo bakjeturn = hardwareMap.servo.get("bakjeturn");
        DcMotor intakeR = hardwareMap.dcMotor.get("intaker");
        DcMotor intakeL = hardwareMap.dcMotor.get("intakel");

        intakeL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);


        //red upper:
        bakjedicht.setPosition(0.25);
        Jewels();
        Forward(22, .4);
        StrafeRight(21, 0.6);
        Reverse(7, .6);
        TurnLeft(10, .3);
        bakjeturn.setPosition(0.2);
        sleep(1000);
        bakjedicht.setPosition(0.1);
        sleep(1000);
        Reverse(5, .3);
        Forward(7, .3);
        //2nd run
        bakjedicht.setPosition(0.25);
        sleep(750);
        bakjeturn.setPosition(.7);
        sleep(750);
        bakjedicht.setPosition(.1);
        sleep(750);

        intakeL.setPower(.9);
        intakeR.setPower(.9);
        Forward(20, .6);
        Forward(20, .2);
        Reverse(40, .6);
        intakeL.setPower(0);
        intakeR.setPower(0);
        bakjedicht.setPosition(.25);
        sleep(1000);
        StrafeLeft(20, .5);
        TurnRight(3, .3);
        bakjeturn.setPosition(0.2);
        sleep(750);
        bakjedicht.setPosition(0.1);
        sleep(750);
        Reverse(6, .2);
        Forward(4, .2);
        bakjeturn.setPosition(.7);
        sleep(1000);



        /*blue upper:
        bakjedicht.setPosition(0.25);
        Forward(20, .3);
        StrafeLeft(20, 0.3);
        Reverse(15, .3);
        TurnRight(19, .3);
        bakjeturn.setPosition(0.2);
        sleep(1000);
        bakjedicht.setPosition(0.1);
        sleep(1000);
        Reverse(6, 0.2);
        Forward(3, .2);
        //2nd run
        StrafeLeft(20, .3);
        bakjeturn.setPosition(.7);
        sleep(1000);
        TurnLeft(4, 0.3);
        intakeL.setPower(.9);
        intakeR.setPower(.9);
        Forward(25, .2);
        Reverse(30, .2);
        intakeL.setPower(0);
        intakeR.setPower(0);
        bakjedicht.setPosition(.25);
        sleep(1000);
        StrafeRight(10, .3);
        bakjeturn.setPosition(0.2);
        sleep(1000);
        bakjedicht.setPosition(0.1);
        sleep(1000);
        Reverse(6, 0.2);
        Forward(4, .2);
        bakjeturn.setPosition(.7);
        */



        //0.7 bakje turn standaard
        //0.1 open bakje


    }
}