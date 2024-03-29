package org.firstinspires.ftc.teamcode.OpenCV;

import static org.firstinspires.ftc.teamcode.hardware.Globals.ALLIANCE;
import static org.firstinspires.ftc.teamcode.hardware.Globals.SIDE;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.OpenCV.Location;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class PropPipelinev3 implements VisionProcessor {
    private static final boolean DEBUG = false;

    private volatile Location location = Location.NONE;

    private final Mat hsv = new Mat();

    public static int redLeftX = (int) (815 / 1.5);
    public static int redLeftY = (int) (550 / 1.5);

    public static int redCenterX = (int) (1365 / 1.5);
    public static int redCenterY = (int) (475 / 1.5);
    public static int redRightX = (int) (1365 / 1.5);
    public static int redRightY = (int) (475 / 1.5);

    public static int blueLeftX = (int) (240 / 1.5);
    public static int blueLeftY = (int) (525 / 1.5);

    public static int blueCenterX = (int) (925 / 1.5);
    public static int blueCenterY = (int) (485 / 1.5);
    public static int blueRightX = (int) (925 / 1.5);
    public static int blueRightY = (int) (485 / 1.5);

    public static int leftWidth = (int) (175 / 1.5);
    public static int leftHeight = (int) (100 / 1.5);

    public static int centerWidth = (int) (125 / 1.5);
    public static int centerHeight = (int) (125 / 1.5);
    public static int rightWidth = (int) (125 / 1.5);
    public static int rightHeight = (int) (125 / 1.5);


    public static double BLUE_TRESHOLD = 70;
    public static double RED_TRESHOLD = 100;

    public double leftColor = 0.0;
    public double centerColor = 0.0;
    public double rightColor = 0.0;

    public Scalar left = new Scalar(0, 0, 0);
    public Scalar center = new Scalar(0, 0, 0);
    public Scalar right = new Scalar(0, 0, 0);

    private Telemetry telemetry;
    private Mat frame;

    public PropPipelinev3() {
        this(null);
    }

    public PropPipelinev3(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        this.frame = frame.clone(); // Clone the frame for drawing on it

        Rect leftZoneArea;
        Rect centerZoneArea;
        Rect rightZoneArea;

        if (ALLIANCE == Location.RED && SIDE == Location.FAR || ALLIANCE == Location.BLUE && SIDE == Location.CLOSE) {
            leftZoneArea = new Rect(redLeftX, redLeftY, leftWidth, leftHeight);
            centerZoneArea = new Rect(redCenterX, redCenterY, centerWidth, centerHeight);
            rightZoneArea = new Rect(redRightX, redRightY, rightWidth, rightHeight);
        } else {
            leftZoneArea = new Rect(blueLeftX, blueLeftY, leftWidth, leftHeight);
            centerZoneArea = new Rect(blueCenterX, blueCenterY, centerWidth, centerHeight);
            rightZoneArea = new Rect(blueRightX, blueRightY, rightWidth, rightHeight);
        }

        Mat leftZone = frame.submat(leftZoneArea);
        Mat centerZone = frame.submat(centerZoneArea);
        Mat rightZone = frame.submat(rightZoneArea);

        Imgproc.blur(leftZone, leftZone, new Size(5, 5));
        Imgproc.blur(centerZone, centerZone, new Size(5, 5));
        Imgproc.blur(rightZone, rightZone, new Size(5, 5));

        left = Core.mean(leftZone);
        center = Core.mean(centerZone);
        right = Core.mean(rightZone);

        double threshold = ALLIANCE == Location.RED ? RED_TRESHOLD : BLUE_TRESHOLD;
        int idx = ALLIANCE == Location.RED ? 0 : 2;

        leftColor = left.val[idx];
        centerColor = center.val[idx];
        rightColor = right.val[idx];

        if (leftColor > threshold && (left.val[0] + left.val[1] + left.val[2] - left.val[idx] < left.val[idx])) {
            // left zone has it
            location = Location.LEFT;
            Imgproc.rectangle(this.frame, leftZoneArea, new Scalar(255, 255, 255), 10);
        } else if (centerColor > threshold && (center.val[0] + center.val[1] + center.val[2] - center.val[idx] < center.val[idx])) {
            // center zone has it
            location = Location.CENTER;
            Imgproc.rectangle(this.frame, centerZoneArea, new Scalar(255, 255, 255), 10);
        } else if (rightColor > threshold && (right.val[0] + right.val[1] + right.val[2] - right.val[idx] < right.val[idx])) {
            // center zone has it
            location = Location.CENTER;
            Imgproc.rectangle(this.frame, centerZoneArea, new Scalar(255, 255, 255), 10);

        } else  {
            // right zone has it
            location = Location.NONE;
        }

        leftZone.release();
        centerZone.release();
        rightZone.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public Location getLocation() {
        return this.location;
    }

    public Mat getFrame() {
        return frame;
    }
}
