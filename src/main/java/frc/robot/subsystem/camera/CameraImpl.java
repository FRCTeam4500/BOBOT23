package frc.robot.subsystem.camera;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.VideoMode;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.wpilibj.Timer;

import java.util.HashSet;



/** This class holds the camera on the intake of the 2022 robot
 * The camera server can be accessed on shuffleboard, does not specifically need to be sent
 */
public class CameraImpl {

    private int id;

    public CameraImpl(int id) {
        this.id = id;
    }

    /** creates a simple or complex stream based on type defined in constructor */
    public void start() {
        createSimpleStream();
        createAprilTagStream();
    }

    // starts a camera server automaically without changing any default values
    public void createSimpleStream() {
        UsbCamera camera = CameraServer.startAutomaticCapture(id);
        camera.setResolution(CameraConstants.width, CameraConstants.height);
        camera.setFPS(CameraConstants.fps);

        
    }

    public void createAprilTagStream() {
        new Thread(
            () -> {
                CvSink cvSink = CameraServer.getVideo();
                CvSource outputStream = CameraServer.putVideo("AprilTags", CameraConstants.width, CameraConstants.height);

                Mat mat = new Mat();
                Mat grayMat = new Mat();

                var pt0 = new Point(); // Point on apriltag, RENAME later based on which these are
                var pt1 = new Point();
                var pt2 = new Point();
                var pt3 = new Point();
                var center = new Point(); // Thankfully we do not have to calculate this
                var red = new Scalar(0, 0, 255); // Color
                var green = new Scalar(0, 255, 0); // Color

                var aprilTagDetector = new AprilTagDetector();

                var config = aprilTagDetector.getConfig();
                config.quadSigma = 0.8f; // Pretty much decimated for us, makes runtime a teeny tiny bit faster (we need it)
                aprilTagDetector.setConfig(config);

                var quadThreshParams = aprilTagDetector.getQuadThresholdParameters();
                quadThreshParams.minClusterPixels = 250;
                quadThreshParams.criticalAngle *= 5; // default is 10
                quadThreshParams.maxLineFitMSE *= 1.5;
                aprilTagDetector.setQuadThresholdParameters(quadThreshParams);

                aprilTagDetector.addFamily("tag16h5"); // Specific to field tags

                var timer = new Timer();
                timer.start();
                var count = 0; // For debugging purposes, need to check if image needs to be decimated more.

                while (!Thread.interrupted()) {
                    if (cvSink.grabFrame(mat) == 0) {
                        outputStream.notifyError(cvSink.getError());
                        continue;
                    }

                    Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

                    var results = aprilTagDetector.detect(grayMat);

                    var set = new HashSet<>();

                    for (var result: results) {
                        count += 1;
                        pt0.x = result.getCornerX(0); // Corner Index X
                        pt1.x = result.getCornerX(1);
                        pt2.x = result.getCornerX(2);
                        pt3.x = result.getCornerX(3);

                        pt0.y = result.getCornerY(0); // Corner Index Y
                        pt1.y = result.getCornerY(1);
                        pt2.y = result.getCornerY(2);
                        pt3.y = result.getCornerY(3);

                        center.x = result.getCenterX(); // Lucky team 4500 having wrappers around everything lmao
                        center.y = result.getCenterY();

                        set.add(result.getId()); // ID match done for us

                        /* Draw Box around the tag, you feel me? */
                        Imgproc.line(mat, pt0, pt1, red, 5);
                        Imgproc.line(mat, pt1, pt2, red, 5);
                        Imgproc.line(mat, pt2, pt3, red, 5);
                        Imgproc.line(mat, pt3, pt0, red, 5);

                        Imgproc.circle(mat, center, 4, green); // Circle in the center of the tag
                        Imgproc.putText(mat, String.valueOf(result.getId()), pt2, Imgproc.FONT_HERSHEY_SIMPLEX, 2, green, 7); // ID of the tag

                    };

                    for (var id : set){
                        System.out.println("Tag: " + String.valueOf(id)); // Debug First, Program Later
                    }

                    if (timer.advanceIfElapsed(1.0)){
                        System.out.println("detections per second: " + String.valueOf(count)); // Where count is used :)
                        count = 0;
                    }

                    outputStream.putFrame(mat);
                }
                aprilTagDetector.close();
            });
    }
}