package frc.robot.subsystems;

import org.opencv.core.Mat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Supposed to be a subsystem to work with the camera on the robot

public class Vision extends SubsystemBase {
    Thread visionThread;
    
    public Vision() {
         visionThread =
            new Thread(
                () -> {
                // Get the UsbCamera from CameraServer
                UsbCamera camera = CameraServer.startAutomaticCapture();
                // Set the resolution
                camera.setResolution(640, 460);
                System.out.println(camera.getBrightness());

                // Get a CvSink. This will capture Mats from the camera
                CvSink cvSink = CameraServer.getVideo();
                // Setup a CvSource. This will send images back to the Dashboard
                CvSource outputStream = CameraServer.putVideo("Rectangle", 1080, 720);

                // Mats are very memory expensive. Lets reuse this Mat.
                Mat mat = new Mat();

                // This cannot be 'true'. The program will never exit if it is. This
                // lets the robot stop this thread when restarting robot code or
                // deploying.
                while (!Thread.interrupted()) {
                    // Tell the CvSink to grab a frame from the camera and put it
                    // in the source mat.  If there is an error notify the output.
                    if (cvSink.grabFrame(mat) == 0) {
                    // Send the output the error.
                    outputStream.notifyError(cvSink.getError());
                    // skip the rest of the current iteration
                    continue;
                    }
                    // Put a rectangle on the image
                    // Imgproc.rectangle(
                        // mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                    // Give the output stream a new image to display
                    outputStream.putFrame(mat);
                }
            });
    }

    public void startVision() {
        visionThread.setDaemon(true);
        visionThread.start();
    }

    public Thread getVisionThread() {
        return visionThread;
    }
}
