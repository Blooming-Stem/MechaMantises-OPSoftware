package org.firstinspires.ftc.teamcode.computervision.pipelines;

import static java.lang.Math.atan;
import static java.lang.Math.tan;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.List;
import java.util.Collections;
import java.util.Comparator;


//for dashboard
/*@Config*/
public class ConeObserverPipeline extends OpenCvPipeline {

    public double pixel_height = 0.0;
    public double coneCenterX = 0.0;

    public double coneCenterY = 0.0;

    public double distance = 0.0;




    //backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 110;
    public static double strictHighS = 255;

    public static double camera_fov = 55;

    public ConeObserverPipeline() {
        frameList = new ArrayList<>();
        distance = getDistanceFromCone();

    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
        Scalar lowHSV = new Scalar(110,50, 50); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(130, 255, 255); // lenient higher bound HSV for yellow
        Scalar color = new Scalar(0,256, 0);
        Mat mask = new Mat();
        Core.inRange(mat, lowHSV, highHSV, mask);
        Mat kernel = new Mat();

        Mat opening = new Mat();

        Imgproc.morphologyEx(mask, opening, Imgproc.MORPH_CLOSE, kernel);

        Mat closing = new Mat();

        Mat finalMat = new Mat();


        Imgproc.morphologyEx(opening, closing, Imgproc.MORPH_OPEN, kernel);




        //contours, apply post processing to information
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(closing, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        if(contours.size()>0){

            contours.sort(Collections.reverseOrder(Comparator.comparingDouble(a -> Imgproc.boundingRect(a).area())));

            //get biggest contour
            MatOfPoint biggest = contours.get(0);
            Rect rectangle = Imgproc.boundingRect(biggest);


            Moments p = Imgproc.moments(biggest);
            int x = (int) (p.get_m10() / p.get_m00());
            int y = (int) (p.get_m01() / p.get_m00());







            Imgproc.circle(input, new Point(x,y), 2, color, -1);

            Imgproc.rectangle(input, rectangle, color, 2);

            pixel_height = rectangle.height;
            coneCenterX = x;
            coneCenterY = y;


        }


        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 3) {
            frameList.remove(0);
        }



        //release all the data


        finalMat.release();


        mat.release();
        mask.release();
        opening.release();
        closing.release();





        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)
        return input;
    }
    public double getDistanceFromCone(){
        double distance = (5*796.8)/pixel_height;
        return distance;

    }
    public double getlateralDistance(){
        double x = getDistanceFromCone()*tan(Math.toRadians(camera_fov/2));
        double y = (coneCenterX*x)/320;
        double lateralDistance = y-x;

        return lateralDistance;



    }
    public double turnAngle(){
        double x = getDistanceFromCone();

        double y = getlateralDistance();

        double angle = atan(y/x);

        return Math.toDegrees(angle);
    }




}