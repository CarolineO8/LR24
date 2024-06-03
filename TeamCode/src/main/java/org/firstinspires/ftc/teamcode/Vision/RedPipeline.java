package org.firstinspires.ftc.teamcode.Vision;

import static org.opencv.core.Core.inRange;
import static org.opencv.core.CvType.CV_8U;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.FONT_HERSHEY_COMPLEX;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.drawContours;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.findContours;
import static org.opencv.imgproc.Imgproc.rectangle;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class RedPipeline extends OpenCvPipeline {

    public static int max_H = 160;
    public static int max_S = 200;
    public static int max_V = 255;

    public static Rect largestRect;

    public static int min_H = 135;
    public static int min_S = 10;
    public static int min_V = 130;

    public static double offset = 0;
    public static int erodeConstant = 1;
    public static int dilateConstant = 1;
    //for camera offset, going to try and make it better

    public Rect rectangleOutline;

    public static boolean targetDetected = false;


    //stuff directly from Collin's vision code pipeline
    private static int IMG_HEIGHT = 0;
    private static int IMG_WIDTH = 0;
    //sets up variables to collect image details

    private Mat output = new Mat(),
            modified = new Mat();
    private ArrayList<MatOfPoint> contours = new ArrayList<>();

    private Mat hierarchy = new Mat();
    //stuff for variables


    // Rectangle settings, for output image
    private Scalar orange = new Scalar(252, 186, 3);
    private Scalar lightBlue = new Scalar(3, 252, 227);

    private Scalar green = new Scalar(4, 210, 227);
    //idk what this color actually looks like
    private int thickness = 10;
    private int font = FONT_HERSHEY_COMPLEX;


    @Override
    public Mat processFrame(Mat input) {//this thing will do the actual stuff
        input.copyTo(output);


        IMG_HEIGHT = input.rows();
        IMG_WIDTH = input.cols();
        //just saving info

        //Scalar MIN_THRESH_PROP = new Scalar(0, 35, 38);
        //  Scalar MAX_THRESH_PROP = new Scalar(15, 100, 63);

        Scalar MIN_THRESH_PROP = new Scalar(min_H, min_S, min_V);
        Scalar MAX_THRESH_PROP = new Scalar(max_H, max_S, max_V);
        //setting up all the color thresholds


        // Imgproc.cvtColor(input, modified, COLOR_RGB2HSV);
        Imgproc.cvtColor(input, modified, COLOR_BGR2HSV);

        //goes from RGB to HSV color space
        //replace blue w/ red so it can use both sides of red color


        inRange(modified, MIN_THRESH_PROP, MAX_THRESH_PROP, modified);


        Rect submatRect = new Rect(new Point(4, 4), new Point(IMG_WIDTH, IMG_HEIGHT));
        modified = modified.submat(submatRect);
        //actual threshold thing to correct for top of screen being wierd and glitchy

        erode(modified, modified, new Mat(erodeConstant, erodeConstant, CV_8U));
        //erode constant currently = 1, change to erode more or less
        dilate(modified, modified, new Mat(dilateConstant, dilateConstant, CV_8U));
        //dialate constant currently 1, should have erode/dialate be equal
        //this is the weird erode dilate thing that gets rid of stray pixels


        //contours.clear();
        contours = new ArrayList<>();

        findContours(modified, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        //figures out all the pixels on the edges of the blob, useful for finding center


        List<Rect> rects = new ArrayList<>();
        for (int i = 0; i < contours.size(); i++) {
            Rect rect = boundingRect(contours.get(i));
            rects.add(rect);
        }//creates a bounding rectangle


        if (rects.size() != 0) {

            this.largestRect = VisionUtils.sortRectsByMaxOption(1, VisionUtils.RECT_OPTION.AREA, rects).get(0);

            rectangle(output, largestRect, orange, 10); //, thickness);

            targetDetected = true;


        } else {
            targetDetected = false;
        }

        //rectangleOutline = largestRect.clone();
//sorts for the largest blocks

        //   Rect testRectangale = new Rect(new Point(0, 0), new  Point(100, 100));
//testing things
        drawContours(output, contours, -1, lightBlue);

        //rectangle(output, largestRect, orange);


        return output;

    }


    public static Rect getRectangle() {
        return largestRect;
    }

    public static double getError() {
        if (targetDetected) {
            double centerRect = largestRect.x + (largestRect.width / 2);
            //error 157
            double error = (IMG_WIDTH / 2) - centerRect;
            return error;
        } else {
            return 0;
        }

    }


    public static double getWidth() {
        if (targetDetected) {
            return largestRect.width;

        } else {
            return 0;
        }
    }

    public static double getCenter() {
        double centerRect = 0;
        if (largestRect != null) {
            centerRect = largestRect.x + (largestRect.width / 2) + offset;
        }

        return centerRect;
    }

    public static int getRectPos() {
        int rectPos = 0;

        if (largestRect != null) {
            if (getCenter() < IMG_WIDTH / 5) {
                rectPos = 1;
                //left position
            } else if (getCenter() > IMG_WIDTH / 5 && getCenter() < IMG_WIDTH - (IMG_WIDTH /3.5)) {
                rectPos = 2;

            } else if (getCenter() > IMG_WIDTH - (IMG_WIDTH / 3.5)) {
                //all of these values are super random and I am aware of how messy this is
                rectPos = 3;
                //right position
            }
        }


        return (rectPos);
    }

    public static void getPixel(int pixelx, int pixely){
        int pixelH;
        int pixelS;
        int pixelV;

    }
}