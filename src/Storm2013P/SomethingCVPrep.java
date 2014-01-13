/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Storm2013P; 
import com.googlecode.javacv.CanvasFrame;
import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_imgproc.*;
import edu.wpi.first.smartdashboard.camera.WPICameraExtension;
import edu.wpi.first.smartdashboard.properties.IntegerProperty;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIImage;
import edu.wpi.first.smartdashboard.properties.MultiProperty;
import edu.wpi.first.wpijavacv.WPIBinaryImage;
import edu.wpi.first.wpijavacv.WPIContour;
import edu.wpi.first.wpijavacv.WPIPolygon;
import com.googlecode.javacv.cpp.opencv_imgproc;
import com.googlecode.javacv.cpp.opencv_imgproc.*;
import edu.wpi.first.smartdashboard.camera.WPILaptopCameraExtension;
import edu.wpi.first.smartdashboard.gui.elements.Label;
import edu.wpi.first.smartdashboard.properties.ColorProperty;
import edu.wpi.first.smartdashboard.properties.DoubleProperty;
import edu.wpi.first.wpijavacv.StormExtensions;
import edu.wpi.first.wpijavacv.WPIColor;
import edu.wpi.first.wpijavacv.WPIPoint;
import java.awt.Color;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JDialog;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.smartdashboard.robot.Robot;
import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.Scanner;
import javax.imageio.ImageIO;

/**
 *
 * @author Tim Gordon
 * Start date: 10/22/13
 */


public class SomethingCVPrep 
//extends WPILaptopCameraExtension {
extends WPICameraExtension {
    
    public MultiProperty processing = new MultiProperty(this, "Process how far");
    


    public enum processingSteps{
        doNothing,threshold,contours,everything
    }
    
    public DoubleProperty  cameraXAngle = new DoubleProperty(this, "Camera horizontal FOV angle", 47);
    public DoubleProperty  cameraYAngle = new DoubleProperty(this, "Camera vertical FOV angle", 36.13);
    public IntegerProperty 
            hueMinCircle = new IntegerProperty(this, "Hue minimum value for circle", 85),
            hueMaxCircle = new IntegerProperty(this, "Hue maximum value for circle", 130),
            satMinCircle = new IntegerProperty(this, "Saturation minimum value for circle", 110),
            satMaxCircle = new IntegerProperty(this, "Saturation maximum value for circle", 185),
            valMinCircle = new IntegerProperty(this, "Value minimum value for circle", 0),
            valMaxCircle = new IntegerProperty(this, "Value maximum value for circle", 185),
            hueMinSquare = new IntegerProperty(this, "Hue minimum value for rectangle", 85),
            hueMaxSquare = new IntegerProperty(this, "Hue maximum value for rectangle", 130),
            satMinSquare = new IntegerProperty(this, "Saturation minimum value for rectangle", 110),
            satMaxSquare = new IntegerProperty(this, "Saturation maximum value for rectangle", 185),
            valMinSquare = new IntegerProperty(this, "Value minimum value for rectangle", 0),
            valMaxSquare = new IntegerProperty(this, "Value maximum value for rectangle", 185),
            closings = new IntegerProperty(this, "Closing iterations", 2);
    public ColorProperty colorProp = new ColorProperty(this, "Contour color", Color.BLACK);
    public IplConvKernel morphologyKernel;
    public IntegerProperty 
            width = new IntegerProperty(this, "Contour width", 5),
            verticesCirc = new IntegerProperty(this, "Number of vertices", 8),
            verticesRect = new IntegerProperty(this, "Number of vertices", 4),
            targetWidth = new IntegerProperty(this, "Target width for targets", 62),
            targetHeight = new IntegerProperty(this, "Target height for targets", 20);
            
    private WPIImage ret;
            
    public DoubleProperty
            distance = new DoubleProperty(this, "Target radius of circle", .2),
            range = new DoubleProperty(this, "Margin between target and actual radii", 0),
    
            minRatio = new DoubleProperty(this, "Minimum value for target ratio", 0.25),
            maxRatio = new DoubleProperty(this, "Maximum value for target ratio", 0.50),
            targetMargin  = new DoubleProperty(this, "Margin of error for target dimensions", 2.0),
            percentAccCircle = new DoubleProperty(this, "Polygon approx for ball", 10),
            percentAccRect = new DoubleProperty(this, "Polygon approx for rectangle", 10);
    
    private IplImage bin;
    
    public ArrayList<IplImage> displayedImages = new ArrayList<>();
    private ArrayList<CanvasFrame> frames = new ArrayList<>();
    
    private static final ITable outputTable = Robot.getTable();
    
    public SomethingCVPrep() {
        processing.add("Threshold", processingSteps.threshold);
        processing.add("Nothing", processingSteps.doNothing);
        processing.add("Contours", processingSteps.contours);
        processing.add("Everything", processingSteps.everything);
        try {
            System.setOut(new PrintStream("C:\\Users\\Tim\\Downloads\\SomethingCVPrep.txt"));
        } catch (FileNotFoundException ex) {
            Logger.getLogger(SomethingCVPrep.class.getName()).log(Level.SEVERE, null, ex);
        }
    }
    
    @Override
    public WPIImage processImage(WPIColorImage rawImage){
        if(processing.getValue()== (processingSteps.doNothing)){return rawImage;}
        for(int i = 0;i<2;i++){
            WPIBinaryImage thresholds;
            WPIContour [] countours;
            WPIPolygon [] checkedPolygons;
            WPIColor wpiColorProp = new WPIColor(colorProp.getValue());
            
            if(i==0){
                thresholds = findThresholds(rawImage, hueMinCircle.getValue(), hueMaxCircle.getValue(), satMinCircle.getValue(), satMaxCircle.getValue(), valMinCircle.getValue(), valMaxCircle.getValue());
            }else{
                thresholds = findThresholds(rawImage, hueMinSquare.getValue(), hueMaxSquare.getValue(), satMinSquare.getValue(), satMaxSquare.getValue(), valMinSquare.getValue(), valMaxSquare.getValue());
            }
            if(processing.getValue() == processingSteps.threshold){
                // Allocate _ret if it's the first time, otherwise reuse it.
                if(ret == null) {
                    ret = StormExtensions.makeWPIGrayscaleImage(bin);
                } else {
                    StormExtensions.copyImage(ret, bin);
                }

                return ret;
            }


            countours = StormExtensions.findConvexContours(thresholds);

            if(processing.getValue() == processingSteps.contours){rawImage.drawContours(countours, wpiColorProp, width.getValue());return rawImage;}
            
            if(i == 0){ //if i is 0, it looks for the ball. If it is 1, it looks for the target(s)
                if(countours.length != 0){
                    checkedPolygons = findCircle(countours, verticesCirc.getValue());
                    if(checkedPolygons != null && checkedPolygons.length !=0){

                        for(int x = 0;x<checkedPolygons.length;x++){

                            double centerX, centerY, YPos, XPos, YAngle, XAngle;
                            WPIPolygon y = checkedPolygons[x];

                            centerX = y.getX() + (y.getWidth()/2);
                            centerY = y.getY() + (y.getHeight()/2);

                            XPos = (2*centerX)/rawImage.getWidth() - 1;
                            YPos = (2*centerY)/rawImage.getHeight() - 1;

                            YAngle = YPos * (cameraYAngle.getValue()/2);
                            XAngle = XPos * (cameraXAngle.getValue()/2);

                            outputTable.putNumber("Ball X position ", XPos);
                            outputTable.putNumber("Ball Y position ", YPos);
                            outputTable.putNumber("Ball horizontal angle", XAngle);
                            outputTable.putNumber("Ball vertical angle", YAngle);
                            outputTable.putBoolean("Found ball ", true);
                        }

                        rawImage.drawPolygons(checkedPolygons, wpiColorProp, width.getValue());

                    }else{
                        outputTable.putBoolean("Found ball ", false);
                    }
                }
            }else{
                if(countours.length != 0){
                    checkedPolygons = checkPolygons(countours, verticesRect.getValue());
                    if(checkedPolygons != null && checkedPolygons.length !=0){

                        for(int x = 0;x<checkedPolygons.length;x++){

                            double centerX, centerY, YPos, XPos, YAngle, XAngle;
                            WPIPolygon y = checkedPolygons[x];

                            centerX = y.getX() + (y.getWidth()/2);
                            centerY = y.getY() + (y.getHeight()/2);

                            XPos = (2*centerX)/rawImage.getWidth() - 1;
                            YPos = (2*centerY)/rawImage.getHeight() - 1;

                            YAngle = YPos * (cameraYAngle.getValue()/2);
                            XAngle = XPos * (cameraXAngle.getValue()/2);

                            outputTable.putNumber("Target X position ", XPos);
                            outputTable.putNumber("Target Y position ", YPos);
                            outputTable.putNumber("Target horizontal angle", XAngle);
                            outputTable.putNumber("Target vertical angle", YAngle);
                            outputTable.putBoolean("Found target ", true);
                        }

                        rawImage.drawPolygons(checkedPolygons, wpiColorProp, width.getValue());

                    }else{
                        outputTable.putBoolean("Found target ", false);
                    }
                }
            }
        }
        return rawImage;
    }
    
    
    public WPIBinaryImage findThresholds(WPIColorImage rawImage, Integer hueMin, Integer hueMax, Integer satMin, Integer satMax, Integer valMin, Integer valMax) {
        
        IplImage hueMinImg;
        IplImage satMinImg;
        IplImage valMinImg;
        IplImage temp;
        IplConvKernel morphKernel;
        morphKernel = IplConvKernel.create(3, 3, 1, 1, opencv_imgproc.CV_SHAPE_RECT, null);;
        
        hueMinImg = IplImage.create(cvSize(rawImage.getWidth(), rawImage.getHeight()), 8, 1);
        satMinImg = IplImage.create(cvSize(rawImage.getWidth(), rawImage.getHeight()), 8, 1);
        valMinImg = IplImage.create(cvSize(rawImage.getWidth(), rawImage.getHeight()), 8, 1);
        bin = IplImage.create(cvSize(rawImage.getWidth(), rawImage.getHeight()), 8, 1);
        temp = IplImage.create(cvSize(rawImage.getWidth(), rawImage.getHeight()), 8, 1);
        
        IplImage camIn = StormExtensions.getIplImage(rawImage);
        IplImage hsv =IplImage.create(cvSize(rawImage.getWidth(), rawImage.getHeight()), 8, 3);
        opencv_imgproc.cvCvtColor(camIn, hsv, opencv_imgproc.CV_BGR2HSV);
        cvSplit(hsv, hueMinImg, satMinImg, valMinImg, null);
        
        opencv_imgproc.cvThreshold(hueMinImg, bin, hueMin - 1, 255, opencv_imgproc.CV_THRESH_BINARY);
        opencv_imgproc.cvThreshold(hueMinImg, temp, hueMax, 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        
        cvAnd(temp, bin, hueMinImg, null);
        
        opencv_imgproc.cvThreshold(satMinImg, bin, satMin - 1, 255, opencv_imgproc.CV_THRESH_BINARY);
        opencv_imgproc.cvThreshold(satMinImg, temp, satMax, 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        
        cvAnd(temp, bin, satMinImg, null);
                
        opencv_imgproc.cvThreshold(valMinImg, bin, valMin - 1, 255, opencv_imgproc.CV_THRESH_BINARY);
        opencv_imgproc.cvThreshold(valMinImg, temp, valMax, 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        
        cvAnd(temp, bin, bin, null);
        
        cvAnd(satMinImg, bin, bin, null);
        cvAnd(hueMinImg, bin, bin, null);
        
        opencv_imgproc.cvMorphologyEx(bin, bin, null, morphKernel, opencv_imgproc.CV_MOP_CLOSE, closings.getValue());
        
        return StormExtensions.makeWPIBinaryImage(bin);
        
    }
    
    public WPIPolygon[] checkPolygons(WPIContour[] countours, Integer vertices) {
        
        //System.out.println("Got in well enough");
        ArrayList<WPIPolygon> polygons = new ArrayList<WPIPolygon>();
        ArrayList<WPIPolygon> rectangles = new ArrayList<WPIPolygon>();
        WPIPolygon [] rects;
        int q = 0;
        ArrayList<WPIPolygon> checkedPolygons = new ArrayList<WPIPolygon>();
                
        for(WPIContour c:countours){
            
            double sideRatio = ((double)c.getHeight()/(double)c.getWidth());
            int targetArea = targetHeight.getValue() * targetWidth.getValue();
            if(sideRatio < maxRatio.getValue() && sideRatio > minRatio.getValue() 
                    && targetArea > (targetArea - targetMargin.getValue()) && targetArea < (targetArea + targetMargin.getValue())){
                polygons.add(c.approxPolygon(percentAccRect.getValue())); q++; System.out.println("Added " + q + "polygon");
            }
        
            for(int y = 0; y < polygons.size(); y++){
                int centerX, centerY, YPos, XPos;

                WPIPolygon p = polygons.get(y);

                int [] sideOrder = new int[vertices]; //1 means side is horizontal, 2 means it is vertical, 0 means it's perfectly diagonal
                WPIPoint [] points;
                boolean orderChecks = false;
                if(p.isConvex() && p.getNumVertices() == vertices){
                    points = p.getPoints();
                    for(int x = 0; x< vertices;x++){
                        if(diff(points[x].getX(), points[(x+1) %vertices].getX()) > 
                                diff(points[x].getY(), points[(x+1) %vertices].getY())){
                            sideOrder[x] = 1;
                        }else{
                            if(diff(points[x].getX(), points[(x+1) %vertices].getX()) > 
                                    diff(points[x].getY(), points[(x+1) %vertices].getY())){
                                sideOrder[x] = 2;
                            }
                        }
                    }
                
                    for(int x = 0; x<vertices; x++){
                        if(!(sideOrder[x] + sideOrder[(x+1) %vertices] == 3)){
                            orderChecks = false;
                        }
                    }
                }
                if(!orderChecks){
                    polygons.remove(y);   
                }
            
            
            }
            if(polygons.size() != 0){
               checkedPolygons.addAll(polygons);
            }
    
            
        }
        if(checkedPolygons.size() != 0){
            rects = new WPIPolygon[checkedPolygons.size()];
            for(int i =0;i<checkedPolygons.size();i++){
                rects[i]=checkedPolygons.get(i);
            }
            return rects;
        }
        return null;
        //TODO: MAKE METHOD FOR FINDING SLOPE
    }
        
    public int diff(int x1, int x2){
        return Math.abs(x1 - x2);
    }
    
    public WPIPolygon[] findCircle(WPIContour[] contours, int vertices){
        ArrayList<WPIPolygon> circleChecked = new ArrayList<WPIPolygon> ();
        for(int x=0;x<contours.length;x++){       
            WPIPolygon poly = contours[x].approxPolygon(percentAccCircle.getValue());
            if(poly.getNumVertices() >= vertices){
                circleChecked.add(poly);
            }
        }
       for(int x=0;x<circleChecked.size();x++){
           boolean notCircle=false;
           double xAverage = 0, yAverage = 0;
           for(int y=0;y<circleChecked.get(x).getPoints().length;y++){
               xAverage+=circleChecked.get(x).getPoints()[y].getX();
               yAverage+=circleChecked.get(x).getPoints()[y].getY();
           }
           xAverage /= circleChecked.get(x).getPoints().length;
           yAverage /= circleChecked.get(x).getPoints().length;
           for(int y=0;y<circleChecked.get(x).getPoints().length;y++){
               double radius = distanceForm(xAverage, yAverage, circleChecked.get(x).getPoints()[y].getX(), circleChecked.get(x).getPoints()[y].getY());
               if(!(Math.abs((radius/distance.getValue()) - 1) <= range.getValue())){
                   notCircle = true;
               }
           }
           if(!notCircle){
               circleChecked.remove(x);
           }
       }
       
       
       if(circleChecked.size() != 0){
            WPIPolygon [] circles = new WPIPolygon[circleChecked.size()];
        
            for(int x = 0; x < circleChecked.size();x++){
                circles[x] = circleChecked.get(x);
            }
        //System.out.println("Found polygon(s)");
        return circles;
        }
       return null;
    }
       public static double distanceForm(double x1, double y1, double x2, double y2){
           return (Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2)));
       }
       
       public static void main(String[] args){
           
           SomethingCVPrep cv = new SomethingCVPrep();
           
           Scanner scanner = new Scanner(System.in);
           if(args.length == 0){
               System.out.println("Put in filenames seperated by spaces");
               System.exit(0);
           }
           for(int x = 0; x<args.length; x++){
               String filename = args[x];
               
               System.out.println(filename + ": ");
               
               WPIColorImage rawImage;
               
               try {
                  rawImage = new WPIColorImage(ImageIO.read(new File(filename)));
               } catch (IOException e) {
                  System.err.println("Could not find file!");
                  return;
            }
               
               WPIImage result;
               
               cv.processing.setValue(processingSteps.everything);
               result = cv.processImage(rawImage);
               
               cv.displayImage("Result: " + filename, StormExtensions.getIplImage(result));
               
               System.out.println("Enter continues");
               
               scanner.nextLine();
           }
       }
       
       public void displayImage(String title,IplImage image) {
            IplImage newImage = IplImage.create(image.cvSize(), image.depth(), image.nChannels());
            cvCopy(image, newImage);
            displayedImages.add(newImage);
            CanvasFrame result = new CanvasFrame(title);
            result.showImage(newImage.getBufferedImage());
            frames.add(result);
        }
    }
