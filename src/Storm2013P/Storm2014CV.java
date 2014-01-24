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
import edu.wpi.first.smartdashboard.properties.BooleanProperty;
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


public class Storm2014CV 
//extends WPILaptopCameraExtension {
extends WPICameraExtension {
    
    public MultiProperty processing = new MultiProperty(this, "Process how far");
    


    public enum processingSteps{
        doNothing,thresholdBall, thresholdTarget,contoursBall, contoursTarget,everything
    }
    
    public DoubleProperty  
            cameraXAngle = new DoubleProperty(this, "Camera horizontal FOV angle", 47),
            cameraYAngle = new DoubleProperty(this, "Camera vertical FOV angle", 36.13);
    public IntegerProperty 
            hueMinCircle = new IntegerProperty(this, "Hue minimum value for circle", 85),
            hueMaxCircle = new IntegerProperty(this, "Hue maximum value for circle", 130),
            satMinCircle = new IntegerProperty(this, "Saturation minimum value for circle", 110),
            satMaxCircle = new IntegerProperty(this, "Saturation maximum value for circle", 185),
            valMinCircle = new IntegerProperty(this, "Value minimum value for circle", 0),
            valMaxCircle = new IntegerProperty(this, "Value maximum value for circle", 185),
            hueMinSquare = new IntegerProperty(this, "Hue minimum value for rectangle", 50),
            hueMaxSquare = new IntegerProperty(this, "Hue maximum value for rectangle", 90),
            satMinSquare = new IntegerProperty(this, "Saturation minimum value for rectangle", 220),
            satMaxSquare = new IntegerProperty(this, "Saturation maximum value for rectangle", 255),
            valMinSquare = new IntegerProperty(this, "Value minimum value for rectangle", 60),
            valMaxSquare = new IntegerProperty(this, "Value maximum value for rectangle", 255),
            closings     = new IntegerProperty(this, "Closing iterations", 2),
            width        = new IntegerProperty(this, "Contour width", 5),
            verticesCirc = new IntegerProperty(this, "Number of ball vertices", 8),
            verticesRect = new IntegerProperty(this, "Number of target vertices", 4);
    public ColorProperty 
            circleColorProp = new ColorProperty(this, "Contour color for the ball", Color.BLACK),
            horizontalColorProp = new ColorProperty(this, "Contour color for the horizontal target", Color.MAGENTA),
            verticalColorProp = new ColorProperty(this, "Contour color for the vertical target", Color.CYAN);
    //public IplConvKernel morphologyKernel;
            
    private WPIImage ret;
    
    public WPIColorImage testImage, processImage;
    
    public BooleanProperty imageTest = new BooleanProperty(this, "Test with image?", false);
    
    public DoubleProperty
            distance = new DoubleProperty(this, "Target radius of circle", .2),
            range = new DoubleProperty(this, "Margin between target and actual radii", 0),
            percentAccCircle = new DoubleProperty(this, "Polygon approx for ball", 10),
            percentAccRect = new DoubleProperty(this, "Polygon approx for rectangle", 10),
            
            verticalTargetRatio = new DoubleProperty(this, "Ratio of vertical target, b:h", 0.125), //for last year's target, use 0.5
            verticalTargetMargin = new DoubleProperty(this, "Margin for vertical ratio", .04), //for last year's target, use 0.3
            horizontalTargetRatio = new DoubleProperty(this, "Ratio of horizontal target, b:h", 5.875), //for last year's target, use 2.0
            horizontalTargetMargin = new DoubleProperty(this, "Margin for horizontal ratio", .3), //for last year's target, use 0.9
            
            ballPerimeterVArea = new DoubleProperty(this, "Value for circumference over area of ball to be less than", 1.2); //for last year's target, use 0.9
    
    private IplImage bin;
    public boolean [] isVertical = new boolean [4];
    
    public ArrayList<IplImage> displayedImages = new ArrayList<>();
    
    private static final ITable outputTable = Robot.getTable();
    
    public Storm2014CV() {
        processing.add("Threshold Ball", processingSteps.thresholdBall);
        processing.add("Threshold Target", processingSteps.thresholdTarget);
        processing.add("Nothing", processingSteps.doNothing);
        processing.add("Contour Ball", processingSteps.contoursBall);
        processing.add("Contour Target", processingSteps.contoursTarget);
        processing.add("Everything", processingSteps.everything);
        /*try {
            System.setOut(new PrintStream("C:\\Users\\Tim\\Downloads\\SomethingCVPrep.txt"));
        } catch (FileNotFoundException ex) {
            Logger.getLogger(SomethingCVPrep.class.getName()).log(Level.SEVERE, null, ex);
        }*/
    }
    
    
    @Override
    public WPIImage processImage(WPIColorImage rawImage){
        if(imageTest.getValue()){
            try {
                testImage = new WPIColorImage(ImageIO.read(new File("test.jpg")));
            } catch (IOException ex) {
                ex.printStackTrace();
            }
            
            processImage = new WPIColorImage(testImage.getBufferedImage());
            
            rawImage = processImage;
        }
        if(processing.getValue()== (processingSteps.doNothing)){return rawImage;}
        
        for(int i = 0;i<2;i++){ //if i is 0, it looks for the ball. If it is 1, it looks for the target(s)
            WPIBinaryImage thresholds;
            WPIContour [] countours;
            WPIPolygon [] checkedPolygons;
            WPIColor wpiCircleColorProp = new WPIColor(circleColorProp.getValue());
            WPIColor wpiHorizontalColorProp = new WPIColor(horizontalColorProp.getValue());
            WPIColor wpiVerticalColorProp = new WPIColor(verticalColorProp.getValue());
            IplImage thresholdIPL;
            //System.out.println("tic");
            if(i==0){
                thresholds = findThresholds(rawImage, hueMinCircle.getValue(), hueMaxCircle.getValue(), satMinCircle.getValue(), satMaxCircle.getValue(), valMinCircle.getValue(), valMaxCircle.getValue());
                thresholdIPL = StormExtensions.getIplImage(thresholds);
                if(processing.getValue() == processingSteps.thresholdBall){
                // Allocate ret if it's the first time, otherwise reuse it.
                    if(ret == null) {
                        ret = StormExtensions.makeWPIGrayscaleImage(thresholdIPL);
                    } else {
                        StormExtensions.copyImage(ret, thresholdIPL);
                    }

                    return ret;
                }
            }else{
                thresholds = findThresholds(rawImage, hueMinSquare.getValue(), hueMaxSquare.getValue(), satMinSquare.getValue(), satMaxSquare.getValue(), valMinSquare.getValue(), valMaxSquare.getValue());
                thresholdIPL = StormExtensions.getIplImage(thresholds);
                if(processing.getValue() == processingSteps.thresholdTarget){
                // Allocate ret if it's the first time, otherwise reuse it.
                    if(ret == null) {
                        ret = StormExtensions.makeWPIGrayscaleImage(thresholdIPL);
                    } else {
                        StormExtensions.copyImage(ret, thresholdIPL);
                    }

                    return ret;
                }
            }
            
            countours = StormExtensions.findConvexContours(thresholds);

            if(processing.getValue() == processingSteps.contoursBall && i == 0){rawImage.drawContours(countours, wpiCircleColorProp, width.getValue());return rawImage;
            }else{if(processing.getValue() == processingSteps.contoursTarget && i == 1){rawImage.drawContours(countours, wpiCircleColorProp, width.getValue());return rawImage;}}
            
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

                            outputTable.putNumber("Ball X position", XPos);
                            outputTable.putNumber("Ball Y position", YPos);
                            outputTable.putNumber("Ball horizontal angle", XAngle);
                            outputTable.putNumber("Ball vertical angle", YAngle);
                            outputTable.putBoolean("Found ball", true);
                        }

                        rawImage.drawPolygons(checkedPolygons, wpiCircleColorProp, width.getValue());

                    }else{
                        outputTable.putBoolean("Found ball", false);
                    }
                }
            }else{
                if(countours.length != 0){
                    checkedPolygons = checkPolygons(countours, verticesRect.getValue().intValue());
                    if(checkedPolygons != null && checkedPolygons.length !=0){

                        ArrayList<WPIPolygon> horizontalRectangleList = new ArrayList<>();
                        ArrayList<WPIPolygon> verticalRectangleList = new ArrayList<>();
                        
                        for(WPIPolygon y: checkedPolygons){

                            //System.out.println("About to call isVert");
                            
                            if(isVert(y)){
                                verticalRectangleList.add(y);
                            }else{
                                horizontalRectangleList.add(y);
                            }
                            
                        }
                        
                        ArrayList<WPIPolygon> finalVertical = new ArrayList<>();
                        ArrayList<WPIPolygon> finalHorizontal = new ArrayList<>();
                        System.out.println("tic-toc");
                        for(WPIPolygon p : verticalRectangleList){
                            
                            double [] sideLengths = new double[verticesRect.getValue().intValue()];
                            
                            
                            WPIPoint [] points;
                            points = p.getPoints();
                            for(int j = 0;j<verticesRect.getValue().intValue();j++){
                                sideLengths[j]=distanceForm(points[j].getX(), points[j].getY(), points[(j+1) % verticesRect.getValue().intValue()].getX(), points[(j+1) % verticesRect.getValue().intValue()].getY());
                            }

                            double averageSide1 = (sideLengths[0] + sideLengths[2])/2;
                            double averageSide2 = (sideLengths[1] + sideLengths[3])/2;
                            
                            double aspectRatio = averageSide2/averageSide1;
                            //System.out.println(Math.abs((verticalTargetRatio.getValue().doubleValue()) - (aspectRatio)) + "     " + verticalTargetMargin.getValue().doubleValue());
                            if(Math.abs((verticalTargetRatio.getValue().doubleValue()) - (aspectRatio)) < verticalTargetMargin.getValue().doubleValue()){
                                finalVertical.add(p);
                            }else{
                                System.out.println(Math.abs((verticalTargetRatio.getValue().doubleValue()) - (aspectRatio)) + "     " + verticalTargetMargin.getValue().doubleValue());
                            }
                        }
                        for(WPIPolygon p : horizontalRectangleList){
                            double [] sideLengths = new double[verticesRect.getValue().intValue()];
                            
                            WPIPoint [] points;
                            points = p.getPoints();
                            for(int j = 0;j<verticesRect.getValue().intValue();j++){
                                sideLengths[j]=distanceForm(points[j].getX(), points[j].getY(), points[(j+1) % verticesRect.getValue().intValue()].getX(), points[(j+1) % verticesRect.getValue().intValue()].getY());
                            }

                            double averageSide1 = (sideLengths[0] + sideLengths[2])/2;
                            double averageSide2 = (sideLengths[1] + sideLengths[3])/2;
                            
                            //System.out.println(averageSide2);
                            
                            double aspectRatio = averageSide2/averageSide1;
                            //System.out.println(Math.abs((horizontalTargetRatio.getValue().doubleValue()) - (aspectRatio)) + "     " + horizontalTargetMargin.getValue().doubleValue());
                            
                            if(Math.abs((horizontalTargetRatio.getValue().doubleValue()) - (aspectRatio)) < horizontalTargetMargin.getValue().doubleValue()){
                                finalHorizontal.add(p);
                            }else{
                                System.out.println(Math.abs((horizontalTargetRatio.getValue().doubleValue()) - (aspectRatio)) + "     " + horizontalTargetMargin.getValue().doubleValue());
                            }
                        }
                        
                        WPIPolygon[] verticalRectangle = new WPIPolygon[finalVertical.size()];
                        WPIPolygon[] horizontalRectangle = new WPIPolygon[finalHorizontal.size()];
                        if(horizontalRectangle.length == 0){
                            outputTable.putBoolean("Found horizontal target", false);
                        }
                        if(verticalRectangle.length == 0){
                            outputTable.putBoolean("Found vertical target", false);
                        }
                        for(int j = 0; j<finalVertical.size();j++){
                            
                            WPIPolygon y = finalVertical.get(j);
                            double centerX, centerY, YPos, XPos, YAngle, XAngle;

                            centerX = y.getX() + (y.getWidth()/2);
                            centerY = y.getY() + (y.getHeight()/2);

                            XPos = (2*centerX)/rawImage.getWidth() - 1;
                            YPos = (2*centerY)/rawImage.getHeight() - 1;

                            YAngle = YPos * (cameraYAngle.getValue()/2);
                            XAngle = XPos * (cameraXAngle.getValue()/2);

                            outputTable.putNumber("Vertical target X position", XPos);
                            outputTable.putNumber("Vertical target Y position", YPos);
                            outputTable.putNumber("Vertical target horizontal angle", XAngle);
                            outputTable.putNumber("Vertical target vertical angle", YAngle);
                            
                            
                            
                            verticalRectangle[j] = y;
                        }
                        for(int j = 0;j<finalHorizontal.size();j++){
                            
                            WPIPolygon y = finalHorizontal.get(j);
                            double centerX, centerY, YPos, XPos, YAngle, XAngle;

                            centerX = y.getX() + (y.getWidth()/2);
                            centerY = y.getY() + (y.getHeight()/2);

                            XPos = (2*centerX)/rawImage.getWidth() - 1;
                            YPos = (2*centerY)/rawImage.getHeight() - 1;

                            YAngle = YPos * (cameraYAngle.getValue()/2);
                            XAngle = XPos * (cameraXAngle.getValue()/2);

                            outputTable.putNumber("Horizontal target X position", XPos);
                            outputTable.putNumber("Horizontal target Y position", YPos);
                            outputTable.putNumber("Horizontal target horizontal angle", XAngle);
                            outputTable.putNumber("Horizontal target vertical angle", YAngle);
                            outputTable.putBoolean("Found horizontal target", true);
                            
                            
                            horizontalRectangle[j] = y;
                        }
                        
                        rawImage.drawPolygons(verticalRectangle, wpiVerticalColorProp, width.getValue());
                        rawImage.drawPolygons(horizontalRectangle, wpiHorizontalColorProp, width.getValue());

                    }else{
                        outputTable.putBoolean("Found target(s) ", false);
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
        morphKernel = IplConvKernel.create(3, 3, 1, 1, opencv_imgproc.CV_SHAPE_RECT, null);
        
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
    
    public WPIPolygon[] checkPolygons(WPIContour[] countours, int vertices) {
        
        //System.out.println("Got in well enough");
        ArrayList<WPIPolygon> polygons = new ArrayList<WPIPolygon>();
        ArrayList<WPIPolygon> rectangles = new ArrayList<WPIPolygon>();
        WPIPolygon [] rects;
        
        ArrayList<WPIPolygon> checkedPolygons = new ArrayList<WPIPolygon>();
                
        for(WPIContour c:countours){
            
            double sideRatio = ((double)c.getHeight()/(double)c.getWidth());
            
            polygons.add(c.approxPolygon(percentAccRect.getValue()));
                
                
            for(int y = 0; y < polygons.size(); y++){
                int centerX, centerY, YPos, XPos;
                
                WPIPolygon p = polygons.get(y);
                
                double aspectRatio;
                WPIPoint [] points;
                boolean orderChecks = true;
                //System.out.println(p.isConvex() + "   " + p.getNumVertices() + "   " + vertices);
                if(!p.isConvex() || p.getNumVertices() != vertices){
                    continue;
                }
                
                points = p.getPoints();
                //System.out.println(vertices);

                for(int x = 0; x< vertices;x++){
                    if(checkVert(points[x], points[(x+1) % vertices])){
                        isVertical[x] = true;
                    }else{
                        isVertical[x] = false;
                    }

                }

                if(!(isVertical[0] && !isVertical[1] && isVertical[2] && !isVertical[3]) || (!isVertical[0] && isVertical[1] && !isVertical[2] && isVertical[3])){
                    orderChecks = false;
                    //System.out.println("Failed side order check");
                }
                

                

                if(orderChecks){
                    checkedPolygons.add(p);
                }
            
            
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
    
    public boolean isVert(WPIPolygon p){
        WPIPoint[] points = p.getPoints();
        double averageSide0 = ((distanceForm(points[0].getX(), points[0].getY(), points[1].getX(), points[1].getY())) + (distanceForm(points[2].getX(), points[2].getY(), points[3].getX(), points[3].getY())))/2;
        double averageSide1 = ((distanceForm(points[1].getX(), points[1].getY(), points[2].getX(), points[2].getY())) + (distanceForm(points[3].getX(), points[3].getY(), points[0].getX(), points[0].getY())))/2;
        double aspectRatio;
        if(checkVert(points[0], points[1])){
            aspectRatio = averageSide1/averageSide0;    
        }else{
            aspectRatio = averageSide0/averageSide1;
        }
        //System.out.println(aspectRatio);
        if(aspectRatio <= 1){
            return true;
        }else{
            return false;
        }
    }
    
    
    public boolean checkVert(WPIPoint x1, WPIPoint x2){
        System.out.println(Math.abs(x2.getY()-x1.getY()) > (Math.abs(x2.getX()-x1.getX())));
        return (Math.abs(x2.getY()-x1.getY()) > (Math.abs(x2.getX()-x1.getX())));
    }
    
    public WPIPolygon[] findCircle(WPIContour[] contours, int vertices){
        ArrayList<WPIPolygon> circleChecked = new ArrayList<WPIPolygon> ();
        for(WPIContour x: contours){       
            WPIPolygon poly = x.approxPolygon(percentAccCircle.getValue());
            double contourArea = cvContourArea(StormExtensions.getCvSeq(x), CV_WHOLE_ARR, 0);
            if(poly.getNumVertices() >= vertices && x.getlength()/contourArea < ballPerimeterVArea.getValue()){
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
           
           Storm2014CV cv = new Storm2014CV();
           
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
        }
    }