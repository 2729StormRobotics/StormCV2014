package Storm2013P; 
import com.googlecode.javacv.CanvasFrame;
import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_imgproc.*;
import com.googlecode.javacv.cpp.opencv_core.CvSize;
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
import edu.wpi.first.smartdashboard.properties.BooleanProperty;
import edu.wpi.first.smartdashboard.properties.ColorProperty;
import edu.wpi.first.smartdashboard.properties.DoubleProperty;
import edu.wpi.first.smartdashboard.properties.StringProperty;
import edu.wpi.first.wpijavacv.StormExtensions;
import edu.wpi.first.wpijavacv.WPIColor;
import edu.wpi.first.wpijavacv.WPIPoint;
import java.awt.Color;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.smartdashboard.robot.Robot;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Scanner;
import java.util.logging.Level;
import java.util.logging.Logger;
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
    public MultiProperty ballSelection = new MultiProperty(this, "Select which ball?");
    public MultiProperty targetSelection = new MultiProperty(this, "Select which target?");
    public MultiProperty distanceReference = new MultiProperty(this, "Where on target to base reference point");
    
    
    
    public enum processingSteps{
        doNothing, thresholdBall, thresholdTarget, contoursBall, contoursTarget, polygonApproxBall, polygonApproxTarget, checkCheckedPolygons, showVerticalRects, showHorizontalRects, everything, checkCheckedTargetPolygons
    }
    
    public enum selectionSteps{
        closest, farthest, biggest, centered
    }
    
    public enum referencePoint{
        top, bottom, middle
    }
    
    private final DoubleProperty  
            cameraFovX          = new DoubleProperty(this, "Camera horizontal FOV angle", 47),
            cameraFovY          = new DoubleProperty(this, "Camera vertical FOV angle", 36.13),
            screenPercentage    = new DoubleProperty(this, "Percentage of screen sphere must cover", 1.0),
            cameraAngle         = new DoubleProperty(this, "Angle of camera relative to ground in degrees", 0.0),
            targetHeight        = new DoubleProperty(this, "Height of target in inches", 53.5),
            cameraHeight        = new DoubleProperty(this, "Height of camera in inches", 6.0),
            leverArm        = new DoubleProperty(this, "Length of tilter from pivot point", 12.0);
    private final IntegerProperty        
            hueMinRedBall          = new IntegerProperty(this, "Hue minimum value for red ball", 125), //For picture, 1
            hueMaxRedBall          = new IntegerProperty(this, "Hue maximum value for red ball", 170), //For picture, 19
            satMinRedBall          = new IntegerProperty(this, "Saturation minimum value for red ball", 90), //For picture, 200
            satMaxRedBall          = new IntegerProperty(this, "Saturation maximum value for red ball", 170), //For picture, 215
            valMinRedBall          = new IntegerProperty(this, "Value minimum value for red ball", 50), //For picture, 78
            valMaxRedBall          = new IntegerProperty(this, "Value maximum value for red ball", 230), //For picture, 200
            hueMinBlueBall         = new IntegerProperty(this, "Hue minimum value for blue ball", 100), 
            hueMaxBlueBall         = new IntegerProperty(this, "Hue maximum value for blue ball", 110), 
            satMinBlueBall         = new IntegerProperty(this, "Saturation minimum value for blue ball", 230), 
            satMaxBlueBall         = new IntegerProperty(this, "Saturation maximum value for blue ball", 255), 
            valMinBlueBall         = new IntegerProperty(this, "Value minimum value for blue ball", 10), 
            valMaxBlueBall         = new IntegerProperty(this, "Value maximum value for blue ball", 230), 
            hueMinSquare           = new IntegerProperty(this, "Hue minimum value for rectangle", 50), //For picture: 75
            hueMaxSquare           = new IntegerProperty(this, "Hue maximum value for rectangle", 90), //For picture: 100
            satMinSquare           = new IntegerProperty(this, "Saturation minimum value for rectangle", 220),//for picture: 10
            satMaxSquare           = new IntegerProperty(this, "Saturation maximum value for rectangle", 255),//for picture: 70
            valMinSquare           = new IntegerProperty(this, "Value minimum value for rectangle", 60),//for picture: 210
            valMaxSquare           = new IntegerProperty(this, "Value maximum value for rectangle", 255),//for picture: 255
            closingsForBall        = new IntegerProperty(this, "Closing iterations for ball", 2),
            closingsForTarget      = new IntegerProperty(this, "Closing iterations for target", 0),
            width                  = new IntegerProperty(this, "Contour width", 3),
            verticesCirc           = new IntegerProperty(this, "Number of ball vertices", 5),
            verticesRect           = new IntegerProperty(this, "Number of target vertices", 4);
    private final ColorProperty          
            circleColorProp       = new ColorProperty(this, "Contour color for the ball", Color.YELLOW),
            horizontalColorProp   = new ColorProperty(this, "Contour color for the horizontal target", Color.MAGENTA),
            verticalColorProp     = new ColorProperty(this, "Contour color for the vertical target", Color.CYAN);
    //public IplConvKernel morphologyKernel;
    
    public final StringProperty saveLocation = new StringProperty(this, "Location for pictures", System.getenv("USERPROFILE") + "/Captures");
    
    private WPIImage ret;
    
    private WPIColorImage testImage, processImage;
    
    private final BooleanProperty imageTest = new BooleanProperty(this, "Test with image?", false);
    
    private final DoubleProperty
            percentAccCircle = new DoubleProperty(this, "Polygon approx for ball", 4),
            percentAccRect = new DoubleProperty(this, "Polygon approx for rectangle", 10),
            
            verticalTargetRatio     = new DoubleProperty(this, "Ratio of vertical target, b:h", 0.125), //for last year's target, use 0.5
            verticalTargetMargin    = new DoubleProperty(this, "Margin for vertical ratio", .04), //for last year's target, use 0.3
            horizontalTargetRatio   = new DoubleProperty(this, "Ratio of horizontal target, b:h", 7.4), //for last year's target, use 2.0
            horizontalTargetMargin  = new DoubleProperty(this, "Margin for horizontal ratio", .01), //for last year's target, use 0.9
            
            ballPerimeterVArea      = new DoubleProperty(this, "Value for circumference over area of ball to be less than", 1.4),
    
            ballPercentInPicture    = new DoubleProperty(this, "Percentage of screen that the ball width occupies", 110.0/320.0),
            distance                = new DoubleProperty(this, "Distance from camera for above percentage in inches", 87.5),
    
            savePeriod              = new DoubleProperty(this, "Save period", 1.0);
    private IplImage bin;
    private IplImage hueImg;
    private IplImage satImg;
    private IplImage valImg;
    private IplImage temp;
    private CvSize size;
    private IplImage hsv;
    
    private double horizontalXAngle = 0.0, verticalXAngle = 0.0, horizontalYAngle = 0.0, verticalYAngle = 0.0;
    
    private long previousSaveTime = 0;
    
    private final boolean [] isVertical    = new boolean [4];
    
    private final ArrayList<IplImage> displayedImages = new ArrayList<>();
    
    private static final ITable outputTable = Robot.getTable();
    
    private double imageWidth, imageHeight;
    
    private WPIBinaryImage thresholds;
    private static WPIImage result;
    
    public Storm2014CV() {
        processing.add("Nothing", processingSteps.doNothing);
        processing.add("Threshold Ball", processingSteps.thresholdBall);
        processing.add("Threshold Target", processingSteps.thresholdTarget);
        processing.add("Contour Ball", processingSteps.contoursBall);
        processing.add("Contour Target", processingSteps.contoursTarget);
        processing.add("Approximate ball polygons", processingSteps.polygonApproxBall);
        processing.add("Approximate target polygons", processingSteps.polygonApproxTarget);
        processing.add("Output CheckedPolygons for ball", processingSteps.checkCheckedPolygons);
        processing.add("Output CheckedPolygons for target", processingSteps.checkCheckedTargetPolygons);
        processing.add("Output vertical rectangles", processingSteps.showVerticalRects);
        processing.add("Output horizontal", processingSteps.showHorizontalRects);
        processing.add("Everything", processingSteps.everything);
        processing.setDefault("Everything");
        
        ballSelection.add("Closest ball", selectionSteps.closest);
        ballSelection.add("Farthest ball", selectionSteps.farthest);
        ballSelection.add("Largest ball", selectionSteps.biggest);
        ballSelection.setDefault("Closest ball");
        
        targetSelection.add("Closest target", selectionSteps.closest);
        targetSelection.add("Farthest target", selectionSteps.farthest);
        targetSelection.add("Target closest to center", selectionSteps.centered);
        targetSelection.setDefault("Closest target");
        
        distanceReference.add("Top of target", referencePoint.top);
        distanceReference.add("Middle of target", referencePoint.middle);
        distanceReference.add("Bottom of target", referencePoint.bottom);
        distanceReference.setDefault(referencePoint.middle);
        
        /*try {
            System.setOut(new PrintStream("C:\\Users\\Tim\\Downloads\\SomethingCVPrep.txt"));
        } catch (FileNotFoundException ex) {
            Logger.getLogger(Storm2014CV.class.getName()).log(Level.SEVERE, null, ex);
        }*/
    }
    
    
    @Override
    public WPIImage processImage(WPIColorImage rawImage){
        
        double FovX = cameraFovX.getValue(), FovY = cameraFovY.getValue(), currentCameraAngle = cameraAngle.getValue() + outputTable.getNumber("Tilter angle", 0.0);
        
        long startTime = System.currentTimeMillis();
        Integer ballHueMin, ballHueMax, ballSatMin, ballSatMax, ballValMin, ballValMax;
        
        if(outputTable.getBoolean("Enabled", false)){
            long currentTime = System.currentTimeMillis();
            if(!imageTest.getValue() && savePeriod.getValue() >= 0  && (previousSaveTime <0 || previousSaveTime + savePeriod.getValue() <= currentTime)){
                previousSaveTime = currentTime;
                final IplImage picture = StormExtensions.getIplImage(rawImage);
                new Thread(){
                    private final String fileName = "/Capture " + new SimpleDateFormat("yyyy-MM-dd HH.mm.ss").format(new Date()) + ".jpg";
                    
                    private final IplImage copy = IplImage.create(picture.cvSize(), picture.depth(), picture.nChannels());
                    
                    //Apparently this is the constructor
                    {
                        cvCopy(picture, copy);
                        setPriority(Thread.MIN_PRIORITY);
                    }
                    @Override
                    public void run(){
                        try{
                            System.out.println("Saving \"" + fileName + "\"");
                            File folderFile = new File(saveLocation.getValue());
                            folderFile.mkdirs();
                            File out = new File(saveLocation.getValue() + fileName);
                            ImageIO.write(copy.getBufferedImage(), "jpg", out);
                            System.out.println("Saved \"" + fileName + "\"");
                        }catch(Exception ex){
                            Logger.getLogger(Storm2014CV.class.getName()).log(Level.SEVERE, "Save of \"" + fileName + "\" failed");
                            ex.printStackTrace();
                        }finally{
                            copy.deallocate();
                        }
                    }
                    
                }.start();
            }
        }else{
            previousSaveTime = -1;
        }
        
        if(outputTable.getBoolean("Blue Alliance?")){
            ballHueMin = hueMinBlueBall.getValue();
            ballHueMax = hueMaxBlueBall.getValue();
            ballSatMin = satMinBlueBall.getValue();
            ballSatMax = satMaxBlueBall.getValue();
            ballValMin = valMinBlueBall.getValue();
            ballValMax = valMaxBlueBall.getValue();
        }else{
            ballHueMin = hueMinRedBall.getValue();
            ballHueMax = hueMaxRedBall.getValue();
            ballSatMin = satMinRedBall.getValue();
            ballSatMax = satMaxRedBall.getValue();
            ballValMin = valMinRedBall.getValue();
            ballValMax = valMaxRedBall.getValue();
        }
        
        if(imageTest.getValue()){
            try {
                testImage = new WPIColorImage(ImageIO.read(new File("test.jpg")));
            } catch (IOException ex) {
                ex.printStackTrace();
            }
            
            processImage = new WPIColorImage(testImage.getBufferedImage());
            
            rawImage = processImage;
        }
        
        imageHeight = rawImage.getHeight();
        imageWidth = rawImage.getWidth();
        
        if(processing.getValue()== (processingSteps.doNothing)){return rawImage;}
        
        for(int i = 0;i<2;i++){ //if i is 0, it looks for the ball. If it is 1, it looks for the target(s)
            
            WPIContour [] contours;
            WPIPolygon [] checkedPolygons;
            WPIColor wpiCircleColorProp = new WPIColor(circleColorProp.getValue());
            WPIColor wpiHorizontalColorProp = new WPIColor(horizontalColorProp.getValue());
            WPIColor wpiVerticalColorProp = new WPIColor(verticalColorProp.getValue());
            IplImage thresholdIPL;
            
            if(size == null || size.width() != rawImage.getWidth() || size.height() != rawImage.getHeight()) {
                if(hsv != null){
                    hsv.deallocate();
                    bin.deallocate();
                    hueImg.deallocate();
                    satImg.deallocate();
                    valImg.deallocate();
                    temp.deallocate();
                }
                
                size    = cvSize(rawImage.getWidth(),rawImage.getHeight());
                hsv     = IplImage.create(size, 8, 3);
                bin     = IplImage.create(size, 8, 1);
                hueImg  = IplImage.create(size, 8, 1);
                satImg  = IplImage.create(size, 8, 1);
                valImg  = IplImage.create(size, 8, 1);
                temp    = IplImage.create(size, 8, 1);
            }
            
            if(i==0){
                thresholds = findThresholds(rawImage, ballHueMin, ballHueMax, ballSatMin, ballSatMax, ballValMin, ballValMax, closingsForBall.getValue());
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
                thresholds = findThresholds(rawImage, hueMinSquare.getValue(), hueMaxSquare.getValue(), satMinSquare.getValue(), satMaxSquare.getValue(), valMinSquare.getValue(), valMaxSquare.getValue(), closingsForTarget.getValue());
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
            
            contours = StormExtensions.findConvexContours(thresholds);
            
            if(processing.getValue() == processingSteps.contoursBall && i == 0){
                rawImage.drawContours(contours, wpiCircleColorProp, width.getValue());
                return rawImage;
            }else{
                if(processing.getValue() == processingSteps.contoursTarget && i == 1){
                    rawImage.drawContours(contours, wpiCircleColorProp, width.getValue());
                    return rawImage;
                }
            }
            
            if(i == 0){ //if i is 0, it looks for the ball. If it is 1, it looks for the target(s)
                
                WPIPolygon closest;
                
                double largestRadius = 0.0, longestDistance = 0.0, shortestDistance = 10000.0, distanceToLargestRadius = 0.0;
                boolean foundBall = false;
                
                if(contours.length != 0){
                    
                    checkedPolygons = findCircle(contours, verticesCirc.getValue());
                    
                    if(processing.getValue() == processingSteps.checkCheckedPolygons){
                        rawImage.drawPolygons(checkedPolygons, wpiCircleColorProp, width.getValue());
                        return rawImage;
                    }
                    
                    if(processing.getValue() == processingSteps.polygonApproxBall){
                        WPIPolygon [] returnBalls = new WPIPolygon[contours.length];
                        for(int k = 0;k<returnBalls.length;k++){
                            returnBalls[k] = contours[k].approxPolygon(percentAccCircle.getValue());
                        }
                        rawImage.drawPolygons(returnBalls, wpiCircleColorProp, width.getValue());
                        return rawImage;
                    }
                    
                    if(checkedPolygons != null && checkedPolygons.length !=0){
                        
                        //System.out.println("Have polygons");
                        
                        closest = checkedPolygons[0];
                        
                        for (WPIPolygon p : checkedPolygons) {
                            
                            WPIPoint [] vertices = p.getPoints();
                            double currentDistance;
                            double maxRadius = 0.0;
                            
                            for(int j = 0; j<vertices.length - 1;j++){
                                for(int k = j+1;k<vertices.length;k++){
                                    currentDistance = distanceFormula(vertices[j].getX(), vertices[j].getY(), vertices[k].getX(), vertices[k].getY());
                                    if(currentDistance > maxRadius)
                                        maxRadius = currentDistance;
                                    
                                }
                            }
                            
                            double ballPercentInCamera = maxRadius/(double)rawImage.getWidth();
                            double distanceToBall = (ballPercentInPicture.getValue()/ballPercentInCamera) * distance.getValue();
                            
                            if(distanceToBall < shortestDistance){
                                shortestDistance = distanceToBall;
                                closest = p;
                            }
                            
                            //System.out.println(longestDistance);
                        }
                        WPIPolygon finalBall = closest;
                        double distanceToBall = shortestDistance, XAngle, YAngle;
                        /*if(selection.getValue() == selectionSteps.farthest){
                            finalBall = farthest;
                            distanceToBall = longestDistance;
                        }else{
                            if(selection.getValue() == selectionSteps.biggest){
                                distanceToBall = distanceToLargestRadius;
                                finalBall = largest;
                            }
                        }
                        */
                        XAngle = ((2*(finalBall.getX() + (finalBall.getWidth()/2)))/rawImage.getWidth() - 1) * (FovX/2);
                        YAngle = ((2*(finalBall.getY() + (finalBall.getHeight()/2)))/rawImage.getHeight() - 1) * (FovY/2);
                        
                        foundBall = true;
                        
                        outputTable.putNumber("Ball horizontal angle to center", XAngle);
                        outputTable.putNumber("Ball vertical angle to center", YAngle);
                        outputTable.putNumber("Distance to ball in inches", distanceToBall);
                        rawImage.drawPolygon(finalBall, wpiCircleColorProp, width.getValue());
                        
                    }
                }
                outputTable.putBoolean("Found ball", foundBall);
            }else{
                
                boolean foundHorizontal = false, foundVertical = false;
                WPIPolygon selectedVertical = null;
                double verticalTargetDistance = 0.0; 
                
                if(contours.length != 0){
                    
                    if(processing.getValue() == processingSteps.polygonApproxTarget){
                        WPIPolygon [] returnTargets = new WPIPolygon[contours.length];
                        for(int k = 0;k<returnTargets.length;k++){
                            returnTargets[k] = contours[k].approxPolygon(percentAccRect.getValue());
                        }
                        rawImage.drawPolygons(returnTargets, wpiCircleColorProp, width.getValue());
                        return rawImage;
                    }
                    
                    
                    
                    checkedPolygons = checkPolygons(contours, verticesRect.getValue().intValue());
                    if(checkedPolygons != null && checkedPolygons.length !=0){
                        
                        if(processing.getValue() == processingSteps.checkCheckedTargetPolygons){
                            rawImage.drawPolygons(checkedPolygons, wpiCircleColorProp, width.getValue());
                        }
                        
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
                        
                        if(processing.getValue() == processingSteps.showHorizontalRects){
                            for(WPIPolygon p : horizontalRectangleList){
                                rawImage.drawPolygon(p, wpiCircleColorProp, width.getValue());
                            }
                        }
                        if(processing.getValue() == processingSteps.showVerticalRects){
                            for(WPIPolygon p : verticalRectangleList){
                                rawImage.drawPolygon(p, wpiCircleColorProp, width.getValue());
                            }
                        }
                        
                        ArrayList<WPIPolygon> finalVertical = new ArrayList<>();
                        ArrayList<WPIPolygon> finalHorizontal = new ArrayList<>();
                        //System.out.println("tic-toc");
                        
                        for(WPIPolygon p : verticalRectangleList){
                            
                            double [] sideLengths = new double[verticesRect.getValue().intValue()];
                            
                            
                            WPIPoint [] points;
                            points = p.getPoints();
                            for(int j = 0;j<verticesRect.getValue().intValue();j++){
                                sideLengths[j]=distanceFormula(points[j].getX(), points[j].getY(), points[(j+1) % verticesRect.getValue().intValue()].getX(), points[(j+1) % verticesRect.getValue().intValue()].getY());
                            }

                            double averageSide1 = (sideLengths[0] + sideLengths[2])/2;
                            double averageSide2 = (sideLengths[1] + sideLengths[3])/2;
                            
                            double aspectRatio = averageSide2/averageSide1;
                            //System.out.println(Math.abs((verticalTargetRatio.getValue().doubleValue()) - (aspectRatio)) + "     " + verticalTargetMargin.getValue().doubleValue());
                            if(Math.abs((verticalTargetRatio.getValue().doubleValue()) - (aspectRatio)) < verticalTargetMargin.getValue()){
                                finalVertical.add(p);
                            }else{
                                //System.out.println(Math.abs((verticalTargetRatio.getValue().doubleValue()) - (aspectRatio)) + "     " + verticalTargetMargin.getValue().doubleValue());
                            }
                        }
                        for(WPIPolygon p : horizontalRectangleList){
                            double [] sideLengths = new double[verticesRect.getValue().intValue()];
                            
                            WPIPoint [] points;
                            points = p.getPoints();
                            for(int j = 0;j<verticesRect.getValue().intValue();j++){
                                sideLengths[j]=distanceFormula(points[j].getX(), points[j].getY(), points[(j+1) % verticesRect.getValue().intValue()].getX(), points[(j+1) % verticesRect.getValue().intValue()].getY());
                            }

                            double averageSide1 = (sideLengths[0] + sideLengths[2])/2;
                            double averageSide2 = (sideLengths[1] + sideLengths[3])/2;
                            
                            //System.out.println(averageSide2);
                            
                            double aspectRatio = averageSide2/averageSide1;
                            //System.out.println(Math.abs((horizontalTargetRatio.getValue().doubleValue()) - (aspectRatio)) + "     " + horizontalTargetMargin.getValue().doubleValue());
                            
                            if(Math.abs((horizontalTargetRatio.getValue().doubleValue()) - (aspectRatio)) < horizontalTargetMargin.getValue()){
                                finalHorizontal.add(p);
                            }else{
                                //System.out.println(Math.abs((horizontalTargetRatio.getValue().doubleValue()) - (aspectRatio)) + "     " + horizontalTargetMargin.getValue().doubleValue());
                            }
                        }
                        
                        WPIPolygon[] verticalRectangle = new WPIPolygon[finalVertical.size()];
                        WPIPolygon[] horizontalRectangle = new WPIPolygon[finalHorizontal.size()];
                        double mostCentered = 1.0/0.0, farthest = 0.0, closest = 1.0/0.0, tempVerticalTargetDistance;
                        double tilterAngle = outputTable.getNumber("Tilter angle", 0.0) * Math.PI / 180.0;
                        for(int j = 0; j<finalVertical.size();j++){
                            
                            WPIPolygon y = finalVertical.get(j);
                            double centerX, centerY, YPos, XPos, YAngle, XAngle;
                            
                            centerX = y.getX() + (y.getWidth()/2);
                            centerY = y.getY() + (y.getHeight()/2);
                            
                            XPos = (2*centerX)/rawImage.getWidth() - 1;
                            YPos = (2*centerY)/rawImage.getHeight() - 1;
                            
                            XAngle = XPos * (FovX/2);
                            YAngle = YPos * (FovY/2);
                            
                            double distanceAngle = YAngle;
                            
                            if(distanceReference.getValue() == referencePoint.bottom){
                                distanceAngle = ((2*y.getY() + y.getHeight())/rawImage.getHeight() - 1) * (FovY/2);
                            }
                            if(distanceReference.getValue() == referencePoint.top){
                                distanceAngle = (2*y.getY()/rawImage.getHeight() - 1) * (FovY/2);
                            }
                            
                            tempVerticalTargetDistance = (targetHeight.getValue() - (leverArm.getValue()*Math.sin(tilterAngle) + cameraHeight.getValue()*Math.cos(tilterAngle)))/(Math.tan(cameraAngle.getValue()*Math.PI/180.0 + distanceAngle*Math.PI/180.0));
                            
                            if(targetSelection.getValue() == selectionSteps.closest && tempVerticalTargetDistance <closest){
                                verticalTargetDistance = tempVerticalTargetDistance;
                                verticalXAngle = XAngle;
                                verticalYAngle = YAngle;
                                closest = verticalTargetDistance;
                                foundVertical = true;
                                selectedVertical = finalVertical.get(j);
                            }
                            if(targetSelection.getValue() == selectionSteps.farthest && tempVerticalTargetDistance > farthest){
                                verticalTargetDistance = tempVerticalTargetDistance;
                                verticalXAngle = XAngle;
                                verticalYAngle = YAngle;
                                farthest = verticalTargetDistance;
                                foundVertical = true;
                                selectedVertical = finalVertical.get(j);
                            }
                            if(distanceFormula(0.0, 0.0, Math.abs(XPos), Math.abs(YPos)) < mostCentered && targetSelection.getValue() == selectionSteps.centered){
                                mostCentered = distanceFormula(0.0, 0.0, Math.abs(XPos), Math.abs(YPos));
                                verticalXAngle = XAngle;
                                verticalYAngle = YAngle;
                                verticalTargetDistance = tempVerticalTargetDistance;
                                foundVertical = true;
                                selectedVertical = finalVertical.get(j);
                            }
                            
                        }
                        
                        for(int j = 0;j<finalHorizontal.size();j++){
                            
                            WPIPolygon y = finalHorizontal.get(j);
                            double centerX, centerY, YPos, XPos, YAngle, XAngle;
                            
                            centerX = y.getX() + (y.getWidth()/2);
                            centerY = y.getY() + (y.getHeight()/2);
                            
                            XPos = (2*centerX)/rawImage.getWidth() - 1;
                            YPos = (2*centerY)/rawImage.getHeight() - 1;
                            
                            YAngle = YPos * (cameraFovY.getValue()/2);
                            XAngle = XPos * (cameraFovX.getValue()/2);
                            
                            horizontalXAngle = XAngle;
                            horizontalYAngle = YAngle;
                            foundHorizontal = true;
                            
                            horizontalRectangle[j] = y;
                        }
                        
                        if(processing.getValue() != processingSteps.showVerticalRects && selectedVertical != null){
                            rawImage.drawPolygon(selectedVertical, wpiVerticalColorProp, width.getValue());
                        }
                        if(processing.getValue() != processingSteps.showHorizontalRects){
                            rawImage.drawPolygons(horizontalRectangle, wpiHorizontalColorProp, width.getValue());
                        }
                    }
                }
                
                
                
                outputTable.putBoolean("Found horizontal target", foundHorizontal);
                outputTable.putNumber("Horizontal target horizontal angle", horizontalXAngle);
                outputTable.putNumber("Horizontal target vertical angle", horizontalYAngle);
                outputTable.putBoolean("Found vertical target", foundVertical);
                outputTable.putNumber("Vertical target horizontal angle", verticalXAngle);
                outputTable.putNumber("Vertical target vertical angle", verticalYAngle);
                outputTable.putNumber("Distance to wall in inches", verticalTargetDistance);
            }
        }
        long endTime = System.currentTimeMillis();
        outputTable.putNumber("Time taken in milliseconds", endTime - startTime);
        return rawImage;
    }
    
    public WPIBinaryImage findThresholds(WPIColorImage rawImage, Integer hueMin, Integer hueMax, Integer satMin, Integer satMax, Integer valMin, Integer valMax, Integer closings) {

        IplConvKernel morphKernel;
        
        morphKernel = IplConvKernel.create(3, 3, 1, 1, opencv_imgproc.CV_SHAPE_RECT, null);
        
        IplImage camIn = StormExtensions.getIplImage(rawImage);
        cvCvtColor(camIn, hsv, opencv_imgproc.CV_BGR2HSV);
        
        cvSplit(hsv, hueImg, satImg, valImg, null);
        
        opencv_imgproc.cvThreshold(hueImg, bin, hueMin - 1, 255, opencv_imgproc.CV_THRESH_BINARY);
        opencv_imgproc.cvThreshold(hueImg, temp, hueMax, 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        
        cvAnd(temp, bin, hueImg, null);
        
        opencv_imgproc.cvThreshold(satImg, bin, satMin - 1, 255, opencv_imgproc.CV_THRESH_BINARY);
        opencv_imgproc.cvThreshold(satImg, temp, satMax, 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        
        cvAnd(temp, bin, satImg, null);
                
        opencv_imgproc.cvThreshold(valImg, bin, valMin - 1, 255, opencv_imgproc.CV_THRESH_BINARY);
        opencv_imgproc.cvThreshold(valImg, temp, valMax, 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        
        cvAnd(temp, bin, bin, null);
        
        cvAnd(satImg, bin, bin, null);
        cvAnd(hueImg, bin, bin, null);
        
        opencv_imgproc.cvMorphologyEx(bin, bin, null, morphKernel, opencv_imgproc.CV_MOP_CLOSE, closings);
        
        morphKernel.deallocate();
        
        return StormExtensions.makeWPIBinaryImage(bin);
        
    }
    
    public WPIPolygon[] checkPolygons(WPIContour[] countours, int vertices) {
        
        //System.out.println("Got in well enough");
        ArrayList<WPIPolygon> polygons = new ArrayList<>();
        ArrayList<WPIPolygon> rectangles = new ArrayList<>();
        WPIPolygon [] rects;
        
        ArrayList<WPIPolygon> checkedPolygons = new ArrayList<>();
                
        for(WPIContour c:countours){
            
            double sideRatio = ((double)c.getHeight()/(double)c.getWidth());
            
            polygons.add(c.approxPolygon(percentAccRect.getValue()));
                
            for(int y = 0; y < polygons.size(); y++){
                
                WPIPolygon p = polygons.get(y);
                
                WPIPoint [] points;
                boolean orderChecks = true;
                if(!p.isConvex() || p.getNumVertices() != vertices){
                    continue;
                }
                
                points = p.getPoints();
                //System.out.println(vertices);

                for(int x = 0; x< vertices;x++){
                    isVertical[x] = checkVert(points[x], points[(x+1) % vertices]);
                }

                if(!(isVertical[0] && !isVertical[1] && isVertical[2] && !isVertical[3] || !isVertical[0] && isVertical[1] && !isVertical[2] && isVertical[3])){
                    orderChecks = false;
                    //System.out.println("Failed side order check");
                }else{
                    //System.out.println("passed side order check");
                }
                
                if(orderChecks){
                    checkedPolygons.add(p);
                }
            
            
            }
            
        }
        if(!checkedPolygons.isEmpty()){
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
        double averageSide0 = ((distanceFormula(points[0].getX(), points[0].getY(), points[1].getX(), points[1].getY())) + (distanceFormula(points[2].getX(), points[2].getY(), points[3].getX(), points[3].getY())))/2;
        double averageSide1 = ((distanceFormula(points[1].getX(), points[1].getY(), points[2].getX(), points[2].getY())) + (distanceFormula(points[3].getX(), points[3].getY(), points[0].getX(), points[0].getY())))/2;
        double aspectRatio;
        if(checkVert(points[0], points[1])){
            aspectRatio = averageSide1/averageSide0;    
        }else{
            aspectRatio = averageSide0/averageSide1;
        }
        return aspectRatio <= 1;
    }
    
    public boolean checkVert(WPIPoint x1, WPIPoint x2){
        //System.out.println(Math.abs(x2.getY()-x1.getY()) > (Math.abs(x2.getX()-x1.getX())));
        return (Math.abs(x2.getY()-x1.getY()) > (Math.abs(x2.getX()-x1.getX())));
    }
    
    public WPIPolygon[] findCircle(WPIContour[] contours, int vertices){
        ArrayList<WPIPolygon> circleChecked = new ArrayList<> ();
        for(WPIContour x: contours){       
            WPIPolygon poly = x.approxPolygon(percentAccCircle.getValue());
            double polygonArea = poly.getArea();
            double circumference = poly.getPerimeter();
            //System.out.println("");
            /*System.out.println("Circumference: " + circumference + "\n" + 
                    "Area: " + polygonArea + "\n" + 
                    "Comparison: " + (circumference * circumference)/(4 * Math.PI * polygonArea) + "\n" + 
                    "Width of ball: " + x.getWidth() + "\n" + 
                    "Height of ball: " + x.getHeight() + "\n" +
                    "Width of image: " + imageWidth + "\n" + 
                    "Height of image: " + imageHeight + "\n");*/
            
            if(poly.getNumVertices() < vertices){
                //System.out.println("Failed vertices check");
                continue;
            }
            if(((x.getWidth()*x.getHeight())/(imageHeight*imageWidth))*100.0 < screenPercentage.getValue()){
                //System.out.println("Failed percentage check");
                continue;
            }
        
            if(((circumference * circumference)/(4 * Math.PI * polygonArea) < ballPerimeterVArea.getValue())){
                circleChecked.add(poly);
            }
            
        }
        
        WPIPolygon[] polygons = new WPIPolygon[circleChecked.size()];
        for(int i = 0;i<polygons.length;i++){
            polygons[i] = circleChecked.get(i);
        }
        return polygons;
    }
    
    public static double distanceFormula(double x1, double y1, double x2, double y2){
        return (Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2)));
    }
       
    public static void main(String[] args){
        
        Storm2014CV cv = new Storm2014CV();
       
        Scanner scanner = new Scanner(System.in);
        if(args.length == 0){
            System.out.println("Put in filenames seperated by spaces");
            System.exit(0);
        }
        for (String filename : args) {
            System.out.println(filename + ": ");
               
            WPIColorImage rawImage;
               
            try {
                rawImage = new WPIColorImage(ImageIO.read(new File(filename)));
            } catch (IOException e) {
                System.err.println("Could not find file!");
                return;
            }
               
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
         CanvasFrame resultFrame = new CanvasFrame(title);
         resultFrame.showImage(newImage.getBufferedImage());
         newImage.deallocate();
    }
}
