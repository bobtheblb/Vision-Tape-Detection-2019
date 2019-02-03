// include the librealsense C++ header file

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.h>


// include OpenCV header file

#include <opencv2/opencv.hpp>
#include <iostream>
#include <librealsense2/rs_advanced_mode.hpp>
#include <math.h>
using namespace std;

using namespace cv;

#define MAXCONTOURS 50
#define IDEALSPACINGTOWIDTHRATIO 4.0
#define ALLOWABLESPACINGPERCENTDIFFERENCE 40
#define SCREENWIDTH 640
#define SCREENHEIGHT 480
#define CENTERSCREENX SCREENWIDTH/2

int main()

{
        int iLowH = 0;

        int iHighH = 74;



        int iLowS = 84; 

        int iHighS = 170;



        int iLowV = 1;

        int iHighV = 44;
        
       
    //Contruct a pipeline which abstracts the device

    rs2::pipeline pipe;
   rs2::config cfg;
cfg.enable_stream(RS2_STREAM_DEPTH);

    // Create a context object. This object owns the handles to all connected realsense devices.
    // The returned object should be released with rs2_delete_context(...)
    ////rs2_context* ctx = rs2_create_context(RS2_API_VERSION, &e);
  rs2::context ctx;
auto devices = ctx.query_devices();
size_t device_count = devices.size();
if (!device_count)
{
    cout <<"No device detected. Is it plugged in?\n";
    return EXIT_SUCCESS;
}

// Get the first connected device
//auto dev = devices[0];
//std::ifstream t("/home/bobtheblb/DuncansResources/demos/epicgamermoments/realsensesettings.json");
auto profile = pipe.start();
rs400::advanced_mode dev = profile.get_device();
//advanced_mode_dev.load_json(str);
// Enter advanced mode
   if (dev.is<rs400::advanced_mode>())
   {
       // Get the advanced mode functionality
       auto advanced_mode_dev = dev.as<rs400::advanced_mode>();

       // Load and configure .json file to device
       ifstream t("/home/bobtheblb/DuncansResources/demos/epicgamermoments/realsensesettings.json");
       string str((istreambuf_iterator<char>(t)), istreambuf_iterator<char>());
       advanced_mode_dev.load_json(str);
   }
   else
   {
       cout << "Current device doesn't support advanced-mode!\n";
       return EXIT_FAILURE;
   }
//std::string(i) = "High Density";
//auto sensor = profile.get_device().first<rs2::depth_sensor>();
//sensor.set_option(RS2_OPTION_VISUAL_PRESET, i);
 //pipe.start(cfg);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize

    rs2::frameset frames;

    for(int i = 0; i < 30; i++)

//   while (true) {

    {

        //Wait for all configured streams to produce a frame
        std::vector<rs2::sensor> sensors = dev.query_sensors();
        frames = pipe.wait_for_frames();

    }
     namedWindow("HSV Control", WINDOW_AUTOSIZE );
     namedWindow("Threshold Image", WINDOW_AUTOSIZE );
     namedWindow("Display Image", WINDOW_AUTOSIZE );   
    //Get each frame
 cvCreateTrackbar("LowH", "HSV Control", &iLowH, 179); //Hue (0 - 179)

 cvCreateTrackbar("HighH", "HSV Control", &iHighH, 179);



 cvCreateTrackbar("LowS", "HSV Control", &iLowS, 255); //Saturation (0 - 255)

 cvCreateTrackbar("HighS", "HSV Control", &iHighS, 255);



 cvCreateTrackbar("LowV", "HSV Control", &iLowV, 255); //Value (0 - 255)

 cvCreateTrackbar("HighV", "HSV Control", &iHighV, 255);
    Mat Thresholdimg;
    while (true) {
    Mat structuring_element = getStructuringElement(MORPH_RECT, Size(5, 5));

    //rs2::frame depth = data.get_depth_frame();
    frames = pipe.wait_for_frames();
    //depth = dec.process(depth); 


    rs2::frame color_frame = frames.get_color_frame();



    // Creating OpenCV Matrix from a color image

    Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
    
    Mat canny_result;
    Mat boundingbox = Mat::zeros(color.rows, color.cols, CV_8UC3);

    // Display in a GUI
   
    cvtColor(color, Thresholdimg, COLOR_BGR2HSV);
    
    inRange(color, Scalar(iLowH, iLowS, iLowV),Scalar(iHighH, iHighS, iHighV), Thresholdimg);
    
    imshow("Threshold Image", Thresholdimg);
    
    Canny(Thresholdimg, canny_result, 128, 128);
    
    std::vector<std::vector<Point>> contours;
    
    findContours(Thresholdimg, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    RotatedRect rotbox;
        // Find the largest rectangle
    float highestnum[(contours.size()/4)];
    
    Rect largest_rectangle;
   
    int contournum = 0;
    
    typedef struct
    {
        Point2f A;
        Point2f B;
        float Slope;
        bool pos0orneg1;
        float RectWidth;
        float RectCX;
        float RectCY;
        float LineCX;
        
    }LineEndpts; 

    typedef struct 
    {
        Point2f A;
        int ContourIndexLeft;
        int ContourIndexRight;   
        float TapePairCenterX;
    }ReflectiveTapePair;
    


    ReflectiveTapePair ReflectiveTapePairs[MAXCONTOURS];
    LineEndpts RectLongestLines[MAXCONTOURS]; 
    
    int CombinedReflectiveTapePair[MAXCONTOURS];
    cout << "Got to level 0!" << endl;
    
    int RectIndex = 0;
    

//***********************************************************
//LOOP FOR FINDING TAPE CONTOURS AND GATHERING INFO

    for (auto& contour : contours) {
        cout << "got to level 0.1!" << endl;
        float highestnums[contours.size()];
        cout << "got to level 0.25!" << endl;
        Rect current_rect = boundingRect(contour);
         cout << "got to level 0.5!" << endl;
         rectangle(boundingbox, current_rect, Scalar(0, 255, 0), 2);
        contournum = contournum + 1;
        rotbox = minAreaRect(contour);
        //boxpoints(rotbox);
        Point2f corners[4];
        rotbox.points(corners);
        //drawContours()
        //drawContours(boundingbox, rotbox.points, -1, Scalar(0, 0, 255));
        line(boundingbox, corners[0], corners[1], Scalar(0,255,255), 3, 8, 0);
        line(boundingbox, corners[1], corners[2], Scalar(0,255,255), 3, 8, 0);
        line(boundingbox, corners[2], corners[3], Scalar(0,255,255), 3, 8, 0);
        line(boundingbox, corners[3], corners[0], Scalar(0,255,255), 3, 8, 0);

        float DeltaXArray[4];
        float DeltaYArray[4];
        float lines[4];
         cout << "got to level 1!" << endl;

        DeltaXArray[0] = corners[1].x - corners[0].x;
        DeltaYArray[0] = corners[1].y - corners[0].y; 

        DeltaXArray[1] = corners[2].x - corners[1].x;
        DeltaYArray[1] = corners[2].y - corners[1].y; 

        DeltaXArray[2] = corners[3].x - corners[2].x;
        DeltaYArray[2] = corners[3].y - corners[2].y; 

        DeltaXArray[3] = corners[0].x - corners[3].x;
        DeltaYArray[3] = corners[0].y - corners[3].y; 

        lines[0] = sqrt(((DeltaXArray[0] * DeltaXArray[0]) + (DeltaYArray[0] * DeltaYArray[0])));
        lines[1] = sqrt(((DeltaXArray[1] * DeltaXArray[1]) + (DeltaYArray[1] * DeltaYArray[1])));
        lines[2] = sqrt(((DeltaXArray[2] * DeltaXArray[2]) + (DeltaYArray[2] * DeltaYArray[2])));
        lines[3] = sqrt(((DeltaXArray[3] * DeltaXArray[3]) + (DeltaYArray[3] * DeltaYArray[3])));

        
cout << "got to level 2!" << endl;

        
         if (RectIndex > MAXCONTOURS - 10)
            {
                RectIndex = 41;
            }
        
        RectLongestLines[RectIndex].RectCX = (corners[0].x + corners[1].x + corners[2].x + corners[3].x)/4.0;
        RectLongestLines[RectIndex].RectCY = (corners[0].y + corners[1].y + corners[2].y + corners[3].y)/4.0;
       

        int LongestLineIndex = 0;
        int ShortestLineIndex = 0;
        float HighNum = lines[0];
        float LowNum = lines[0];
        for(int i = 1; i <= 3; i++)
        {
            if (HighNum < lines[i])
            {
                HighNum = lines[i];
                
                LongestLineIndex = i;
            }
            if (LowNum > lines[i])
            {
                LowNum = lines[i];
                ShortestLineIndex = i;
            }
        }//for(int i = 1; i <= 3; i++)

        cout << "got to level 3!" << endl;
        RectLongestLines[RectIndex].A.x = corners[LongestLineIndex].x;
        RectLongestLines[RectIndex].A.y = corners[LongestLineIndex].y;

        if(LongestLineIndex == 3)
        {
         RectLongestLines[RectIndex].B.x = corners[0].x;  
         RectLongestLines[RectIndex].B.y = corners[0].y;
         //RectLongestLines[RectIndex];
        }
        else
        {
         RectLongestLines[RectIndex].B.x = corners[LongestLineIndex+1].x;
         RectLongestLines[RectIndex].B.y = corners[LongestLineIndex+1].y;
        }
        RectLongestLines[RectIndex].Slope = DeltaYArray[LongestLineIndex]/DeltaXArray[LongestLineIndex];

        if (RectLongestLines[RectIndex].Slope > 0)
        {
            RectLongestLines[RectIndex].pos0orneg1 = 0;
        }
        else 
        {
            RectLongestLines[RectIndex].pos0orneg1 = 1;
        }

        RectLongestLines[RectIndex].RectWidth = LowNum;



        cout << "got to level 4!" << endl;
        /*else
        {
          RectLongestLines[RectIndex].B.y = corners[LineIndex+1].y;
          
        }*/

        

        //float biggestnum[4] = {distanceone, distancetwo, distancethree, distancefour};
        //float highest = 0;
      
        /*for(int i = 0; i < contours.size() + 1; )
        {

        }
`  




        for(int i = 0; i < 5; i++)
    {
        if (biggestnum[i] > highest)
        {
            highest = biggestnum[i];
        }
    }
highestnums[contournum] = highest;
  */


RectIndex++;
}


//END OF LOOP FOR FINDING TAPE CONTOURS AND GATHERING INFO
//***********************************************************


    
    
    cout << "got to level 10!" << endl;
    cout << "RectIndex" << RectIndex << endl;

    int NumReflectivePairs = 0;

//*********************************************************** 
  //LOOP FOR MATCHING TAPE PAIRS
 
    if (RectIndex != 0)
    {
    for(int SlopeIndexA = 0; SlopeIndexA < RectIndex - 1; SlopeIndexA++) 
        {
           
            for(int SlopeIndexB = SlopeIndexA + 1; SlopeIndexB <= RectIndex; SlopeIndexB++)
            {
                 cout << "failed A slope" << RectLongestLines[SlopeIndexA].Slope << endl;
                 cout << "failed B slope" << RectLongestLines[SlopeIndexB].Slope << endl;   
                    
                    if (
                        (
                         signbit(RectLongestLines[SlopeIndexA].Slope) != 0 &&//negative
                         signbit(RectLongestLines[SlopeIndexB].Slope) == 0 &&
                         RectLongestLines[SlopeIndexA].A.x < RectLongestLines[SlopeIndexB].A.x
                        ) ||
                        (
                         signbit(RectLongestLines[SlopeIndexB].Slope) != 0 &&
                         signbit(RectLongestLines[SlopeIndexA].Slope) == 0 &&
                         RectLongestLines[SlopeIndexB].A.x < RectLongestLines[SlopeIndexA].A.x
                        ) )
                    {

                        float MeasuredTapeSpacing = abs(abs(RectLongestLines[SlopeIndexA].RectCX - RectLongestLines[SlopeIndexB].RectCX) - RectLongestLines[SlopeIndexA].RectWidth);

                        float ActualSpacingtoWidthRatio = MeasuredTapeSpacing/RectLongestLines[SlopeIndexA].RectWidth;    

                        float TapeSpacingRatioDifference = abs(IDEALSPACINGTOWIDTHRATIO - ActualSpacingtoWidthRatio);

                        float TapeSpacingRatioPercentDifference = (TapeSpacingRatioDifference/IDEALSPACINGTOWIDTHRATIO) * 100;
                        cout << "Got to level 100 " << endl;
                        cout << "Made it to end of Slope tests " << endl;
                        cout << "Left tape pos " << RectLongestLines[SlopeIndexA].A.x << endl;
                        cout << "Right tape pos " << RectLongestLines[SlopeIndexB].A.x << endl;
                        cout << "A tape slope " << RectLongestLines[SlopeIndexA].Slope << endl;
                        cout << "B tape slope " << RectLongestLines[SlopeIndexB].Slope << endl;
                        cout << "RectWidth " << MeasuredTapeSpacing/RectLongestLines[SlopeIndexA].RectWidth;
                        cout << "Measured spacing " << MeasuredTapeSpacing;
                        cout << "Measured ratio spacing to width " << ActualSpacingtoWidthRatio;                       
                            if (TapeSpacingRatioPercentDifference < ALLOWABLESPACINGPERCENTDIFFERENCE)
                            {
                                exit;
                        ReflectiveTapePairs[NumReflectivePairs].ContourIndexLeft = SlopeIndexA;

                        ReflectiveTapePairs[NumReflectivePairs].ContourIndexRight = SlopeIndexB;
                       
                        ReflectiveTapePairs[NumReflectivePairs].TapePairCenterX = ((RectLongestLines[SlopeIndexA].RectCX + RectLongestLines[SlopeIndexB].RectCX)/2);
                        NumReflectivePairs++;  
                        
                        cout << "Made it to end of spacing tests!!!" << endl;
                        
                        cout << "ContourIndexLeft" << ReflectiveTapePairs[NumReflectivePairs].ContourIndexLeft << endl;
                    
                        cout << "ContourIndexRight" << ReflectiveTapePairs[NumReflectivePairs].ContourIndexRight << endl;
                            }

                        
                    }//if (RectLongestLines[SlopeIndexA].Slope > 0)
                    else
                    {
                        cout << "A tape slope" << RectLongestLines[SlopeIndexA].Slope << endl;
                        cout << "B tape slope" << RectLongestLines[SlopeIndexB].Slope << endl;
                    }

            }//for(int SlopeIndexB = 0; SlopeIndexB <= RectIndex; SlopeIndexB++)


        }


        }//for(int SlopeIndexA = 0; SlopeIndexA <= RectIndex; SlopeIndexA++) 


    //END OF LOOP FOR MATCHING TAPE PAIRS
//***********************************************************
        int CenterPairIndex = 0;

        if (NumReflectivePairs != 0)
        {

        int RectCount = RectIndex;

        if (RectCount > MAXCONTOURS - 1)
            {
                RectCount = MAXCONTOURS;
            }
            cout << "Hi" << std::endl;
            cout << "RectCount" << (RectCount) << std::endl;
        if (RectCount == 0)
        {
            cout << "rEeEeEeEe" << endl;
        }
         else
        {
        cout << "got to level 15!" << endl;



        float ClosestPairDistance = ReflectiveTapePairs[0].TapePairCenterX;
        
//***********************************************************
//START LOOP FIND CLOSEST TO CENTER
        int RectIndex;

        for (RectIndex = 0; RectIndex < NumReflectivePairs; RectIndex++)
        {
            float TapePairDistanceFromCenter = abs(ReflectiveTapePairs[RectIndex].TapePairCenterX - CENTERSCREENX);
            if (ClosestPairDistance > TapePairDistanceFromCenter)
            {
                ClosestPairDistance = TapePairDistanceFromCenter;
                CenterPairIndex = RectIndex; 
            }
        }
        

//END OF LOOP FIND CLOSEST TO CENTER
//***********************************************************




        cout << "got to level 16!" << endl;
        // Declares the points
        Point2f leftpoint; 
        Point2f rightpoint;
        cout << "got to level 16.25" << endl;
        cout << "RectIndex" << RectIndex << endl;
        cout << "NumReflectivePairs" << NumReflectivePairs << endl;
        
        if(NumReflectivePairs != 0)
        {
            leftpoint.x = RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectCX;
            cout << "got to level 16.3" << endl;
            leftpoint.y = RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectCY;
            cout << "got to level 16.5!" << endl;
            cout << "RectIndex" << RectIndex << endl;
            rightpoint.x = RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectCX;
            cout << "got to level 16.74" << endl;
            rightpoint.y = RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectCY;
            
            cout << "got to level 17!" << endl;
            // Creates a line between center points of left and right tape from the tape pairs
            line(boundingbox, leftpoint, rightpoint, Scalar(0, 255, 255), 3, 8, 0);
        }
        
        } 

        cout << "got to level 18!" << endl;
    drawContours(boundingbox, contours, -1, Scalar(0, 0, 255));
        cout << "got to level 19!" << endl;
    rectangle(boundingbox, largest_rectangle, Scalar(0, 255, 0), 2);
    }

    imshow("bounding boxes", boundingbox);
    cout << "got to level 20" << endl;


    
    imshow("Canny Result", canny_result);
    cout << "got to level 21" << endl;
    imshow("Display Image", color);
   
    cout << "running" << endl;

     waitKey(25);

    //cout << "running" << endl;

}

    return 0;
}