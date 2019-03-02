// include the librealsense C++ header file
#include <simple_socket_server.hpp>
#include <simple_socket.hpp>
#include <librealsense2/rs_advanced_mode.h>
#include <stdio.h>
#include <math.h>
// include OpenCV header file
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <librealsense2/rs_advanced_mode.hpp>
#include <math.h>
#include <librealsense2/rs.hpp>
//#include </home/bobtheblb/librealsense/wrappers/opencv/cv-helpers.hpp>
#include <histogram.hpp>
#include <opencv2/core.hpp>
#include <bgrd_frame_source.hpp>
#include <map>
#include <vector>
#include <numeric>
#include <string>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <sstream>
#include <stdlib.h>

using namespace std;

using namespace cv;

#define MAXCONTOURS 50
#define IDEALSPACINGTOWIDTHRATIO 4.0
#define ALLOWABLESPACINGPERCENTDIFFERENCE 60
#define SCREENWIDTH 1280
#define SCREENHEIGHT 720
#define CENTERSCREENX SCREENWIDTH/2
#define CENTERSCREENY SCREENHEIGHT/2
#define IDEALHEIGHTTOWIDTHRATIO 3
#define ALLOWABLEASPECTRATIODIFFERENCE 1.2
#define PI 3.14159
#define PORT 5060
#define MAXBUF 256


int main(int argc, char** argv)

{
        int iLowH = 0;

        int iHighH = 115;

        
        int iLowS = 121; 
        
        
        int ConnectionCount = 0;


        const size_t MaxBufferSize = 60;
        char Buffer[MaxBufferSize] = {0};
        
        SimpleSocket* servsock;

        servsock = new SimpleSocketServer(PORT);

        int iHighS = 239;
        //bool alive;

        

        /*
        float test = 10.112111;
        float FirstAngleToMove = (int)((test * 100) + 0.5);
        FirstAngleToMove = FirstAngleToMove / 100;
        cout << FirstAngleToMove << endl; */


        float FirstAngleToMove;
        float ControlPointDistanceFromCenter;
        float SecondAngleToMove;
        float LeftDepth;
        float RightDepth;

        std::cout << "Uh Hi" << endl;

        float iFirstAngleToMove;
        float iControlPointDistanceFromCenter;
        float iSecondAngleToMove;
        float iLeftDepth;
        float iRightDepth;

        struct sockaddr_in servaddr;

        struct sockaddr clientaddr;

        int clientlen = sizeof(clientaddr);

        int flags;

        char buffer_rec[512] = {7};

        int iLowV = 0;

        int iHighV = 122;
        
        char buffer_shutdown[MAXBUF] = "Goodnight";

        memset(&servaddr, '0', sizeof(servaddr));

        servaddr.sin_family = AF_INET;

        servaddr.sin_port = htons(5060);

        servaddr.sin_addr.s_addr = /*inet_addr("127.0.0.1");*/ INADDR_ANY;

        

        //servsock = socket(AF_INET, SOCK_STREAM, 0);

        

        int opt = 1;

        bool breakout = 0;
       
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream( rs2_stream::RS2_STREAM_DEPTH, SCREENWIDTH, SCREENHEIGHT, rs2_format::RS2_FORMAT_Z16,  30);
        
        cfg.enable_stream(rs2_stream::RS2_STREAM_COLOR, SCREENWIDTH, SCREENHEIGHT, rs2_format::RS2_FORMAT_BGR8, 30);
        auto profile = pipe.start(cfg);
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
//rs2::device dev = pipe.get_active_profile().get_device();
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

    if (servsock < 0)

    {

        std::cout << "Failed to create socket" << std::endl;

    }

    
    
     
    std::cout << "Made it here yee yee" << endl;
    //char Buffer[MAXBUF];
    //Contruct a pipeline which abstracts the device


//std::string(i) = "High Density";
auto sensor = profile.get_device().first<rs2::depth_sensor>();
//rs2::frame depth = data.get_depth_frame();
//auto depth_mat = depth_frame_to_meters(profile, depth_frame);
 //pipe.start(cfg);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
//   auto sensor = profile.get_device().first<rs2::depth_sensor>();
    rs2::frameset frames;

    for(int i = 0; i < 10; i++)

//   while (true) {

    {

        cout << "sup " << endl;
        //Wait for all configured streams to produce a frame
        std::vector<rs2::sensor> sensors = dev.query_sensors();
        frames = pipe.wait_for_frames();

    }
 
    Mat Thresholdimg;
    //rs2::spatial_filter spat;
    //spat.set_option(RS2_OPTION_HOLES_FILL, 5);
    rs2::align align(RS2_STREAM_COLOR);



    //auto leftdepth_hist = Histogram<unsigned short>(min_hist, max_hist);
    //auto rightdepth_hist = Histogram<unsigned short>(min_hist, max_hist);
    

    

    ssize_t ret;

    //auto leftdepthregionhistogram = Histogram<unsigned short>(200, 1000);
    //auto rightdepthregionhistogram = Histogram<unsigned short>(200, 1000);
    
    //dev.enable_stream( rs::stream::depth, SCREENWIDTH, SCREENHEIGHT, rs::format::z16, 60);
    while (true) {
    servsock->re_establish();

    
    //cout << "left type " << endl;
    //cout << "right type " << &rightdepthregionhistogram << endl;

    auto data = pipe.wait_for_frames();
    frames = pipe.wait_for_frames();
    //rs2::align align_to(RS2_STREAM_COLOR);
    //depth = dec.process(depth); 
    //data = align_to.process(data);
    //rs2::frame depth = data.get_depth_frame();
    //data = align_to.process(data);
    
    //rs2::frame color_frame = frames.get_color_frame();
    //auto depth_frame = data.get_depth_frame();

    rs2::align align_to(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(frames);
    rs2::video_frame color_frame = aligned_frames.first(RS2_STREAM_COLOR);
    rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();
    
    
    //auto depth = depth_frame_to_meters(pipe, depth_frame);
    //cout << depth << endl;
    //depth = depth * 1000;
    //auto depth_mat = depth_frame_to_meters(pipe, depth_frame);
    //rs2::frame depth_frame = frames.get_depth_frame();

   // int depthwidth = depth_frame.as<rs2::video_frame>().get_width();
    //int depthheight = depth_frame.as<rs2::video_frame>().get_width();
    // Creating OpenCV Matrix from a color image
    Mat color(Size(SCREENWIDTH, SCREENHEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
    Mat depth(Size(SCREENWIDTH, SCREENHEIGHT), CV_16UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
    
    //imshow("Depth framee ", depth);
    
    

    //imshow("left mat", LeftDepthRegionMat);
    //imshow("right mat", RightDepthRegionMat);
    //imshow("depth mat", depth);
 
   


    //mat depth(Size(depthwidth, depthheight), CV_8UC3, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
    Mat canny_result;
    Mat boundingbox = Mat::zeros(color.rows, color.cols, CV_8UC3);
    Mat AutoAlignImage = Mat::zeros(800, 800, CV_8UC3);
    // Display in a GUI
  
    cvtColor(color, Thresholdimg, COLOR_BGR2HSV);
    
    inRange(color, Scalar(iLowH, iLowS, iLowV),Scalar(iHighH, iHighS, iHighV), Thresholdimg);
    
    //********************************************************************
    //erode and dilate to get rid of small blobs in the background
    Mat structuring_element = getStructuringElement(MORPH_RECT, Size(4, 4));
    erode(Thresholdimg, Thresholdimg, structuring_element);
    dilate(Thresholdimg, Thresholdimg, structuring_element);

    //**********************************************************************
    
    
    
    Point2f LeftRobot, RightRobot;

    LeftRobot.x = 350;
    LeftRobot.y = 600;
    RightRobot.x = 450;
    RightRobot.y = 600;

    line(AutoAlignImage, LeftRobot, RightRobot, Scalar(0, 255, 255), 3, 8, 0);
    
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
        bool IsStrip;
        float RectLength;
        Point2f RotRectPointA;
        Point2f RotRectPointB;
        Point2f RotRectPointC;
        Point2f RotRectPointD;
        Point2f OutermostPoint;
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
    //cout << "Got to level 0!" << endl;
    
    int RectIndex = 0;
    

//***********************************************************
//LOOP FOR FINDING TAPE CONTOURS AND GATHERING input

    for (auto& contour : contours) {
        

        //cout << "got to level 0.1!" << endl;
        float highestnums[contours.size()];
        //cout << "got to level 0.25!" << endl;
        Rect current_rect = boundingRect(contour);
        // cout << "got to level 0.5!" << endl;
        




         rectangle(boundingbox, current_rect, Scalar(0, 255, 0), 2);
        




        contournum = contournum + 1;
        rotbox = minAreaRect(contour);
        //boxpoints(rotbox);
        Point2f corners[4];
        rotbox.points(corners);
        //drawContours()
        //drawContours(boundingbox, rotbox.points, -1, Scalar(0, 0, 255));

        

        float DeltaXArray[4];
        float DeltaYArray[4];
        float lines[4];
        // cout << "got to level 1!" << endl;

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
        
        RectLongestLines[RectIndex].RotRectPointA.x = corners[0].x;
        RectLongestLines[RectIndex].RotRectPointA.y = corners[0].y;
        RectLongestLines[RectIndex].RotRectPointB.x = corners[1].x;
        RectLongestLines[RectIndex].RotRectPointB.y = corners[1].y;
        RectLongestLines[RectIndex].RotRectPointC.x = corners[2].x;
        RectLongestLines[RectIndex].RotRectPointC.y = corners[2].y;
        RectLongestLines[RectIndex].RotRectPointD.x = corners[3].x;
        RectLongestLines[RectIndex].RotRectPointD.y = corners[3].y;

        
        
        
        //cout << "got to level 2!" << endl;

        
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

       // cout << "got to level 3!" << endl;
        RectLongestLines[RectIndex].A.x = corners[LongestLineIndex].x;
        RectLongestLines[RectIndex].A.y = corners[LongestLineIndex].y;

        RectLongestLines[RectIndex].RectWidth = LowNum;

        RectLongestLines[RectIndex].RectLength = HighNum;




        float HeightWidthRatio = RectLongestLines[RectIndex].RectLength / RectLongestLines[RectIndex].RectWidth;
        float HeightWidthRatioDifference = abs(HeightWidthRatio - IDEALHEIGHTTOWIDTHRATIO);
        
        if (HeightWidthRatioDifference < ALLOWABLEASPECTRATIODIFFERENCE)
        {
            RectLongestLines[RectIndex].IsStrip = 1;
        }
        else
        {
            RectLongestLines[RectIndex].IsStrip = 0;
        }
        
        
        if (RectLongestLines[RectIndex].IsStrip == 1)
        {
            line(boundingbox, corners[0], corners[1], Scalar(0,255,255), 3, 8, 0);
            line(boundingbox, corners[1], corners[2], Scalar(0,255,255), 3, 8, 0);
            line(boundingbox, corners[2], corners[3], Scalar(0,255,255), 3, 8, 0);
            line(boundingbox, corners[3], corners[0], Scalar(0,255,255), 3, 8, 0);
        }    

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

       

        



        //cout << "got to level 4!" << endl;
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




//END OF LOOP FOR FINDING TAPE CONTOURS AND GATHERING input
//***********************************************************


    
    
    //cout << "got to level 10!" << endl;
    //cout << "RectIndex" << RectIndex << endl;

    int NumReflectivePairs = 0;

//*********************************************************** 
  //LOOP FOR MATCHING TAPE PAIRS
    
    float RealPairDistance;

    if (RectIndex != 0)
    {
    for(int SlopeIndexA = 0; SlopeIndexA < RectIndex - 1; SlopeIndexA++) 
        {
           
            for(int SlopeIndexB = SlopeIndexA + 1; SlopeIndexB <= RectIndex; SlopeIndexB++)
            {
                 if (RectLongestLines[SlopeIndexA].IsStrip == 0 || RectLongestLines[SlopeIndexB].IsStrip == 0)    
                        {
                          //  cout << "Failed aspect ratio test" << endl;
                        }   

                 //cout << "failed A slope" << RectLongestLines[SlopeIndexA].Slope << endl;
                 //cout << "failed B slope" << RectLongestLines[SlopeIndexB].Slope << endl;   
                    
                if (RectLongestLines[SlopeIndexA].IsStrip == 1 && RectLongestLines[SlopeIndexB].IsStrip == 1)
                {



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
                        /*cout << "Got to level 100 " << endl;
                        cout << "Made it to end of Slope tests " << endl;
                        cout << "Left tape pos " << RectLongestLines[SlopeIndexA].A.x << endl;
                        cout << "Right tape pos " << RectLongestLines[SlopeIndexB].A.x << endl;
                        cout << "A tape slope " << RectLongestLines[SlopeIndexA].Slope << endl;
                        cout << "B tape slope " << RectLongestLines[SlopeIndexB].Slope << endl;
                        cout << "RectWidth " << MeasuredTapeSpacing/RectLongestLines[SlopeIndexA].RectWidth;
                        cout << "Measured spacing " << MeasuredTapeSpacing;
                        cout << "Measured ratio spacing to width " << ActualSpacingtoWidthRatio;                       
                        */
                        
                            if (TapeSpacingRatioPercentDifference < ALLOWABLESPACINGPERCENTDIFFERENCE)
                            {
                                exit;
                        if (RectLongestLines[SlopeIndexA].A.x < RectLongestLines[SlopeIndexB].A.x)
                        {
                            ReflectiveTapePairs[NumReflectivePairs].ContourIndexLeft = SlopeIndexA;

                            ReflectiveTapePairs[NumReflectivePairs].ContourIndexRight = SlopeIndexB;
                        } 
                        else
                        {
                            ReflectiveTapePairs[NumReflectivePairs].ContourIndexLeft = SlopeIndexB;

                            ReflectiveTapePairs[NumReflectivePairs].ContourIndexRight = SlopeIndexA;
                        }   
                       

                       
                        ReflectiveTapePairs[NumReflectivePairs].TapePairCenterX = ((RectLongestLines[SlopeIndexA].RectCX + RectLongestLines[SlopeIndexB].RectCX)/2);
                        NumReflectivePairs++;  
                        
                       // cout << "Made it to end of spacing tests!!!" << endl;
                        
                        //cout << "ContourIndexLeft" << ReflectiveTapePairs[NumReflectivePairs].ContourIndexLeft << endl;
                    
                        //cout << "ContourIndexRight" << ReflectiveTapePairs[NumReflectivePairs].ContourIndexRight << endl;
                            }

                        
                    }//if (RectLongestLines[SlopeIndexA].Slope > 0)
                    else
                    {
                      //  cout << "A tape slope" << RectLongestLines[SlopeIndexA].Slope << endl;
                      //  cout << "B tape slope" << RectLongestLines[SlopeIndexB].Slope << endl;
                    }

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
           // cout << "Hi" << std::endl;
           // cout << "RectCount" << (RectCount) << std::endl;
        if (RectCount == 0)
        {
            //cout << "rEeEeEeEe" << endl;
        }
         else
        {
        //cout << "got to level 15!" << endl;



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
                int CenterPairIndex = RectIndex; 
                RealPairDistance = ReflectiveTapePairs[RectIndex].TapePairCenterX - CENTERSCREENX;
            }
        }
        

//END OF LOOP FIND CLOSEST TO CENTER
//***********************************************************

        float LeftRectCornersX[4] = {RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RotRectPointA.x, RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RotRectPointB.x, RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RotRectPointC.x, RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RotRectPointD.x};
        float RightRectCornersX[4] = {RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RotRectPointA.x, RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RotRectPointB.x, RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RotRectPointC.x, RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RotRectPointD.x};
        
        RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].OutermostPoint.x = RightRectCornersX[0];
        
        int LeftOutermostCornerIndex = 0;
        int RightOutermostCornerIndex = 0;

//***********************************************************
//START LOOP FIND OUTERMOST POINTS
       RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].OutermostPoint.x = LeftRectCornersX[0];
       RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].OutermostPoint.x = RightRectCornersX[0];


       int LeftOutermostCornerx = LeftRectCornersX[0] ;

       for (int CornerIndex = 0; CornerIndex < 3; CornerIndex++)
       {

            int CornerXPlus1 = LeftRectCornersX[CornerIndex+1] ;
            

            if (CornerXPlus1 <= LeftOutermostCornerx)
            {
                LeftOutermostCornerx = CornerXPlus1 ;
                LeftOutermostCornerIndex = CornerIndex + 1 ;
            }
             // cout << "Left +1 point :D " << CornerXPlus1 << endl;
             // cout << "Left outermost Corner X " << LeftOutermostCornerx << endl;
             // cout << "Left Corner Index " << LeftOutermostCornerIndex << endl;
       }
      // cout << "Final leftmost x point" << LeftRectCornersX[LeftOutermostCornerIndex];
       
      int RightOutermostCornerx = RightRectCornersX[0] ;

       for (int CornerIndex = 0; CornerIndex < 3; CornerIndex++)
       {

            int CornerXPlus1 = RightRectCornersX[CornerIndex+1] ;
            

            if (CornerXPlus1 >= RightOutermostCornerx)
            {
                RightOutermostCornerx = CornerXPlus1 ;
                RightOutermostCornerIndex = CornerIndex + 1 ;
            }
             // cout << "Right +1 point :D " << CornerXPlus1 << endl;
             // cout << "Right outermost Corner X " << RightOutermostCornerx << endl;
             // cout << "Right Corner Index " << RightOutermostCornerIndex << endl;
       }
       //cout << "Final rightmost x point" << RightRectCornersX[RightOutermostCornerIndex];
//END OF LOOP FIND OUTERMOST POINTS
//***********************************************************
        int LeftRectTopY = RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectCY - (RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectLength / 1.5);
        int LeftRectBottomY = RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectCY + (RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectLength / 1.5);
        int RightRectTopY = RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectCY - (RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectLength / 1.5);
        int RightRectBottomY = RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectCY + (RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectLength / 1.5);

        float LeftDepthRegionMean;
        float RightDepthRegionMean;

        float CenterDepth;

        float AverageRectWidth = (RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectWidth + RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectWidth) / 2;
                        
        float ExpectedTapeWidth = 50.8;

        float ExpectedTapeWidthToMeasured = ExpectedTapeWidth / AverageRectWidth;
                        
        float RealTapePairDistanceFromCenter = RealPairDistance * (ExpectedTapeWidthToMeasured);

        

        if  ((
            (LeftRectCornersX[LeftOutermostCornerIndex] - RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectWidth / 1) > 20 
            && (RightRectCornersX[RightOutermostCornerIndex] + RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectWidth / 8) + (RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectWidth / 1) < SCREENWIDTH - 20
            )
            
            &&

            
            ((LeftRectTopY > 10 && RightRectTopY > 10) && (LeftRectBottomY < SCREENHEIGHT - 10 && RightRectBottomY < SCREENHEIGHT - 10))
            ) 

            {   
           
                        Rect RightDepthRegion(RightRectCornersX[RightOutermostCornerIndex] + RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectWidth / 8, 
                            RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectCY - (RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectLength / 1.9),                   
                             (RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectWidth / 1),
                             (RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectLength * 1));
                    
                        Rect LeftDepthRegion(LeftRectCornersX[LeftOutermostCornerIndex] - RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectWidth / 0.9,
                            RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectCY - (RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectLength / 1.9),
                             (RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectWidth / 1), 
                             (RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectLength * 1));

                        //LeftDepthRegion = LeftDepthRegion  & Rect(0, 0, depth_mat.cols, depth_mat.rows);

                        //RightDepthRegion = RightDepthRegion  & Rect(0, 0, depth_mat.cols, depth_mat.rows);                                                                                                                 

                        rectangle(boundingbox, LeftDepthRegion, cv::Scalar(0, 255, 0));

                        rectangle(boundingbox, RightDepthRegion, cv::Scalar(0, 0, 255));

                        //rectangle(color, LeftDepthRegion, cv::Scalar(0, 255, 0));

                        //rectangle(color, RightDepthRegion, cv::Scalar(0, 255, 0));
                       
                        //cout << LeftDepthRegion << endl;
                        //cout << RightDepthRegion << endl;
                        //cout << "parameters for left rect" << RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectLength << " " << RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectWidth << endl;
                        //cout << "parameters for right rect" << RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectLength << " " << RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectWidth << endl;
                        
                        
                        Mat LeftDepthRegionMat = depth(LeftDepthRegion);
                        Mat RightDepthRegionMat = depth(RightDepthRegion);
                        
                        //cout << "Left mat           " << LeftDepthRegionMat << endl;
                        //cout << "Right mat" << RightDepthRegionMat << endl;
                        //cv::Mat LeftDepthRegionMat = frameset.depth(LeftDepthRegion);

                        //cv::Mat RightDepthRegionMat = frameset.depth(RightDepthRegion);
                       // imshow("left region depth pls", depth);
                        //leftdepthregionhistogram.clear();
                        //rightdepthregionhistogram.clear();

                        //leftdepthregionhistogram.insert_image(LeftDepthRegionMat);
                        //rightdepthregionhistogram.insert_image(RightDepthRegionMat);
                       
                        //Scalar leftdepthmeanregion = mean(LeftDepthRegionMat);
                        //Scalar rightdepthmeanregion = mean(RightDepthRegionMat);

                        //cout << leftdepthmeanregion << "Left mean " << endl;
                        //cout << rightdepthmeanregion << "Right mean" << endl;
                        
                    

                        if (NumReflectivePairs != 0)
                        {
                        int LeftRowsIndex;
                        int LeftColumnsIndex;
                        long int LeftDepthTotal = 0;
                        int LeftValidNumberCount = 0;
                        long int RightDepthTotal = 0;
                        int RightValidNumberCount = 0;
                        int RightRowsIndex;
                        int RightColumnsIndex;

                        for (LeftRowsIndex = 1; LeftRowsIndex < LeftDepthRegionMat.rows; LeftRowsIndex++)
                        {
                            for (LeftColumnsIndex = 1; LeftColumnsIndex < LeftDepthRegionMat.cols; LeftColumnsIndex++)
                            {
                                if (LeftDepthRegionMat.at<unsigned short>(LeftRowsIndex, LeftColumnsIndex) > 400 && LeftDepthRegionMat.at<unsigned short>(LeftRowsIndex, LeftColumnsIndex) < 5000)
                                {
                                    LeftDepthTotal = LeftDepthTotal + LeftDepthRegionMat.at<unsigned short>(LeftRowsIndex, LeftColumnsIndex);
                                    LeftValidNumberCount++;
                                }
                            }
                        }
                       
                        

                        for (RightRowsIndex = 1; RightRowsIndex < RightDepthRegionMat.rows; RightRowsIndex++)
                        {
                            for (RightColumnsIndex = 1; RightColumnsIndex < RightDepthRegionMat.cols; RightColumnsIndex++)
                            {
                                if (RightDepthRegionMat.at<unsigned short>(RightRowsIndex, RightColumnsIndex) > 400 && RightDepthRegionMat.at<unsigned short>(RightRowsIndex, RightColumnsIndex) < 5000)
                                {
                                    RightDepthTotal = RightDepthTotal + RightDepthRegionMat.at<unsigned short>(RightRowsIndex, RightColumnsIndex);
                                    RightValidNumberCount++;
                                    //cout << "Made it to right if statements " << endl;
                                }
                            }
                        }
                        //cout << "we made it to the right side bois" << endl;
                        
                        cout << LeftValidNumberCount << endl;
                        cout << RightValidNumberCount << endl;
                        //cout << "Left Depth Region Mat " << LeftDepthRegionMat << endl;
                        //cout << RightDepthRegionMat << endl;
                       




                        if (RightValidNumberCount != 0 && LeftValidNumberCount != 0)
                        {
                        

                        LeftDepthRegionMean = (float)LeftDepthTotal / (float)LeftValidNumberCount;
                        RightDepthRegionMean = (float)RightDepthTotal / (float)RightValidNumberCount;
                        
                        cout << "Right region mean pls work!!!!!!! " << RightDepthRegionMean << endl;
                        cout << "Left region mean pls work!!!!!!! " << LeftDepthRegionMean << endl;
                        //float LeftDepth = 500;
                        //float RightDepth = 600;
                        float LeftDepth = LeftDepthRegionMean;
                        float RightDepth = RightDepthRegionMean;
                        float DepthDifference = (RightDepth - LeftDepth);
                        float CenterDepth = (LeftDepth + RightDepth) / 2;

                        float VisionTargetDistanceFromCenter = RealTapePairDistanceFromCenter;

                        float ControlPointDistanceFromVisionTarget = 1041.4;
                        float VisionTargetLength = 381;
                        float VisionTargetSlope = (180 / PI) * asin(DepthDifference / VisionTargetLength);

                        float ControlPointDistanceXFromVisionTarget = abs(ControlPointDistanceFromVisionTarget * sin(VisionTargetSlope * (PI / 180)));
                        float ControlPointDistanceYFromVisionTarget = abs(ControlPointDistanceFromVisionTarget * cos(VisionTargetSlope * (PI / 180)));
                        cout << VisionTargetSlope << " Slope of Target" << endl;
                        cout << ControlPointDistanceYFromVisionTarget << " Y Control Point Distance Y From Target" << endl;
                        cout << ControlPointDistanceXFromVisionTarget << " X Control Point Distance X From Target" << endl;

                        float ControlPointDistanceXFromCenter;

                        if (VisionTargetSlope > 0)
                        {
                            ControlPointDistanceXFromCenter = VisionTargetDistanceFromCenter + ControlPointDistanceXFromVisionTarget;
                        }

                        if (VisionTargetSlope < 0)
                        {
                            ControlPointDistanceXFromCenter = VisionTargetDistanceFromCenter - ControlPointDistanceXFromVisionTarget;
                        }
                        float ControlPointDistanceYFromCenter = CenterDepth - ControlPointDistanceYFromVisionTarget;

                        float FirstAngleToMove = (180 / PI) * atan2((double)ControlPointDistanceXFromCenter, (double)ControlPointDistanceYFromCenter);
                        cout << FirstAngleToMove << " First Angle To Move" << endl;
                        cout << ControlPointDistanceXFromCenter << " Control Point Distance X From Center" << endl;
                        cout << ControlPointDistanceYFromCenter << " Control Point Distance Y From Center" << endl;

                        float ControlPointDistanceFromCenter = sqrt(pow(ControlPointDistanceXFromCenter, 2) + pow(ControlPointDistanceYFromCenter, 2));
                        cout << ControlPointDistanceFromCenter << " Control Point Euclidean Distance From Center" << endl;

                        float SecondAngleToMove = VisionTargetSlope - FirstAngleToMove;
                        cout << SecondAngleToMove << " Second Angle To Move" << endl;
                        cout << VisionTargetDistanceFromCenter << " VIsion Target Real Distance From Camera Center in Millimeters please wooork";
                        
                        
                        }
                        
                        }

                    
                
            
            }      
        
        
        //line(AutoAlignImage, Point(400, 600), Point(400 + (ControlPointDistanceXFromCenter / 5), 600 - (ControlPointDistanceYFromCenter / 5)), Scalar(0, 255, 255), 3, 8, 0);
        Point2f leftpoint; 
        Point2f rightpoint; 

        if(NumReflectivePairs != 0)
        {
            leftpoint.x = RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectCX;
            //cout << "got to level 16.3" << endl;
            leftpoint.y = RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexLeft].RectCY;
            //cout << "got to level 16.5!" << endl;
            //cout << "RectIndex" << RectIndex << endl;
            rightpoint.x = RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectCX;
            //cout << "got to level 16.74" << endl;
            rightpoint.y = RectLongestLines[ReflectiveTapePairs[CenterPairIndex].ContourIndexRight].RectCY;
            
            //cout << "got to level 17!" << endl;
            // Creates a line between center points of left and right tape from the tape pairs
            line(boundingbox, leftpoint, rightpoint, Scalar(0, 255, 255), 3, 8, 0);
        }
        
        } 

        //cout << "got to level 18!" << endl;
    drawContours(boundingbox, contours, -1, Scalar(0, 0, 255));
       // cout << "got to level 19!" << endl;
    rectangle(boundingbox, largest_rectangle, Scalar(0, 255, 0), 2);
    }


    cout << "got to level 21" << endl;
    
   
    cout << "running" << endl;

     waitKey(25);

    //cout << "running" << endl;
                        
                          FirstAngleToMove = 420.6924;
                          ControlPointDistanceFromCenter = 69.42069;
                          SecondAngleToMove = 555.5555;
                          LeftDepth = 666.6;
                          RightDepth = 999.99999;




                          iFirstAngleToMove = (int)((FirstAngleToMove * 100) + 0.5);
                          iFirstAngleToMove = (float)(iFirstAngleToMove / 100);  
                          
                          iControlPointDistanceFromCenter = (int)((ControlPointDistanceFromCenter * 100) + 0.5);
                          iControlPointDistanceFromCenter = (float)iControlPointDistanceFromCenter / 100;

                          iSecondAngleToMove = (int)((SecondAngleToMove * 100) + 0.5);
                          iSecondAngleToMove = (float)iSecondAngleToMove / 100;

                          iLeftDepth = (int)((LeftDepth * 100) + 0.5);
                          iLeftDepth = (float)iLeftDepth / 100;

                          iRightDepth = (int)((RightDepth * 100) + 0.5);
                          iRightDepth = (float)iRightDepth / 100;
          
                        if(NumReflectivePairs == 0)
                        {
                            /*iFirstAngleToMove = 0;
                            iControlPointDistanceFromCenter = 0;
                            iSecondAngleToMove = 0;
                            iLeftDepth = 0;
                            iRightDepth = 0;*/
                        }


                        char Buffer[MAXBUF] = {0};
                        cout << iFirstAngleToMove << " " << iSecondAngleToMove << endl;
                         /* iFirstAngleToMove = 1;
                          iControlPointDistanceFromCenter = 2;
                          iSecondAngleToMove = 3;
                          iLeftDepth = 4;
                          iRightDepth = 5; */
                        /*
            strcat(Buffer," A ") ;

            strcat(Buffer,to_string(iFirstAngleToMove).c_str()) ;

            strcat(Buffer," B ") ;

            strcat(Buffer,to_string(iControlPointDistanceFromCenter).c_str()) ;

            strcat(Buffer," C ") ;

            strcat(Buffer,to_string(iSecondAngleToMove).c_str()) ;

            strcat(Buffer," L ") ;

            strcat(Buffer, to_string(iLeftDepth).c_str()) ;

            strcat(Buffer," R ") ;

            strcat(Buffer, to_string(iRightDepth).c_str()) ;

            strcat(Buffer," Z ") ;

            strcat(Buffer, to_string(ConnectionCount).c_str()) ;

            strcat(Buffer,"\0") ;       */   
            ConnectionCount++;
            sprintf(Buffer, "A %f B %f C %f L %f R %f D %i\0", iFirstAngleToMove, iControlPointDistanceFromCenter, iSecondAngleToMove, iLeftDepth, iRightDepth, ConnectionCount);

            cout << "This is the buffer " << Buffer << std::endl;           
                        //int writing = sock->write(input, strlen(input));
       
        if(servsock->write(Buffer, MaxBufferSize))
        {
            cout << "Sent Info " << Buffer << endl;
            Buffer[MaxBufferSize] = {0};
        }
        else
        {
            cout << "Error writing info to client" << endl;
        }

        
                        
    


    if( (char)27 == waitKey(1))
    {
        
        
        return -1;
        
    }

    if(argc > 0)
    {
        namedWindow("HSV Control", WINDOW_AUTOSIZE );
        namedWindow("Threshold Image", WINDOW_AUTOSIZE );
     //namedWindow("Display Image", WINDOW_AUTOSIZE );   
    
    //Get each frame


         cvCreateTrackbar("LowH", "HSV Control", &iLowH, 179); //Hue (0 - 179)

         cvCreateTrackbar("HighH", "HSV Control", &iHighH, 179);



         cvCreateTrackbar("LowS", "HSV Control", &iLowS, 255); //Saturation (0 - 255)

         cvCreateTrackbar("HighS", "HSV Control", &iHighS, 255);



         cvCreateTrackbar("LowV", "HSV Control", &iLowV, 255); //FirstAngleToMove (0 - 255)

         cvCreateTrackbar("HighV", "HSV Control", &iHighV, 255);
        imshow("bounding boxes", boundingbox);
   // cout << "got to level 20" << endl;
        imshow("Threshold Image", Thresholdimg);
        imshow("Auto align image", AutoAlignImage);
    
        imshow("Canny Result", canny_result);
        imshow("Display Image", color);

    }
}
    
    return 0;
}