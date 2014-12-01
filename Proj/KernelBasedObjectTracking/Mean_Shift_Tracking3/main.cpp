#include <iostream>
#include <string>
#include <stdio.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "meanshift.h"


using namespace std; 
using namespace cv;


Mat image;

bool selectObject = false;
int trackObject = 0;
Point origin;
Rect selection;


static void onMouse( int event, int x, int y, int, void* )
{
	if( selectObject )
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);

		selection &= Rect(0, 0, image.cols, image.rows);
	}

	switch( event )
	{
	case CV_EVENT_LBUTTONDOWN:
		origin = Point(x,y);
		selection = Rect(x,y,0,0);
		selectObject = true;
		break;
	case CV_EVENT_LBUTTONUP:
		selectObject = false;
		if( selection.width > 0 && selection.height > 0 )
			trackObject = -1;
		break;
	}
}


int main(int argc, char** argv) 
{
    string video_filename = "tracking_video.avi";

	VideoCapture capture;
	Rect trackWindow;
	MeanShiftTracker meanshift;

    capture.open(video_filename);

	if( !capture.isOpened() )
	{
        cout << "Open " << video_filename << " failed" << endl;
		return EXIT_FAILURE;
	}

	namedWindow( "MeanShift Demo", 0 );
	setMouseCallback( "MeanShift Demo", onMouse, 0 );

	Mat frame;
	bool paused = false;
    Mat referenceImg;
    Rect referenceRect;
    Rect candidateRect;
    Mat referenceModel;

    int delay = 10;
	int fps = (int) capture.get(CV_CAP_PROP_FPS);
	if (fps != 0)
	{
		delay = 1000/fps;
	}


	for(;;)
	{
		if( !paused )
		{
			capture >> frame;
			if( frame.empty() )
				break;
		}

		frame.copyTo(image);

		if( !paused )
		{

			if( trackObject )
			{

				if( trackObject < 0 )
				{
					trackWindow = selection;
					trackObject = 1;

                    referenceImg = image;
                    referenceRect = trackWindow;
                    candidateRect = trackWindow;

                    referenceModel = meanshift.buildModel(image, trackWindow);
                    meanshift.setTargetModel(referenceModel);
				}

                // perform meanshift
                if(meanshift.findTarget(referenceImg, referenceRect, image, candidateRect))
                {
                    trackWindow = candidateRect;

                    if( trackWindow.area() <= 1 )
                    {
                        int cols = image.cols, rows = image.rows, r = (MIN(cols, rows) + 5)/6;
                        trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
                            trackWindow.x + r, trackWindow.y + r) &
                            Rect(0, 0, cols, rows);
                    }

                    rectangle(image,
                        Point(trackWindow.x, trackWindow.y),
                        Point(trackWindow.x+trackWindow.width,
                        trackWindow.y+trackWindow.height),
                        Scalar(0,0,255));
                }
			}
		}
		else if( trackObject < 0 )
        {
			paused = false;
        }


		if( selectObject && selection.width > 0 && selection.height > 0 )
		{
			Mat roi(image, selection);
			bitwise_not(roi, roi);
		}

		imshow( "MeanShift Demo", image );

		char c = (char)waitKey(delay);
		if( c == 27 )
			break;
		switch(c)
		{
		case 'p':
			paused = !paused;
			break;
		default:
			;
		}

	}

	return EXIT_SUCCESS;

}
