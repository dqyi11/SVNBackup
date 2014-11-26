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


const char* keys =
{
	"{    c|withCamera          |false             |with camera    }"
	"{    v|  videoStreamAddress|  F:/data/ant.avi |video file path}"
	"{    1|                    |0                 |camera id      }"

};


static void help()
{
	cout << "\nThis is a demo that shows mean-shift based tracking\n"
			"You select a color objects such as your face and it tracks it.\n"
			"This reads from video camera (0 by default, or the camera number the user enters\n"
			"Usage: \n"
			"   ./MeanShift [camera number]\n";

	cout << "\n\nHot keys: \n"
			"\tESC - quit the program\n"
			"\tp - pause video\n"
			"To initialize tracking, select the object with mouse\n";
}


int main(int argc, char** argv) 
{

	//string videoStreamAddress = "F:/data/ant.avi";
	//string videoStreamAddress = "F:/data/stgeorge.avi";
	//string videoStreamAddress = "F:/data/PETS2000.avi";
	//string videoStreamAddress = "F:/data/PETS2001.avi";
	//string videoStreamAddress = "F:/data/Walk1.mpg";

	help();

	CommandLineParser parser(argc, argv, keys);

	bool withCam = parser.get<bool>("withCamera");
	string videoStreamAddress = parser.get<string>("v");
	int camNum = parser.get<int>("1");


	VideoCapture capture;
	Rect trackWindow;
	MeanShiftTracker meanshift;

	if ( withCam )
	{
		capture.open(camNum);
	} 
	else
	{
		capture.open(videoStreamAddress);
	}

	if( !capture.isOpened() )
	{
		help();
		cout << "***Could not initialize capturing...***\n";
		cout << "Current parameter's value: \n";
		parser.printParams();
		return EXIT_FAILURE;
	}


	namedWindow( "MeanShift Demo", 0 );
	setMouseCallback( "MeanShift Demo", onMouse, 0 );

	Mat frame;
	bool paused = false;


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


					meanshift.setInitialFrame(image);
					meanshift.setTarget_initial(trackWindow);
					meanshift.setTarget_current(trackWindow);

					meanshift.buildTargetModel(image, trackWindow);

				}

				// perform meanshift iterations
				if ( meanshift.run(image, 10) == EXIT_SUCCESS )
				{
					// draw rectangle around the tracked object
					trackWindow = meanshift.getTarget_current();

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
			paused = false;


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
