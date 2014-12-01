#include <QApplication>
#include <QDebug>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "targetdetector.h"

cv::Rect box,next_box;
bool drawing_box = false;
bool selected = false;
int frame_count = 0;

void create_mouse_callback(int event,int x,int y,int flag,void* param)
{
    cv::Mat *image = (cv::Mat*) param;
    switch( event )
    {
        case CV_EVENT_MOUSEMOVE:
            if( drawing_box ){
                box.width = x-box.x;
                box.height = y-box.y;
            }
            break;

        case CV_EVENT_LBUTTONDOWN:
            drawing_box = true;
            box = cv::Rect( x, y, 0, 0 );
            break;

        case CV_EVENT_LBUTTONUP:
            drawing_box = false;
            if( box.width < 0 ){
                box.x += box.width;
                box.width *= -1;
            }
            if( box.height < 0 ){
                box.y += box.height;
                box.height *= -1;
            }
            cv::rectangle(*image,box,cv::Scalar(0),2);
            selected = true;
            break;
    }

}

int main(int argc, char *argv[])
{
    cv::Mat orig_img,temp_img,curr_img, target_model,target_candidate, weight;
    float dist = 0.0;

    TargetDetector detector;

    cv::VideoCapture start_capture;
    start_capture = cv::VideoCapture("tracking_video.avi");

    for(int i = 0; i<5; i++)
        start_capture.read(orig_img);

    cv::namedWindow("original image");

    temp_img = orig_img.clone();

    cv::setMouseCallback("original image",create_mouse_callback,(void*) &orig_img);

    cv::imshow("original image",orig_img);

    while(selected == false)
    {
        cv::Mat temp;

        temp_img.copyTo(temp);

        if( drawing_box )
            cv::rectangle( temp, box,cv::Scalar(0),2);


        cv::imshow("original image", temp );

        if( cv::waitKey( 15 )==27 )
            break;
    }

    std::cout << "\n Selected Box rect X: " << box.x << " Y:" << box.y << " W: " << box.width << " H: " << box.height << std::endl;

    if(box.width%2==0)
        box.width++;
    if(box.height%2==0)
        box.height++;


    cv::waitKey(0);

    while(1)
    {
        std::cout << "\n In Loop Box rect X: " << box.x << " Y:" << box.y << " W: " << box.width << " H: " << box.height << std::endl;

        if(!start_capture.read(curr_img))
            break;

        //start_capture.read(orig_img);
        //start_capture.read(orig_img);

        frame_count++;

        std::cout << "\n Frame count " << frame_count << std::endl;


        if(dist < 0.6 && frame_count > 10)
        {
            //target_model = target_candidate.clone();
            frame_count = 0;
            //std::cout << "gp";
        }

        next_box = detector.findTarget(orig_img, box, curr_img);


        std::cout << "\n Detected box X:" << next_box.x << " Y:" << next_box.y << " W:" << next_box.width << " H:" << next_box.height << std::endl;

        cv::rectangle(curr_img,next_box,cv::Scalar(0));

        cv::imshow("Tracking",curr_img);

        cv::waitKey(5);

        box = next_box;


    }

    cv::waitKey(0);

    return 0;
}
