// g++ stereo_calibration.cpp `pkg-config --cflags --libs opencv` -lpthread -O2 -Wall -o stereo_calibration
// run: 
// ./stereo_calibration 10 6 3
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>

using namespace cv;
using namespace std;

char Data[512]; // array to be used for storing data
FILE * pFile = NULL; // pointer to a file

int main(int argc, char* argv[])
{
    int numBoards = atoi(argv[1]);
    int board_w = atoi(argv[2]);
    int board_h = atoi(argv[3]);

    Size board_sz = Size(board_w, board_h);
    int board_n = board_w*board_h;

    vector<vector<Point3f> > object_points;
    vector<vector<Point2f> > imagePoints1, imagePoints2;
    vector<Point2f> corners1, corners2;

    vector<Point3f> obj;
    for (int j=0; j<board_n; j++)
    {
        obj.push_back(Point3f(j/board_w, j%board_w, 0.0f));
    }

    Mat img1, img2, gray1, gray2;
		namedWindow( "image1", 0 ); namedWindow( "image2", 0 );
		cvMoveWindow("image1", 0, 0);cvMoveWindow("image2", 100, 0);
    VideoCapture cap1 = VideoCapture(1);
    VideoCapture cap2 = VideoCapture(2);

    int success = 0; char k = 0;
    bool found1 = false, found2 = false;

		printf("\n\nPress spacebar to save corners\n");
    while (success < numBoards)
    {
        cap1 >> img1;
        cap2 >> img2;
        /*resize(img1, img1, Size(320, 280));
        resize(img2, img2, Size(320, 280));// */
        cvtColor(img1, gray1, CV_BGR2GRAY);
        cvtColor(img2, gray2, CV_BGR2GRAY);

        found1 = findChessboardCorners(img1, board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        found2 = findChessboardCorners(img2, board_sz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

        if (found1)
        {
            cornerSubPix(gray1, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray1, board_sz, corners1, found1);
        }

        if (found2)
        {
            cornerSubPix(gray2, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray2, board_sz, corners2, found2);
        }
        
        imshow("image1", gray1);
        imshow("image2", gray2);

        k = (char)waitKey(2);
        
				if (found1 && found2)
        {
					
        }
        if (k == 27)
        {
            exit(1);
        }
        if (k == ' ' && found1 !=0 && found2 != 0)
        {
            imagePoints1.push_back(corners1);
            imagePoints2.push_back(corners2);
            object_points.push_back(obj);
            printf ("Corners stored\n");
            success++; 

            if (success >= numBoards)
            {
                break;
            }
        }
    }

    destroyAllWindows();
    printf("Starting Calibration\n");
    Mat K1 = Mat(3, 3, CV_64FC1); // intrinsic param
    Mat K2 = Mat(3, 3, CV_64FC1); // intrinsic param
    Mat D1, D2; // distortions
    Mat R, T, E, F; // rotation, translation, essential. fundamental matrices
		Mat Pl, Pr; // left 3x3 and right 3x1 parts of projection matrix 2
		double P1[3][4] = {{0.0}}, P2[3][4] = {{0.0}}; // projection matrices

    stereoCalibrate(object_points, imagePoints1, imagePoints2, 
                    K1, D1, K2, D2, img1.size(), R, T, E, F,
                    cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5), 
                    CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);

		// compute projection matrices
		Pl = K2 * R; Pr = K2 * T;
		P1[0][0]=K1.at<double>(0,0); P1[0][1]=K1.at<double>(0,1); P1[0][2]=K1.at<double>(0,2); P1[0][3]=0;
		P1[1][0]=K1.at<double>(1,0); P1[1][1]=K1.at<double>(1,1); P1[1][2]=K1.at<double>(1,2); P1[1][3]=0;
		P1[2][0]=K1.at<double>(2,0); P1[2][1]=K1.at<double>(2,1); P1[2][2]=K1.at<double>(2,2); P1[2][3]=0;
		P2[0][0]=Pl.at<double>(0,0); P2[0][1]=Pl.at<double>(0,1); P2[0][2]=Pl.at<double>(0,2); P2[0][3]=Pr.at<double>(0);
    P2[1][0]=Pl.at<double>(1,0); P2[1][1]=Pl.at<double>(1,1); P2[1][2]=Pl.at<double>(1,2); P2[1][3]=Pr.at<double>(1);
    P2[2][0]=Pl.at<double>(2,0); P2[2][1]=Pl.at<double>(2,1); P2[2][2]=Pl.at<double>(2,2); P2[2][3]=Pr.at<double>(2);
 
    FileStorage fs1("mystereocalib.yml", FileStorage::WRITE);
    fs1 << "Intrinsic parameters camera 1 K1" << K1;
    fs1 << "Intrinsic parameters camera 1 K2" << K2;
		fs1 << "Multiplication of K2 x R" << Pl;
		fs1 << "Multiplication of K2 x T" << Pr;
    fs1 << "Rotation from camera 2 to camera 1 R" << R;
    fs1 << "Translation from camera 2 to camera 1 T" << T;
    fs1 << "Essential matrix E" << E;
    fs1 << "Fundamental matrix F" << F;
    
		printf("Done Calibration\n");

		// save calibration parameters
		pFile = fopen("projection_matrix_1.txt","w"); 
		memset( &Data, 0, sizeof(Data) );
		sprintf( Data, "%g %g %g %g %g %g %g %g %g %g %g %g", P1[0][0],P1[0][1],P1[0][2],P1[0][3],P1[1][0],P1[1][1],P1[1][2],P1[1][3],P1[2][0],P1[2][1],P1[2][2],P1[2][3]);
		fputs(Data, pFile); fclose (pFile);
		pFile = fopen("projection_matrix_2.txt","w"); 
    memset( &Data, 0, sizeof(Data) );
		sprintf( Data, "%g %g %g %g %g %g %g %g %g %g %g %g", P2[0][0],P2[0][1],P2[0][2],P2[0][3],P2[1][0],P2[1][1],P2[1][2],P2[1][3],P2[2][0],P2[2][1],P2[2][2],P2[2][3]);
		fputs(Data, pFile); fclose (pFile);

		// print parameters
		printf("\nProjection matrices:\n\n");
		printf("Camera 1:\n%g %g %g %g\n%g %g %g %g\n%g %g %g %g\n\n",P1[0][0],P1[0][1],P1[0][2],P1[0][3],P1[1][0],P1[1][1],P1[1][2],P1[1][3],P1[2][0],P1[2][1],P1[2][2],P1[2][3]);
    printf("Camera 2:\n%g %g %g %g\n%g %g %g %g\n%g %g %g %g\n\n",P2[0][0],P2[0][1],P2[0][2],P2[0][3],P2[1][0],P2[1][1],P2[1][2],P2[1][3],P2[2][0],P2[2][1],P2[2][2],P2[2][3]);

    cap1.release();
    cap2.release();

    return(0);
}

