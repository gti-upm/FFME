//-------------------------------------------------------------------------
// demo1.cpp 
//-------------------------------------------------------------------------
// Description: tests the FFME class computing the sparse motion vector 
// field of one video sequence.
//-------------------------------------------------------------------------
// Requirements: 
// -OpenCV library.
// -FFME library.
//-------------------------------------------------------------------------
// Author: Carlos Roberto del Blanco Adán, 
//         Grupo de Tratamiento de Imágenes, GTI SSR, Madrid
//		   cda@gti.ssr.upm.es
// Date: 2009
// Version 1.0
//-------------------------------------------------------------------------

#include "stdafx.h"

//OpenCV dependencies.
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"

//Motion estimation class.
#include "FFME.h"


//*****************************************Input*************************************************//
//Video.
const char* filename1 = "marcavial.avi";
//const char* filename1 = "optical_flow_input.avi";
//***********************************************************************************************//


//*****************************************Parameters********************************************//
bool LUT = true; //Used Look-Up Tables(true) or functions(false). LUTs makes computations faster.
int noMaxPoints = 10000; //Maximum number of singular points.
//***********************************************************************************************//


int _tmain(int argc, _TCHAR* argv[])
{
	//--Declarations--//.
	int i;
	IplImage* img1_U83C = 0;
	IplImage* img2_U81C = 0;
	IplImage* img3_U83C = 0;
    FFME ffme;
	CvPoint2D32f* singPoints1;
	CvPoint2D32f* singPoints2;
	CvPoint2D32f* tmpPoints;
	int noSingPoints1;
	int noSingPoints2;
	float** descriptors1;
	float** descriptors2;
	float** tmpDesc;
	int lengthDesc;
	int noCorr;
	CvPoint2D32f** correspondences;
	CvCapture* capture;
	int heigth;
	int width;
	int fps;
	int noFrames;
	int first;
	int last;
	CvVideoWriter *writer = 0; //To write videos.
	int isColor = 1;
	double timeMeasured;


	//--Open video file--//
	capture = cvCreateFileCapture(filename1);
	img1_U83C = cvQueryFrame(capture); //Captures one frame to reaf the its properties.
	heigth = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
	width = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
	fps = (int) cvGetCaptureProperty(capture, CV_CAP_PROP_FPS);
	noFrames = (int) cvGetCaptureProperty(capture,  CV_CAP_PROP_FRAME_COUNT);


	//--Inicialización de la escritura de vídeo avi--//
	writer = cvCreateVideoWriter("out.avi", CV_FOURCC('M','J','P','G'), fps, cvSize(width,heigth), isColor);


	//--Initialization--//.
	//Singular point locations.
	singPoints1 = (CvPoint2D32f*)cvAlloc(noMaxPoints * sizeof(CvPoint2D32f));
	singPoints2 = (CvPoint2D32f*)cvAlloc(noMaxPoints * sizeof(CvPoint2D32f));
	//FFME class initialization.
	ffme.iniFFME(img1_U83C->width, img1_U83C->height, img1_U83C->origin, noMaxPoints);
	//Singular point description.
	lengthDesc = ffme.m_widthArrayHist*ffme.m_widthArrayHist*ffme.m_noBinsOriHist; //Length of descriptor vector.
	descriptors1 = (float**)cvAlloc(noMaxPoints * sizeof(float*));
	for(i=0;i<noMaxPoints;i++)
	{
		descriptors1[i] = (float*)cvAlloc(lengthDesc * sizeof(float));
	}
	descriptors2 = (float**)cvAlloc(noMaxPoints * sizeof(float*));
	for(i=0;i<noMaxPoints;i++)
	{
		descriptors2[i] = (float*)cvAlloc(lengthDesc * sizeof(float));
	}
	//Correspondences.
	correspondences = (CvPoint2D32f**)cvAlloc(noMaxPoints * sizeof(CvPoint2D32f*));
	for(i=0;i<noMaxPoints;i++)
	{
		correspondences[i] = (CvPoint2D32f*)cvAlloc(2 * sizeof(CvPoint2D32f));
	}

	//Allocate memory for results.
	img2_U81C = cvCreateImage(cvGetSize(img1_U83C), IPL_DEPTH_8U, 1);
	img2_U81C->origin = img1_U83C->origin; 
	img3_U83C = cvCreateImage(cvGetSize(img1_U83C), IPL_DEPTH_8U, 3);
	img3_U83C->origin = img1_U83C->origin; 
	

	//Processing loop.
	first = 1;
	last = noFrames;
	for(i=first;i<last;i++)
	{
		//Store one frame in the internal memory.
		if(!cvGrabFrame(capture))
		{              
		  printf("Could not grab a frame\n\7");
		  exit(-1);
		}
		//Get a pointer to the frame in the internal buffer. The image must no be modified.
		img1_U83C = cvRetrieveFrame(capture);
		cvCvtColor( img1_U83C, img2_U81C, CV_BGR2GRAY );

		if(i == first) //Initialization.
		{
			//Singular point detection.
			ffme.singPtoDetLut(img2_U81C, singPoints1, &noSingPoints1);
			//Singular point description.
			ffme.singPtoDescLut(singPoints1, noSingPoints1, descriptors1, true);

			//Update buffers.
			tmpPoints = singPoints2; //Exchange pointers.
			singPoints2 = singPoints1; 
			singPoints1 = tmpPoints;
			noSingPoints2 = noSingPoints1;
			tmpDesc = descriptors2; //Exchange pointers.
			descriptors2 = descriptors1; 
			descriptors1 = tmpDesc;
		}
		else
		{
			//Starts time counter.
			timeMeasured = (double)cvGetTickCount();

			//Singular point detection.
			ffme.singPtoDetLut(img2_U81C, singPoints1, &noSingPoints1);
			//Singular point description.
			ffme.singPtoDescLut(singPoints1, noSingPoints1, descriptors1, true);
			//Matching of singular points. 
			ffme.m_radMaxSearch = 32;
			ffme.matchSingPtos(singPoints2, noSingPoints2, descriptors2, singPoints1, noSingPoints1, descriptors1, correspondences, &noCorr);

			//Stops time counter.
			timeMeasured = (double)cvGetTickCount() - timeMeasured;
			printf("Measured time matching: %.1f\n", timeMeasured/(cvGetTickFrequency()*1000.));

			//Show motion vector field.
			cvMerge(img2_U81C, img2_U81C, img2_U81C, NULL, img3_U83C);
			drawCorr(img3_U83C, correspondences, noCorr);
			//Add a frame to the output video.
			cvWriteFrame(writer, img3_U83C);
			cvNamedWindow( "MVF", 1 );
			cvShowImage( "MVF", img3_U83C);
			printf("\nNumber of Corr: %d\n\n", noCorr);
			cvWaitKey(1);

			//Update buffers.
			tmpPoints = singPoints2; //Exchange pointers.
			singPoints2 = singPoints1; 
			singPoints1 = tmpPoints;
			noSingPoints2 = noSingPoints1;
			tmpDesc = descriptors2; //Exchange pointers.
			descriptors2 = descriptors1; 
			descriptors1 = tmpDesc;
		}
	}

	
	//*************************************Release memory********************************************//
	//Images.
	//img1_U83C is managed by cvCaptured object.
	cvReleaseImage(&img2_U81C);
	cvReleaseImage(&img3_U83C);

	//Data.
	cvFree(&singPoints1);
	cvFree(&singPoints2);
	for(i=0;i<noMaxPoints;i++)
	{
		cvFree(descriptors1+i);
	}
	cvFree(&descriptors1);
	for(i=0;i<noMaxPoints;i++)
	{
		cvFree(descriptors2+i);
	}
	cvFree(&descriptors2);
	for(i=0;i<noMaxPoints;i++)
	{
		cvFree(correspondences+i);
	}
	cvFree(&correspondences);

	//Capture object.
	cvReleaseCapture(&capture);

	//Windows.
	cvDestroyWindow( "MVF" );

	//Release video writer.
	cvReleaseVideoWriter(&writer);

	//***********************************************************************************************//

    return 0;
}

