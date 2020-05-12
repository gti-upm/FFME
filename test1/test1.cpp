//-------------------------------------------------------------------------
// test1.cpp 
//-------------------------------------------------------------------------
// Description: tests the FFME class for image motion estimation based on 
// features.
//-------------------------------------------------------------------------
// Requirements: 
// -OpenCV library.
// -FFME library.
//-------------------------------------------------------------------------
// Author: Carlos Roberto del Blanco Adán, 
//         Grupo de Tratamiento de Imágenes, GTI SSR, Madrid
//		   cda@gti.ssr.upm.es
// Date: 2008
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
//Images.
const char* filename1 = "1.jpeg";
const char* filename2 = "2.jpeg";
//***********************************************************************************************//


//*****************************************Parameters********************************************//
bool LUT = true; //Used Look-Up Tables(true) or functions(false). LUTs makes computations faster.
int noMaxPoints = 1000; //Maximum number of singular points.
//***********************************************************************************************//


int _tmain(int argc, _TCHAR* argv[])
{
	//--Declarations--//.
	int i;
	IplImage* img1_U81C = 0;
	IplImage* img2_U81C = 0;
	IplImage* img3_S161C = 0;
	IplImage* img4_S161C = 0;
	IplImage* img5_U81C = 0;
	IplImage* img6_U81C = 0;
	IplImage* img7_32F1C = 0;
	IplImage* img8_U81C = 0;
	IplImage* img9_U81C = 0;
	IplImage* img10_U81C = 0;
	IplImage* img11_U81C = 0;
	IplImage* img12_U83C = 0;
	IplImage* img13_U83C = 0;
	IplImage* img14_U83C = 0;
	FFME ffme;
	CvPoint2D32f* singPoints1;
	CvPoint2D32f* singPoints2;
	CvPoint2D32f* gradPoints;
	CvPoint2D32f* cornerPoints;
	int noGradPoints;
	int noCornerPoints;
	int noSingPoints1;
	int noSingPoints2;
	float** descriptors1;
	float** descriptors2;
	int lengthDesc;
	int noCorr;
	CvPoint2D32f** correspondences;


	//--Read images--//.
	img1_U81C = cvLoadImage(filename1, CV_LOAD_IMAGE_GRAYSCALE);
	if(!img1_U81C)
	{
		printf("The image can not be opened: %s\n", filename1);
	}
	img2_U81C = cvLoadImage(filename2, CV_LOAD_IMAGE_GRAYSCALE);
	if(!img2_U81C)
	{
		printf("The image can not be opened: %s\n", filename2);
	}
	//View images.
	cvNamedWindow( "Output1", 1 );
	cvNamedWindow( "Output2", 1 );
	cvShowImage( "Output1", img1_U81C );
	cvShowImage( "Output2", img2_U81C );
	cvWaitKey(0); //Waits user interaction and meanwhile executes pending events.


	//--Initialization--//.
	//Singular point locations.
	singPoints1 = (CvPoint2D32f*)cvAlloc(noMaxPoints * sizeof(CvPoint2D32f));
	singPoints2 = (CvPoint2D32f*)cvAlloc(noMaxPoints * sizeof(CvPoint2D32f));
	//FFME class initialization.
	ffme.iniFFME(img1_U81C->width, img1_U81C->height, img1_U81C->origin, noMaxPoints);
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
	img5_U81C = cvCreateImage(cvGetSize(img1_U81C), IPL_DEPTH_8U, 1);
	img5_U81C->origin = img1_U81C->origin; 
	img6_U81C = cvCreateImage(cvGetSize(img1_U81C), IPL_DEPTH_8U, 1);
	img6_U81C->origin = img1_U81C->origin; 
	img8_U81C = cvCreateImage(cvGetSize(img1_U81C), IPL_DEPTH_8U, 1);
	img8_U81C->origin = img1_U81C->origin; 
	img9_U81C = cvCreateImage(cvGetSize(img1_U81C), IPL_DEPTH_8U, 1);
	img9_U81C->origin = img1_U81C->origin;
	img10_U81C = cvCreateImage(cvGetSize(img1_U81C), IPL_DEPTH_8U, 1);
	img10_U81C->origin = img1_U81C->origin;
	img11_U81C = cvCreateImage(cvGetSize(img1_U81C), IPL_DEPTH_8U, 1);
	img11_U81C->origin = img1_U81C->origin;
	img12_U83C = cvCreateImage(cvGetSize(img1_U81C), IPL_DEPTH_8U, 3);
	img12_U83C->origin = img1_U81C->origin;
	img13_U83C = cvCreateImage(cvGetSize(img1_U81C), IPL_DEPTH_8U, 3);
	img13_U83C->origin = img1_U81C->origin;
	img14_U83C = cvCreateImage(cvGetSize(img1_U81C), IPL_DEPTH_8U, 3);
	img14_U83C->origin = img1_U81C->origin;

	
	if(!LUT)
	{
		//Singular point detection for image 1 based on functions.
		ffme.singPtoDetFunc(img1_U81C, singPoints1, &noSingPoints1);
	}
	else
	{
		//Singular point detection for image 1 based on LUTs.
		ffme.singPtoDetLut(img1_U81C, singPoints1, &noSingPoints1);
	}
	//View intermidiate and final results of the point detection.
	cvNamedWindow( "x gradient", 1 );
	ffme.getHorGradient(&img3_S161C);
	scaleXX1CtoU81C(img3_S161C, img5_U81C); //Scales the intensity values to fit
	cvShowImage( "x gradient", img5_U81C);
	cvNamedWindow( "y gradient", 1 );
	ffme.getVerGradient(&img4_S161C);
	scaleXX1CtoU81C(img4_S161C, img6_U81C); //Scales the intensity values to fit
	cvShowImage( "y gradient", img6_U81C);
	cvWaitKey(0); 
	cvNamedWindow( "Gradient magnitud", 1 );
	ffme.getGradMag(&img7_32F1C);
	scaleXX1CtoU81C(img7_32F1C, img8_U81C); //Scales the intensity values to fit
	cvShowImage( "Gradient magnitud", img8_U81C);
	cvWaitKey(0);
	cvNamedWindow( "Gradient restriction points", 1 );
	ffme.getNoGradPtos(&noGradPoints);
	ffme.getGradPtos(&gradPoints);
	drawPtosBW(img9_U81C, gradPoints, noGradPoints);
	cvShowImage( "Gradient restriction points", img9_U81C);
	cvWaitKey(0);
	cvNamedWindow( "Cornerness restriction points", 1 );
	ffme.getNoCornerPtos(&noCornerPoints);
	ffme.getCornerPtos(&cornerPoints);
	drawPtosBW(img10_U81C, cornerPoints, noCornerPoints);
	cvShowImage( "Cornerness restriction points", img10_U81C);
	cvWaitKey(0);
	cvNamedWindow( "Non-maximal supression and final singular point detection", 1 );
	drawPtosBW(img11_U81C, singPoints1, noSingPoints1);
	cvShowImage( "Non-maximal supression and final singular point detection", img11_U81C);
	cvWaitKey(0);
	cvNamedWindow( "Singular points on an image 1", 1 );
	cvMerge( img1_U81C, img1_U81C, img1_U81C, NULL, img13_U83C );
	drawPtosIm(img13_U83C, singPoints1, noSingPoints1);
	cvShowImage( "Singular points on an image 1", img13_U83C);
	cvWaitKey(0);
	
	
	
	if(!LUT)
	{
		//Singular point description for image 1 based on functions.
		ffme.singPtoDescFunc(singPoints1, noSingPoints1, descriptors1, true);
	}
	else
	{
		//Singular point description for image 1 based on LUTs.
		ffme.singPtoDescLut(singPoints1, noSingPoints1, descriptors1, true);
	}
	//Shows the first descriptor.
	printVectorFloat(descriptors1[0], ffme.m_widthArrayHist * ffme.m_widthArrayHist * ffme.m_noBinsOriHist, ffme.m_noBinsOriHist);
	cvWaitKey(0);


	//Singular point detection for image 2.
	ffme.m_threshGradMag = (int)ffme.m_threshGradMag * 0.9f; //Threshold hysteresis.
	ffme.m_threshHarris = (float)ffme.m_threshHarris * 1.1f; //Threshold hysteresis.
	if(!LUT)
	{
		//Singular point detection for image 1 based on functions.
		ffme.singPtoDetFunc(img2_U81C, singPoints2, &noSingPoints2);
	}
	else
	{
		//Singular point detection for image 1 based on LUTs.
		ffme.singPtoDetLut(img2_U81C, singPoints2, &noSingPoints2);
	}
	//View intermidiate and final results of the point detection.
	cvNamedWindow( "x gradient", 1 );
	ffme.getHorGradient(&img3_S161C);
	scaleXX1CtoU81C(img3_S161C, img5_U81C); //Scales the intensity values to fit
	cvShowImage( "x gradient", img5_U81C);
	cvNamedWindow( "y gradient", 1 );
	ffme.getVerGradient(&img4_S161C);
	scaleXX1CtoU81C(img4_S161C, img6_U81C); //Scales the intensity values to fit
	cvShowImage( "y gradient", img6_U81C);
	cvWaitKey(0); 
	cvNamedWindow( "Gradient magnitud", 1 );
	ffme.getGradMag(&img7_32F1C);
	scaleXX1CtoU81C(img7_32F1C, img8_U81C); //Scales the intensity values to fit
	cvShowImage( "Gradient magnitud", img8_U81C);
	cvWaitKey(0);
	cvNamedWindow( "Gradient restriction points", 1 );
	ffme.getNoGradPtos(&noGradPoints);
	ffme.getGradPtos(&gradPoints);
	drawPtosBW(img9_U81C, gradPoints, noGradPoints);
	cvShowImage( "Gradient restriction points", img9_U81C);
	cvWaitKey(0);
	cvNamedWindow( "Cornerness restriction points", 1 );
	ffme.getNoCornerPtos(&noCornerPoints);
	ffme.getCornerPtos(&cornerPoints);
	drawPtosBW(img10_U81C, cornerPoints, noCornerPoints);
	cvShowImage( "Cornerness restriction points", img10_U81C);
	cvWaitKey(0);
	cvNamedWindow( "Non-maximal supression and final singular point detection", 1 );
	drawPtosBW(img11_U81C, singPoints2, noSingPoints2);
	cvShowImage( "Non-maximal supression and final singular point detection", img11_U81C);
	cvWaitKey(0);
	cvNamedWindow( "Singular points on an image 2", 1 );
	cvMerge(img2_U81C, img2_U81C, img2_U81C, NULL, img14_U83C);
	drawPtosIm(img14_U83C, singPoints2, noSingPoints2);
	cvShowImage( "Singular points on an image 2", img14_U83C);
	cvWaitKey(0);
	

	//Singular point description for image 2.
	if(!LUT)
	{
		//Singular point description for image 1 based on functions.
		ffme.singPtoDescFunc(singPoints2, noSingPoints2, descriptors2, true);
	}
	else
	{
		//Singular point description for image 1 based on LUTs.
		ffme.singPtoDescLut(singPoints2, noSingPoints2, descriptors2, true);
	}
	//Shows the first descriptor.
	printVectorFloat(descriptors2[0], ffme.m_widthArrayHist * ffme.m_widthArrayHist * ffme.m_noBinsOriHist, ffme.m_noBinsOriHist);
	cvWaitKey(0);


	//Matching of singular points. Test if the function obtains correspondences using the same image.
	ffme.matchSingPtos(singPoints1, noSingPoints1, descriptors1, singPoints1, noSingPoints1, descriptors1, correspondences, &noCorr);
	printf("\nNumber of Corr: %d\n\n", noCorr);
	printCorr(correspondences, noCorr);
	cvMerge(img1_U81C, img1_U81C, img1_U81C, NULL, img12_U83C );
	drawCorr(img12_U83C, correspondences, noCorr);
	cvNamedWindow( "Correspondences as MVF", 1 );
	cvShowImage( "Correspondences as MVF", img12_U83C);
	cvWaitKey(0);


	//Matching of singular points. 
	ffme.matchSingPtos(singPoints1, noSingPoints1, descriptors1, singPoints2, noSingPoints2, descriptors2, correspondences, &noCorr);
	printf("\nNumber of Corr: %d\n\n", noCorr);
	printCorr(correspondences, noCorr);
	cvMerge(img2_U81C, img2_U81C, img2_U81C, NULL, img12_U83C);
	drawCorr(img12_U83C, correspondences, noCorr);
	cvNamedWindow( "Correspondences as MVF", 1 );
	cvShowImage( "Correspondences as MVF", img12_U83C);
	cvWaitKey(0);

	//Matching of singular points. Shows all correspondences without second nearest neighbor restriction.
	ffme.m_threshRatSecBest = 1;
	ffme.matchSingPtos(singPoints1, noSingPoints1, descriptors1, singPoints2, noSingPoints2, descriptors2, correspondences, &noCorr);
	printf("\nNumber of Corr: %d\n\n", noCorr);
	printCorr(correspondences, noCorr);
	cvMerge(img2_U81C, img2_U81C, img2_U81C, NULL, img12_U83C);
	drawCorr(img12_U83C, correspondences, noCorr);
	cvNamedWindow( "Correspondences as MVF without restriction", 1 );
	cvShowImage( "Correspondences as MVF without restriction", img12_U83C);
	cvWaitKey(0);
	
	//*************************************Release memory********************************************//
	//Images.
	cvReleaseImage(&img1_U81C);
	cvReleaseImage(&img2_U81C);
	cvReleaseImage(&img5_U81C);
	cvReleaseImage(&img6_U81C);
	cvReleaseImage(&img8_U81C);
	cvReleaseImage(&img9_U81C);
	cvReleaseImage(&img10_U81C);
	cvReleaseImage(&img11_U81C);
	cvReleaseImage(&img12_U83C);
	cvReleaseImage(&img13_U83C);
	cvReleaseImage(&img14_U83C);

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

	//Windows.
	cvDestroyWindow( "Output1" );
	cvDestroyWindow( "Output2" );
	cvDestroyWindow( "x gradient" );
	cvDestroyWindow( "y gradient" );
	cvDestroyWindow( "Gradient magnitud" );
	cvDestroyWindow( "Gradient restriction points" );
	cvDestroyWindow( "Cornerness restriction points" );
	cvDestroyWindow( "Non-maximal supression and final singular point detection" );
	cvDestroyWindow( "Correspondences as MVF" );
	cvDestroyWindow( "Singular points on an image 1" );
	cvDestroyWindow( "Singular points on an image 2" );
	cvDestroyWindow( "Correspondences as MVF without restriction" );
	//***********************************************************************************************//

    return 0;
}

