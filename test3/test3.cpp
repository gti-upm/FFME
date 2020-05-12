//-------------------------------------------------------------------------
// test3.cpp 
//-------------------------------------------------------------------------
// Description: tests the FFME class for image motion estimation based on
// features. It's evaluated the number of correspondences as a function of 
// the noise. Also, the computation time is measured.
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


//--Function declaration--//
// Warping the image using a pure translation transformation with bilinear interpolation.
void transImg(IplImage* img1_U81C, IplImage* img2_U81C, float transHor, float transVer, CvMat* mat1_32F1C);
// Warping the image using a pure rotation transformation with bilinear interpolation.
void rotImg(IplImage* img1_U81C, IplImage* img2_U81C, float angle, CvPoint2D32f center, CvMat* mat1_32F1C);
// Computes the number of true correspondences between two array of points given the ground truth transformation.
void evalCorr(CvPoint2D32f* ptos1, CvPoint2D32f* ptos2, int noPtos, CvMat* mat1_32F1C, float dist, int* noTrueCorr);
// Euclidean distance between points.
float euclDistPtos(CvPoint2D32f* pto1, CvPoint2D32f* pto2);


//*****************************************Input*************************************************//
//Image.
const char* filename1 = "1.jpeg";
//***********************************************************************************************//


//*****************************************Parameters********************************************//
//Geometric transformation.
bool transType = 0; // 0: translation, 1: rotation.
float transHor = 16; //Horizontal translation.
float transVer = 16; //Vertical translation.
float angle = 0; //Rotation angle in degrees.

//Gaussian noise parameters.
int stdNoise = 3;

//Estimation motion parameters.
bool LUT = true; //Used Look-Up Tables(true) or functions(false). LUTs makes computations faster.
int noMaxPoints = 1000; //Maximum number of singular points.
//***********************************************************************************************//


int _tmain(int argc, _TCHAR* argv[])
{
	//--Declarations--//.
	int i;
	IplImage* img1_U81C = 0;
	IplImage* img2_U81C = 0;
	IplImage* img3_U83C = 0;
	IplImage* img4_U83C = 0;
	IplImage* img5_U83C = 0;
	FFME ffme;
	CvPoint2D32f* singPoints1;
	CvPoint2D32f* singPoints2;
	int noSingPoints1;
	int noSingPoints2;
	float** descriptors1;
	float** descriptors2;
	int lengthDesc;
	int noCorr;
	CvPoint2D32f** correspondences;
	CvPoint2D32f centerRot;
	CvMat* mat1_32F1C = cvCreateMat(2, 3, CV_32FC1);
	int64 seed = -1;
	CvRNG  randGen = cvRNG(seed);
	CvMat* arrayGaussianNoise;
	double timeMeasured;
	int noTrueCorr;
	float dist;


	//--Read images--//.
	img1_U81C = cvLoadImage(filename1, CV_LOAD_IMAGE_GRAYSCALE);
	if(!img1_U81C)
	{
		printf("The image can not be opened: %s\n", filename1);
	}

	//View images.
	cvNamedWindow( "Original image", 1 );
	cvShowImage( "Original image", img1_U81C );
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
	img2_U81C = cvCreateImage(cvGetSize(img1_U81C), IPL_DEPTH_8U, 1);
	img2_U81C->origin = img1_U81C->origin; 
	img3_U83C = cvCreateImage(cvGetSize(img1_U81C), IPL_DEPTH_8U, 3);
	img3_U83C->origin = img1_U81C->origin; 
	img4_U83C = cvCreateImage(cvGetSize(img1_U81C), IPL_DEPTH_8U, 3);
	img4_U83C->origin = img1_U81C->origin; 
	img5_U83C = cvCreateImage(cvGetSize(img1_U81C), IPL_DEPTH_8U, 3);
	img5_U83C->origin = img1_U81C->origin; 


	//Geometrical transformation.
	if(transType == 0)
	{
		transImg(img1_U81C, img2_U81C, transHor, transVer, mat1_32F1C);
	}
	else
	{
		centerRot.x = img1_U81C->width/2.0f;
		centerRot.y = img1_U81C->height/2.0f;
		rotImg(img1_U81C, img2_U81C, angle, centerRot, mat1_32F1C);
	}


	//Gaussian noise generation.
	arrayGaussianNoise = cvCreateMat(img1_U81C->height, img1_U81C->width, CV_8UC1);
	cvRandArr(&randGen, arrayGaussianNoise, CV_RAND_NORMAL, cvScalar(0), cvScalar(stdNoise));
	cvAdd(img1_U81C, arrayGaussianNoise, img1_U81C, NULL);
	cvAdd(img2_U81C, arrayGaussianNoise, img2_U81C, NULL);

	
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

	
	//Starts time counter.
	timeMeasured = (double)cvGetTickCount(); 


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


	//Matching of singular points. 
	ffme.matchSingPtos(singPoints1, noSingPoints1, descriptors1, singPoints2, noSingPoints2, descriptors2, correspondences, &noCorr);


	//Stops time counter.
	timeMeasured = (double)cvGetTickCount() - timeMeasured;


	//Tests correspondences.
	dist = 1.5;
	for(i=0;i<noCorr;i++)
	{
		singPoints1[i].x = correspondences[i][0].x;
		singPoints1[i].y = correspondences[i][0].y;
		singPoints2[i].x = correspondences[i][1].x;
		singPoints2[i].y = correspondences[i][1].y;
	}
	evalCorr(singPoints1, singPoints2, noCorr, mat1_32F1C, dist, &noTrueCorr);


	//Shows results.
	printf("Number of points detected: %d\n", noSingPoints1);
	printf("Number of correspondences: %d\n", noCorr);
	printf("Number of true correspondences: %d\n", noTrueCorr);
	printf("Measured time: %.1f\n", timeMeasured/(cvGetTickFrequency()*1000.));
	cvNamedWindow( "Transformed image", 1 );
	cvShowImage( "Transformed image", img2_U81C ); //Imagen transformada.
	cvWaitKey(0);
	
	//*************************************Release memory********************************************//
	//Images.
	cvReleaseImage(&img1_U81C);
	cvReleaseImage(&img2_U81C);
	cvReleaseImage(&img3_U83C);
	cvReleaseImage(&img4_U83C);
	cvReleaseImage(&img5_U83C);

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

	//Windows.
	cvDestroyWindow( "Original image" );
	cvDestroyWindow( "Transformed image" );
	//***********************************************************************************************//

    return 0;
}


//-----------------------------Private functions-----------------------------------------//

/* Warping the image using a pure translation transformation with bilinear interpolation.
   Pixels outside the image bounds are set to zero.
   Inputs:
   -img1_U81C: unsigned 8 bit 1 channel IplImage image to be translated.
   -img2_U81C: (input/output)unsigned 8 bit 1 channel IplImage image translated.
   -transHor: horizontal translation.
   -transVer: vertical translation.
   -mat1_32F1C: (input/output) 2x3 matrix. The function doesn't allocate memory.
   Outputs: --
*/
void transImg(IplImage* img1_U81C, IplImage* img2_U81C, float transHor, float transVer, CvMat* mat1_32F1C)
{
	//2x3 geometric transformation initialization:
	//|1 0 tx|  tx: horizontal translation.
	//|0 1 ty|  ty: vertical translation.
	elemMat32F1C_M(mat1_32F1C, 0, 0) = 1;
	elemMat32F1C_M(mat1_32F1C, 0, 1) = 0;
	elemMat32F1C_M(mat1_32F1C, 0, 2) = transHor;
	elemMat32F1C_M(mat1_32F1C, 1, 0) = 0;
	elemMat32F1C_M(mat1_32F1C, 1, 1) = 1;
	elemMat32F1C_M(mat1_32F1C, 1, 2) = transVer;

	//Image warping. Bilinear interpolation. Pixels outside the image bounds are set to zero.
	cvWarpAffine(img1_U81C, img2_U81C, mat1_32F1C, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
}


/* Warping the image using a pure rotation transformation with bilinear interpolation.
   Pixels outside the image bounds are set to zero.
   Inputs:
   -img1_U81C: unsigned 8 bit 1 channel IplImage image to be rotated.
   -img2_U81C: (input/output)unsigned 8 bit 1 channel IplImage image rotated.
   -angle: rotation angle in degrees.
   -center: rotation center.
   -mat1_32F1C: (input/output) 2x3 matrix. The function doesn't allocate memory.
   Outputs: --
*/
void rotImg(IplImage* img1_U81C, IplImage* img2_U81C, float angle, CvPoint2D32f center, CvMat* mat1_32F1C)
{
	int scale = 1;
	//2x3 geometric transformation initialization:
	//|cos() -sin()  0|  
	//|sin()  cos()  0|  ty: vertical translation.
	cv2DRotationMatrix(center, angle, scale, mat1_32F1C);

	//Image warping. Bilinear interpolation. Pixels outside the image bounds are set to zero.
	cvWarpAffine(img1_U81C, img2_U81C, mat1_32F1C, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
}


/* Computes the number of true correspondences between two array of points given the ground truth transformation.
   Inputs:
   -ptos1: array of points.
   -ptos2: array of points.
   -noPtos: number of points.
   -mat1_32F1C: 2x3 matrix storing the geometric transformation.
   -dist: maximum distance allowed between points to be considered as true correspondence.
   -noTrueCorr: (input/output) number of true correspondences.
   Outputs: --
*/
void evalCorr(CvPoint2D32f* ptos1, CvPoint2D32f* ptos2, int noPtos, CvMat* mat1_32F1C, float dist, int* noTrueCorr)
{
	int i;
	CvPoint2D32f pto3;
	CvMat* ptoHomog1 = cvCreateMat(3, 1, CV_32FC1);
	CvMat* ptoCart1 = cvCreateMat(2, 1, CV_32FC1);

	*noTrueCorr = 0;

	for(i=0;i<noPtos;i++)
	{	
		//Application of the geometric transformation.
		elemMat32F1C_M(ptoHomog1, 0, 0) = ptos1[i].x;
		elemMat32F1C_M(ptoHomog1, 1, 0) = ptos1[i].y;
		elemMat32F1C_M(ptoHomog1, 2, 0) = 1;
		cvMatMul(mat1_32F1C, ptoHomog1, ptoCart1);
		pto3.x = elemMat32F1C_M(ptoCart1, 0, 0);
		pto3.y = elemMat32F1C_M(ptoCart1, 1, 0);

		//Checks if the correspondence is true.
		if(euclDistPtos(&pto3, ptos2+i) <= dist) 
		{
			(*noTrueCorr)++;
		}
	}

	//Desallocates memory.
	cvReleaseMat(&ptoHomog1);
	cvReleaseMat(&ptoCart1);
}


/* Euclidean distance between points.
   Inputs:
   -pto1: point.
   -pto2: point.
   Outputs: Euclidean distance.
*/
float euclDistPtos(CvPoint2D32f* pto1, CvPoint2D32f* pto2)
{
	float difx = (pto1->x - pto2->x);
	float dify = (pto1->y - pto2->y);
	return sqrt(difx * difx + dify * dify);
}