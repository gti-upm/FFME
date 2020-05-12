//-------------------------------------------------------------------------
// miscellaneous.cpp 
//-------------------------------------------------------------------------
// Description: miscellaneous functions and macros.
//-------------------------------------------------------------------------
// Author: Carlos Roberto del Blanco Adán, 
//         Grupo de Tratamiento de Imágenes, GTI SSR, Madrid
//		   cda@gti.ssr.upm.es
// Date: 02/12/2007
// Version 1.0
//-------------------------------------------------------------------------

#include "miscellaneous.h"


/* Converts an image of type XX1C to U81C and scales the intensity values to show the maximum dynamic range of the image. It is used for display purposes. 
   Inputs:
   -imgXX1C: whatever image of 1 channel.
   -imgU81C: (input/output) unsigned 8 bit 1 channel image.
   Outputs:--
*/
void scaleXX1CtoU81C(IplImage* imgXX1C, IplImage* imgU81C)
{
	double min_val, max_val;
	IplImage* tmp = cvCloneImage(imgXX1C);
	cvMinMaxLoc(tmp, &min_val, &max_val); //Max and min image value.
	cvAddS(tmp, cvScalar(-min_val), tmp, NULL); // Shift the intensity range to it begins in zero. 
	max_val = max_val - min_val;
	cvConvertScale( tmp, imgU81C, 255/max_val, 0 ); //Image conversión with linear transformation.
}


/* Draws an array of points as white points in a black and white image.
   Inputs:
   -imgU81C: unsigned 8 bit 1 channel IplImage image.
   -ptos: points to be drawn.
   -noPtos: number of points.
   Outputs:--
*/
void drawPtosBW(IplImage* img_U81C, CvPoint2D32f* ptos, int noPtos)
{
	int i;
	cvZero(img_U81C);
	for(i=0;i<noPtos;i++)
	{
		pixelImgU81C_M(img_U81C, cvRound(ptos[i].y), cvRound(ptos[i].x)) = 255;
	}
}


/* Print the values of a float vector.
   Input:
   -vector: float vector.
   -length: length of the vector to be printed.
   -newline: determines the width in number of floats of each row. 
   Otuput:--
*/
void printVectorFloat(float* vector, int length, int newline)
{
	int i;
	for(i=0;i<length;i++)
	{
		printf("%3.3f ", vector[i]);
		if((i%newline) == (newline-1))
		{
			printf("\n");
		}
	}
	printf("\n");
}


/* Draw singular points on an image.
   Inputs:
   -img8U3C: (input/output) unsigned 8 bit 3 channel IplImage image where the motion vectors are going to be drawn.
   -singPtos: array of locations related to the singular points in the first image.
   -noSingPtos: number of singular points in the first image.
   Outputs: --

*/
void drawPtosIm(IplImage* imgU83C, CvPoint2D32f* singPtos, int noSingPtos)
{
	int i;

	for(i=0; i<noSingPtos; i++)
	{
		cvCircle(imgU83C, cvPointFrom32f(singPtos[i]), 1, CV_RGB(0,255,0), -1, 8,0);
	}
}


/* Draw correspondences as a sparse motion vector field.
   Inputs:
   -img8U3C: (input/output) unsigned 8 bit 3 channel IplImage image where the motion vectors are going to be drawn.
   -correspondences: Nx2 array of correspondences. The first column stores the points of the first image and the second
    column the points of the second image. Each row represents a corresponde.
   -noCorr: number of correspondences.
   Outputs: --

*/
void drawCorr(IplImage* imgU83C, CvPoint2D32f** correspondences, int noCorr)
{
	int i;

	for(i=0; i<noCorr; i++)
	{
		cvCircle(imgU83C, cvPointFrom32f(correspondences[i][0]), 1, CV_RGB(0,255,0), -1, 8,0);
		cvLine(imgU83C, cvPointFrom32f(correspondences[i][0]), cvPointFrom32f(correspondences[i][1]), CV_RGB(0,255,0), 1, 8, 0);
	}
}


/* Print correspondence vector values.
   Inputs:
   -correspondences: Nx2 array of correspondences. The first column stores the points of the first image and the
    second column the points of the second image. Each row represents a corresponde.
   -noCorr: number of correspondences.
   Outputs: --
*/
void printCorr(CvPoint2D32f** correspondences, int noCorr)
{
	int i;
	
	for(i=0;i<noCorr;i++)
	{
		printf("X1:%f  Y1:%f  X2:%f  Y2:%f\n", correspondences[i][0].x, correspondences[i][0].y, correspondences[i][1].x ,correspondences[i][1].y);
	}
}