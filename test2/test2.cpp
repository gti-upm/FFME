//-------------------------------------------------------------------------
// test2.cpp 
//-------------------------------------------------------------------------
// Description: tests the FFME class functions that create descriptors from 
// singular points.
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


//*****************************************Parameters********************************************//.

//***********************************************************************************************//


int _tmain(int argc, _TCHAR* argv[])
{
	//--Declarations--//.
	int i;
	IplImage* img2_32F1C = 0;
	IplImage* img3_S161C = 0;
	IplImage* img4_S161C = 0;
	FFME ffme;
	CvPoint2D32f singPoints1 = cvPoint2D32f( 150.0, 150.0 );
	int noSingPoints1 = 1;
	float** descriptors1;
	int lengthDesc;
	int noMaxPoints = 1000;
	int gradVal;


	//--Initialization--//.
	ffme.iniFFME(300, 300, 0, noMaxPoints);
	descriptors1 = (float**)cvAlloc(noMaxPoints * sizeof(float*));
	lengthDesc = ffme.m_widthArrayHist*ffme.m_widthArrayHist*ffme.m_noBinsOriHist; //Length of descriptor vector.
	for(i=0;i<noMaxPoints;i++)
	{
		descriptors1[i] = (float*)cvAlloc(lengthDesc*sizeof(float));
	}

	//Creates synthetically magnitud gradient image.
	ffme.getGradMag(&img2_32F1C);
	cvSet(img2_32F1C, cvScalar(1));

	//Tests several angles configurations.
	for (i=0;i<(2*ffme.m_noBinsOriHist);i++)
	{
		double ang = i*(2*CV_PI/16.0);
		printf("Ang: %f\n\n", ang*360/(2*CV_PI));
		//Creates synthetically vertical and horizontal gradient images.
		ffme.getHorGradient(&img3_S161C);
		gradVal = cvRound(cos(ang)*1000);
		cvSet(img3_S161C, cvScalar(gradVal));
		ffme.getVerGradient(&img4_S161C);
		gradVal = cvRound(sin(ang)*1000);
		cvSet(img4_S161C, cvScalar(gradVal));

		//Singular point description for image 1 based on functions.
		ffme.singPtoDescFunc(&singPoints1, noSingPoints1, descriptors1, true);
		//Shows the first descriptor.
		printf("Computes descriptor based on functions:\n\n");
		printVectorFloat(descriptors1[0], ffme.m_widthArrayHist * ffme.m_widthArrayHist * ffme.m_noBinsOriHist, ffme.m_noBinsOriHist);

		//Singular point description for image 1 based on LUTs.
		ffme.singPtoDescLut(&singPoints1, noSingPoints1, descriptors1, true);
		//Shows the first descriptor.
		printf("Computes descriptor based on LUTs:\n\n");
		printVectorFloat(descriptors1[0], ffme.m_widthArrayHist * ffme.m_widthArrayHist * ffme.m_noBinsOriHist, ffme.m_noBinsOriHist);
		cvNamedWindow( "tmp", 1 );
		cvWaitKey(0);
	}

	
	//*************************************Release memory********************************************//
	//Data.
	for(i=0;i<noMaxPoints;i++)
	{
		cvFree(descriptors1+i);
	}
	cvFree(&descriptors1);

	//Windows.
	cvDestroyWindow( "tmp" );
	//***********************************************************************************************//

    return 0;
}



