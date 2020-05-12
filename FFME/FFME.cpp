#include "FFME.h"
#include "miscellaneous.h"


FFME::FFME(void)
{
}

FFME::~FFME(void)
{
	int i;
	int noElem = 255*8+1; //Range of possible values for the gradient calculated using a Sobel mask.
	
	//Release memory.
	cvReleaseImage(&m_horGradient_S161C);
	cvReleaseImage(&m_verGradient_S161C);
	cvReleaseImage(&m_magGradient_32F1C);
	cvReleaseImage(&m_phaseGradient_32F1C);
	cvReleaseImage(&m_cornerness_32F1C);
	cvFree(&m_ptosGrad);
	cvFree(&m_ptosCornerness);

    //Release LUTs memory.
	for(i=0;i<noElem;i++)
	{
		cvFree(&(m_LutMagGradient[i]));
	}
	cvFree(&m_LutMagGradient);
	for(i=0;i<noElem;i++)
	{
		cvFree(&(m_LutPhaseGradient[i]));
	}
	cvFree(&m_LutPhaseGradient);
}


/* Initialization of memory and look-up tables.
   Inputs:
   -width: image width.
   -height: image height.
   -origin: image origin (field of the OpenCV IplImage structure).
   -maxNoKeyPoints: maximum number of keypoints that can be detected in an image. Used for memory reserving purposes.
   Output: --
*/
void FFME::iniFFME(int width, int height, int origin, int maxNoKeyPoints)
{
	int i,j;
	int noElem = 255*8+1; //Maximum value for the gradient calculated by means of a Sobel mask.
	int shift = -255*4; 
 

	m_maxNoKeyPoints = maxNoKeyPoints;
	m_widthArrayHist = WIDTH_ARRAY_HIST;
	m_widthSubWinHist = WIDTH_SUBWIN_HIST;
	m_noBinsOriHist = NO_BINS_ORI_HIST;
	m_maxRespCompDesc = (float)MAX_RESP_COMP_DESC;
	m_threshGradMag = THRESH_GRAD_MAG;
	m_threshHarris = THRESH_HARRIS;
	m_widthWinHarris = WIDTH_WIN_HARRIS;
	m_widthWinNonMaxSup = WIDTH_WIN_NONMAXSUP;
	m_threshRatSecBest = (float)THRESH_RATIO_SECOND_BEST_CORR;
	m_radMaxSearch = RAD_MAX_SEARCH;
	

	//Memory reserving for images.
	m_horGradient_S161C = cvCreateImage(cvSize(width, height), IPL_DEPTH_16S,1);
	m_horGradient_S161C->origin = origin;
	m_verGradient_S161C = cvCreateImage(cvSize(width, height), IPL_DEPTH_16S,1);
	m_verGradient_S161C->origin = origin;
	m_magGradient_32F1C = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F,1);
	m_magGradient_32F1C->origin = origin;
	m_phaseGradient_32F1C = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F,1);
	m_phaseGradient_32F1C->origin = origin;
	m_cornerness_32F1C = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F,1);
	m_cornerness_32F1C->origin = origin;


	//Gradient magnitude look-up table.
	m_LutMagGradient = (float**)cvAlloc(noElem * sizeof(float*)); 
	for(i=0;i<noElem;i++)
	{
		m_LutMagGradient[i] = (float*)cvAlloc(noElem * sizeof(float));
	}
	for(i=0;i<noElem;i++) //Setup of the table.
	{
		for(j=0;j<noElem;j++)
		{
			m_LutMagGradient[i][j] = sqrt((float)((shift+i)*(shift+i)+(shift+j)*(shift+j)));
		}
	}


	//Gradient phase look-up table (0,2*pi).
	m_LutPhaseGradient = (float**)cvAlloc(noElem * sizeof(float*)); //Memory reserving.
	const float c2pi = (float)(2*CV_PI);
	for(i=0;i<noElem;i++)
	{
		m_LutPhaseGradient[i] = (float*)cvAlloc(noElem * sizeof(float));
	}
	for(i=0;i<noElem;i++) //Setup of the table.
	{
		for(j=0;j<noElem;j++)
		{
			float tmp = atan2((float)(shift+i), (float)(shift+j));
			//Correct the phase to [0,2pi].
			if(tmp >= 0)
			{
				m_LutPhaseGradient[i][j] = tmp;
			}
			else
			{
				m_LutPhaseGradient[i][j] = tmp + c2pi;
			}
		}
	}


	//Reserving memory for the list of points.
	m_ptosGrad = (CvPoint2D32f*)cvAlloc(width * height * sizeof(CvPoint2D32f));
	m_ptosCornerness = (CvPoint2D32f*)cvAlloc(width * height * sizeof(CvPoint2D32f));
}


/* Singular point detection based on functions.
   Input:
   -img_U81C: unsigned 8 bit input image.
   Output:
   -singPtos: (input/output) array of detected singular points. The function doesn't reserve memory.
   -noSingPtos: (input/output) number of detected singular points.
   Output:--
*/
void FFME::singPtoDetFunc(IplImage* img_U81C, CvPoint2D32f* singPtos, int* noSingPtos)
{
	gradientSobel(img_U81C); //Computes the gradient.
	gradMagFunc(); //Computes the gradient magnitud.
	gradMagThresh(m_threshGradMag, 16); //Selection of points by thresholding of the gradient magnitude.
	cornerThresh(m_threshHarris, m_widthWinHarris); //Selection of points by cornerness restriction.
    nonMinSupCorner(m_widthWinNonMaxSup, singPtos, noSingPtos); //Final point selection through non-minimal supression in the cornerness space.
}


/* Singular point detection based on LUT.
   Input:
   -img_U81C: unsigned 8 bit input image.
   Output:
   -singPtos: (input/output) array of detected singular points. The function doesn't reserve memory.
   -noSingPtos: (input/output) number of detected singular points.
*/
void FFME::singPtoDetLut(IplImage* img_U81C, CvPoint2D32f* singPtos, int* noSingPtos)
{
	gradientSobel(img_U81C);//Computes the gradient.
	gradMagLut();//Computes the gradient.
	gradMagThresh(m_threshGradMag, 16); //Selection of points by thresholding of the gradient magnitude.
	cornerThresh(m_threshHarris, m_widthWinHarris); //Selection of points by cornerness restriction.
    nonMinSupCorner(m_widthWinNonMaxSup, singPtos, noSingPtos); //Final point selection through non-minimal supression in the cornerness space.
}


/* Singular point description based on functions.
   Input:
   -img_U81C: unsigned 8 bit input image.
   -singPtos: array of detected singular points.
   -noSingPtos: number of detected singular points.
   -descriptors: (input/output) array of descriptors corresponding to the singular points. The function doesn't reserve memory.
   -normDesc: flag to indicate if the descriptor vector is normalize (true) or not (false). The normalization is used to adress
    changes in illumination.
   Output: --
*/
void FFME::singPtoDescFunc(CvPoint2D32f* singPtos, int noSingPtos, float** descriptors, bool normDesc)
{
    int i;
	int lengthDesc = m_widthArrayHist*m_widthArrayHist*m_noBinsOriHist;
	int noBytesDescriptor = lengthDesc * sizeof(float);
	gradPhaseFunc(); //Computes de gradient phase.
	for(i=0;i<noSingPtos;i++)
	{
		//Inicialization of the descriptor.
		memset(descriptors[i], 0, noBytesDescriptor);
		//Computes the orientation histograms related to the singular point.
		orientHist(singPtos+i, descriptors[i]);
		
		if(normDesc)
		{
			//Normalization of descriptor vector (to be invariant to illumnination changes).
			normDescrip(descriptors[i], lengthDesc, m_maxRespCompDesc);
		}
	}
}


/* Singular point description based on LUT.
   Input:
   -img_U81C: unsigned 8 bit input image.
   -singPtos: array of detected singular points.
   -noSingPtos: number of detected singular points.
   -descriptors: (input/output) array of descriptors corresponding to the singular points. The function doesn't reserve memory.
   -normDesc: flag to indicate if the descriptor vector is normalize (true) or not (false). The normalization is used to adress
    changes in illumination.
   Output: --
   */
void FFME::singPtoDescLut(CvPoint2D32f* singPtos, int noSingPtos, float** descriptors, bool normDesc)
{
	int i;
	int lengthDesc = m_widthArrayHist*m_widthArrayHist*m_noBinsOriHist;
	int noBytesDescriptor = lengthDesc * sizeof(float);
	gradPhaseLut();//Computes de gradient phase.
	for(i=0;i<noSingPtos;i++)
	{
		//Inicialization of the descriptor.
		memset(descriptors[i], 0, noBytesDescriptor);
		//Computes the orientation histograms related to the singular point.
		orientHist(singPtos+i, descriptors[i]);

		if(normDesc)
		{
			//Normalization of descriptor vector (to be invariant to illumnination changes).
			normDescrip(descriptors[i], lengthDesc, m_maxRespCompDesc);
		}
	}
}

/*Matching of singular points. The second nearest neighbord restriction is applied.
  Inputs:
  -singPtos1: array of locations related to the singular points in the first image.
  -noSingPtos1: number of singular points in the first image.
  -descriptors1: array of descriptors related to the singular points in the first image.
  -singPtos2: array of locations related to the singular points in the second image.
  -noSingPtos2: number of singular points in the second image.
  -descriptors2: array of descriptors related to the singular points in the second image.
  -correspondences: (input/output) Nx2 array of correspondences. The first column stores the points 
   of the first image and the second column the points of the second image. Each row represents a correspondence.
  -noCorr: (input/output) number of correspondences.
  Outputs: --
*/
void FFME::matchSingPtos(CvPoint2D32f* singPtos1, int noSingPtos1, float** descriptors1, 
		                 CvPoint2D32f* singPtos2, int noSingPtos2, float** descriptors2, 
						 CvPoint2D32f** correspondences, int* noCorr)
{
	int i,j;
	int lengthDesc = m_widthArrayHist*m_widthArrayHist*m_noBinsOriHist;
	float dist;
	*noCorr = 0;

	for(i=0; i < noSingPtos1; i++)
	{
		float x1 = singPtos1[i].x;
		float y1 = singPtos1[i].y;
		float* desc1 = descriptors1[i];
		float minDist1 = FLT_MAX;
		float minDist2 = FLT_MAX;
		int j1 = -1;
		int j2 = -1;

		for(j = 0; j < noSingPtos2; j++)
		{
			float x2 = singPtos2[j].x;
			float y2 = singPtos2[j].y;
			float* desc2 = descriptors2[j];
			
			//Maximum radius search restriction. Manhattan distance.
			if(m_radMaxSearch >= abs(x2-x1) && m_radMaxSearch >= abs(y2-y1))
			{
				//Similarity measure between descriptor based on Euclidean distance.
				euclDist(desc1, desc2, lengthDesc, &dist);

				//Check if it is the most similar.
				if(dist < minDist1 )
				{
					minDist1 = dist;
					j1 = j;
				}
				//Check if it is the second most similar.
				else if(dist < minDist2)
				{
					minDist2 = dist;
					j2 = j;
				}
			}
		}

		
		if (j1 != -1 && j2 != -1)
		{
			//Second nearest neighbor restriction.
			if(minDist1 <= minDist2 * m_threshRatSecBest)
			{
				correspondences[*noCorr][0] = singPtos1[i];
				correspondences[*noCorr][1] = singPtos2[j1];
				(*noCorr)++;
			}
		}
		//Only one correspondence exists, so it is supposed correct.
		else if(j1 != -1 && j2 == -1)
		{		
			correspondences[*noCorr][0] = singPtos1[i];
			correspondences[*noCorr][1] = singPtos2[j1];
			(*noCorr)++;
		}
	}
}


//****************************************************************************************
// Private functions
//****************************************************************************************


/* Compute the horizontal and vertical image gradient based on Sobel. The image gradient is not
   normalized, and its range is (-4*255, 4*255). The results are stored in class members: 
   m_horGradient_S161C and m_verGradient_S161C respectively.
   Inputs:
   -img_U81C: unsigned 8 bit 1 channel image.
   Outputs: --
*/
void FFME::gradientSobel(IplImage* img_U81C)
{
	cvSobel( img_U81C, m_horGradient_S161C, 1, 0, 3 ); //Horizontal.
	cvSobel( img_U81C, m_verGradient_S161C, 0, 1, 3 ); //Vertical.
}


/* Compute the gradient magnitude based on functions. The result is stored in the class member:
   m_magGradient_32F1C.
*/
void FFME::gradMagFunc()
{
	int i,j;
	for(i=0;i<m_horGradient_S161C->height;i++)
	{
		for(j=0;j<m_horGradient_S161C->width;j++)
		{
			int dx = pixelImgS161C_M(m_horGradient_S161C, i, j);
			int dy = pixelImgS161C_M(m_verGradient_S161C, i, j);
			pixelImg32F1C_M(m_magGradient_32F1C, i, j) = sqrt((float)(dx*dx + dy*dy)); 
		}
	}
}


//Compute the gradient magnitude based on LUT.
void FFME::gradMagLut()
{
	int i,j;
	int shift = 255*4; //Shift for accessing to the LUT.
	for(i=0;i<m_horGradient_S161C->height;i++)
	{
		for(j=0;j<m_horGradient_S161C->width;j++)
		{
			pixelImg32F1C_M(m_magGradient_32F1C, i, j) = 
				m_LutMagGradient[pixelImgS161C_M(m_verGradient_S161C, i, j) + shift][pixelImgS161C_M(m_horGradient_S161C, i, j) + shift];
		}
	}
}


/* Selection of points by gradient magnitude thresholding. The 'noPix' nearer from image borders are discarded.
   Inputs:
   -thresh: gradient magnitude threshold. You must take into account that the gradient magnitud range is [0,].
   -noPix: number of pixels discarded from the image borders.
   Output: --
*/
void FFME::gradMagThresh(float thresh, int noPix)
{
	int i,j;
	int width = m_magGradient_32F1C->width;
	int height = m_magGradient_32F1C->height;
	m_noPtosGrad = 0;
	int condition1 = height-noPix;
	int condition2 = width-noPix;
	for(i=noPix;i<condition1;i++)
	{
		for(j=noPix;j<condition2;j++)
		{
			if(pixelImg32F1C_M(m_magGradient_32F1C, i, j) >= thresh)
			{
				m_ptosGrad[m_noPtosGrad++] = cvPoint2D32f((float)j,(float)i);
			}
		}
	}
}


/* Selection of points of high cornerness. 
   Inputs:
   -thresh: cornerness threshold based on Harris condition (ratio of eigenvalues).
   -sizeWin: size of the window used to calculate the cornerness.
   Output: --
*/
void FFME::cornerThresh(float thresh, int sizeWin)
{
	int i, u, v;
	int rWin = cvRound((sizeWin-1)/2.0);
	float avgdx, avgdy, avgdxdy, trace, det, cornerness;
	float harrisCond = (thresh+1)*(thresh+1)/thresh; //Harris condition for cornerness.
	m_noPtosCornerness = 0;
	cvSet(m_cornerness_32F1C, cvScalar(FLT_MAX)); //Sparse cornerness map that is used by 'nonMinSupCorner()' function.

	for(i=0;i<m_noPtosGrad;i++)
	{
		CvPoint pto = cvPointFrom32f(m_ptosGrad[i]);
		
		//The point is discarded if its neighborhood is not inside the image bounds. 
		if(checkSquareReg(m_horGradient_S161C, pto.y, pto.x, rWin))
		{		
			avgdx = 0;
			avgdy = 0;
			avgdxdy = 0;
			int condition1 = pto.y + rWin;
			int condition2 = pto.x + rWin;
			for(v=pto.y-rWin; v<=condition1; v++)
			{
				for(u=pto.x-rWin; u<=condition2; u++)
				{
					//Matrix A of Harris: average gradient matrix. The average is not isotropic.
					int dxTmp = pixelImgS161C_M(m_horGradient_S161C, v, u);
					int dyTmp = pixelImgS161C_M(m_verGradient_S161C, v, u);
					avgdx += dxTmp * dxTmp;
					avgdy += dyTmp * dyTmp;
					avgdxdy += dxTmp * dyTmp;;
				}
			}

			trace = avgdx + avgdy;
			det = avgdx * avgdy - avgdxdy * avgdxdy;
			//The point is discarded if its determinant is negative or zero.
			if(det > 0)
			{
				cornerness = trace * trace / det;
				//Check the corner restriction.
				if(cornerness < harrisCond)
				{
					m_ptosCornerness[m_noPtosCornerness++] = m_ptosGrad[i];
					pixelImg32F1C_M(m_cornerness_32F1C, pto.y, pto.x) = cornerness;
				}
			}
		}
	}
}


/* Non minimal supression in the cornerness space.
   Inputs:
   -sizeWin: Size of the window size used to compute the non minimal supression restriction.
   -ptos: (input/output) array of points that fulfill the non minimal supression restriction. 
          The function doesn't reserve memory.
   -noPtos: (input/output) number of points that fulfill the non minimal supression restriction.
   Output: --
*/
void FFME::nonMinSupCorner(int sizeWin, CvPoint2D32f* ptos, int* noPtos)
{
    int i, v, u;
	int rWin = cvFloor((sizeWin-1)/2.0);
	*noPtos = 0;

	for(i=0;i<m_noPtosCornerness;i++)
	{
		CvPoint pto = cvPointFrom32f(m_ptosCornerness[i]);
		float val = pixelImg32F1C_M(m_cornerness_32F1C, pto.y, pto.x);
		int condition1 = pto.y+rWin;
		int condition2 = pto.x+rWin;
		if( val > 0 )//Check for minimum.
		{
			for( v = pto.y-rWin; v <= condition1; v++ )
			{
				for( u = pto.x-rWin; u <= condition2; u++ )
				{
					if( val > pixelImg32F1C_M(m_cornerness_32F1C, v, u))
					{
						goto noMin;
					}
				}
			}
			
			//Check the maximum number of singular points allowed.
			if((*noPtos) <= (m_maxNoKeyPoints - 1))
			{
				ptos[(*noPtos)++] = m_ptosCornerness[i];
			}
			else
			{
				return;
			} 
		}

		noMin:
		;
	}
}


//Compute the gradient phase based on functions. Range = (0,2pi).
void FFME::gradPhaseFunc()
{
	int i,j;
	const float c2pi = (float)(2 * CV_PI);
	for(i=0;i<m_horGradient_S161C->height;i++)
	{
		for(j=0;j<m_horGradient_S161C->width;j++)
		{
			int dx = pixelImgS161C_M(m_horGradient_S161C, i, j);
			int dy = pixelImgS161C_M(m_verGradient_S161C, i, j);
			float tmp = atan2((float)dy, (float)dx); //[-pi,pi]
			//Correct the phase to [0,2pi].
			if(tmp >= 0)
			{
				pixelImg32F1C_M(m_phaseGradient_32F1C, i, j) = tmp;
			}
			else
			{
				pixelImg32F1C_M(m_phaseGradient_32F1C, i, j) = tmp + c2pi;
			}
		}
	}
}


//Compute the gradient phase based on LUT. Range = (0,2pi).
void FFME::gradPhaseLut()
{
	int i,j;
	int shift = 255*4; //Shift for accessing to LUT.
	for(i=0;i<m_horGradient_S161C->height;i++)
	{
		for(j=0;j<m_horGradient_S161C->width;j++)
		{
			pixelImg32F1C_M(m_phaseGradient_32F1C, i, j) = 
			    m_LutPhaseGradient[pixelImgS161C_M(m_verGradient_S161C, i, j) + shift][pixelImgS161C_M(m_horGradient_S161C, i, j) + shift];
		}
	}
}


/* Compute the orientations histograms.
   Input:
   -img_U81C: unsigned 8 bit input image.
   -singPto: singular point.
   -descriptor: (input/output) descriptor vector corresponding to the singular points. The function doesn't reserve memory.
   Output: --
*/
void FFME::orientHist(CvPoint2D32f* singPto, float* descriptor)
{
	int i,j,r,s,width,radius;
	float phaseFactor, exponent, w, shift, rbin0, cbin0, rbin, cbin, obin, gradVal;

	phaseFactor = (float)(m_noBinsOriHist / (2.0 * CV_PI));
	exponent = (float)(m_widthArrayHist * m_widthArrayHist * 0.5); //Exponent of the Gaussian smoothing.
	shift = (float)(m_widthArrayHist * 0.5); //Shift for the coordinates.
	width = m_widthArrayHist * m_widthSubWinHist; //Width in pixels of the neighborhood.
	if((width % 2) != 0) //Check if 'width' is odd.
	{
		radius = (width-1) / 2;
		for(i=-radius; i<=radius; i++) 
		{
			for(j=-radius; j<=radius; j++) 
			{
				r = i + cvRound(singPto->y); //Discrete values.
				s = j + cvRound(singPto->x);

				//Check that pixel coordinates are inside image bounds.
				if (checkMargins(m_magGradient_32F1C, r, s))
				{
		
				//Array histogram coordinates to calculate the Gaussian smoothing contribution.
				rbin0 = ((float)i / m_widthSubWinHist); //Continuos values.
				cbin0 = ((float)j / m_widthSubWinHist);
				//Shifted array histogram coordinates.
				rbin = rbin0 + shift;
				cbin = cbin0 + shift;
				
				w = exp(-(cbin0*cbin0 + rbin0*rbin0) / exponent);//Weigths related to the Gaussian smoothing.
				gradVal = pixelImg32F1C_M(m_magGradient_32F1C, r, s);//Gradient magnitude value.
				obin = pixelImg32F1C_M(m_phaseGradient_32F1C, r, s) * phaseFactor;//Orientation bin.	
				//Orientation histogram contributions by trilinear interpolation.
				trilinearInterp(rbin, cbin, obin, gradVal*w, descriptor);
				}
			}
		}
	}
	else
	{
		radius = width / 2;
		for(i=-radius; i<radius; i++) 
		{
			for(j=-radius; j<radius; j++) 
			{
				r = i + cvRound(singPto->y); //Discrete values.
				s = j + cvRound(singPto->x);

				//Check that pixel coordinates are inside image bounds.
				if (checkMargins(m_magGradient_32F1C, r, s))
				{
					//Array histogram coordinates to calculate the Gaussian smoothing contribution.
					rbin0 = ((float)(i + 0.5) / m_widthSubWinHist); //Continuos values.
					cbin0 = ((float)(j + 0.5) / m_widthSubWinHist);
					//Shifted array histogram coordinates.
					rbin = rbin0 + shift;
					cbin = cbin0 + shift;

					w = exp(-(cbin0*cbin0 + rbin0*rbin0) / exponent);//Weigths related to the Gaussian smoothing.
					gradVal = pixelImg32F1C_M(m_magGradient_32F1C, r, s);//Gradient magnitude value.
					obin = pixelImg32F1C_M(m_phaseGradient_32F1C, r, s) * phaseFactor;//Orientation bin.	
					//Orientation histogram contributions by trilinear interpolation.
					trilinearInterp(rbin, cbin, obin, gradVal*w, descriptor);
				}
			}
		}
	}
}


/* Orientation histogram contributions by trilinear interpolation.
   Input:
   -rbin, cbin: raw and column bin. 
   -obin: orientation bin.
   -gradVal: gradient value.
   -descriptor: address of the descriptor related to the singular point.
   Output:--
*/
void FFME::trilinearInterp(float rbin, float cbin, float obin, float gradVal, float* descriptor)
{
	float d_r, d_c, d_o, val1, val2, val3;
	int shiftRow, shiftCol;
	int r0, c0, o0, rb, cb, ob, r, c, o;

	//Rectify the contribution to the bin.
	rbin = rbin-0.5f;
	cbin = cbin-0.5f;
	//Interpolation factors.
	r0 = cvFloor(rbin);
	c0 = cvFloor(cbin);
	o0 = cvFloor(obin);
	d_r = rbin - r0;
	d_c = cbin - c0;
	d_o = obin - o0;

	//Trilinear interpolation. A pixel can contributed up to 8 orientation bins.
	for(r = 0; r <= 1; r++)
	{
		rb = r0 + r;
		if(rb >= 0  &&  rb <= m_widthArrayHist-1)
		{
			val1 = gradVal*(( r == 0 )? 1.0f - d_r : d_r);
			shiftRow = rb*m_widthArrayHist*m_noBinsOriHist;
			for(c = 0; c <= 1; c++)
			{
				cb = c0 + c;
				if(cb >= 0  &&  cb <= m_widthArrayHist-1)
				{
					val2 = val1*(( c == 0 )? 1.0f - d_c : d_c);
					shiftCol = cb*m_noBinsOriHist;
					for(o = 0; o <= 1; o++)
					{
						val3 = val2*(( o == 0 )? 1.0f - d_o : d_o);
						ob = (o0 + o) % m_noBinsOriHist; //Range checking.
						*(descriptor+shiftRow+shiftCol+ob) += val3;
					}
				}
			}
		}
	}
}


/* Normalization of descriptor vector.
   The purpose of the normalization is to be invariant to affine 
   illumnination changes and to present some robustness to non-linear illuminations changes.
   Input:
   -descriptor: vector descriptor obtained by the function 'singPtoDescXXX()'.
   -length: number of components of the descriptor vector.
   -maxRespComp: maximum response allowed per vector component.
   Output: --
*/
void FFME::normDescrip(float* descriptor, int length, float maxRespComp)
{
	int i;
	bool reNorm = false;

	//Normalization.
	normVector(descriptor, length);
	
	//Restrict the contribution of each vector. It makes the descriptor more robust to non-linear illumination changes.
	for(i = 0; i<length; i++)
	{
		if(descriptor[i] > maxRespComp)
		{
			descriptor[i] = maxRespComp;
			reNorm = true;
		}
	}
	
	//Re-normalization.
	if(reNorm)
	{
		normVector(descriptor, length);
	}
}


/* Vector normalization for float type.
   Inputs:
   -vector: vector to be normalized. 
   -length: vector length.
   Outputs: --
*/
void FFME::normVector(float* vector, int length)
{
	int i;
	float val, sumSq, invSumSq;

	sumSq = 0;
	for(i=0; i<length; i++)
	{
		val = vector[i];
		sumSq += val*val;
	}
	invSumSq = (float)(1.0 / sqrt(sumSq)); 
	for(i = 0; i<length; i++)
	{
		vector[i] *= invSumSq;
	}
}


/* Euclidean distance of two vectors of type float.
   Inputs:
   -vector1: float vector.
   -vector2: float vector.
   -length: length of vector 1 and 2.
   -dist: (input/output) euclidean distance.
   Outputs: --
*/
void FFME::euclDist(float* vector1, float* vector2, int length, float* dist)
{
	int i;
	float tmp, dif;
		
	tmp = 0;
	for( i = 0; i < length; i++ )
	{
		dif = vector1[i] - vector2[i];
		tmp += dif * dif;
	}
	*dist = sqrt(tmp);
}


