/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2014, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     encmain.cpp
	\brief    Encoder application main
	*/

#include "TLibCommon/GlobalVariables.h"
#include <time.h>
#include <iostream>
#include <stdlib.h> 
#include <Windows.h>
#include "TAppEncTop.h"
#include "TLibEncoder/AnnexBwrite.h"

#include "TAppCommon/program_options_lite.h"
#include "../App/TAppDecoder/TAppDecTop.h"
#include "TLibDecoder/AnnexBread.h"
#include "TLibDecoder/NALread.h"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define _CRT_SECURE_NO_WARNINGS 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "downconvert.h"
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define CRTDBG_MAP_ALLOC
#include <crtdbg.h>

using namespace std;
namespace po = df::program_options_lite;

//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Main function
// ====================================================================================================================
/*
TComList<TComPic*> pcListPicDec;
TComList<TComPic*> pcListPicRec;
TComPic* mypcPic = NULL;
TComPic* pcPicDec = NULL;
TComPic* pcPicRec = NULL;
int kk_framesToBeEncoded = 0;
int myuiPartUnitIdx = 0;
*/
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void
createColorComponent(ColorComponent& c, int maxwidth, int maxheight)
{
	maxwidth = ((maxwidth + 15) >> 4) << 4;
	maxheight = ((maxheight + 15) >> 4) << 4;
	int size = maxwidth * maxheight;
	c.stride = maxwidth;
	c.lines = maxheight;
	c.data = new unsigned char[size];
	c.data2 = new unsigned char[size];

	if (!c.data || !c.data2)
	{
		fprintf(stderr, "\nERROR: memory allocation failed!\n\n");
		exit(1);
	}
}

void
deleteColorComponent(ColorComponent& c)
{
	delete[] c.data;
	delete[] c.data2;
	c.stride = 0;
	c.lines = 0;
	c.data = 0;
	c.data2 = 0;
}

int
readColorComponent(ColorComponent& c, Pel* piPicOrg, int stride, int width, int height, bool second)
{
	assert(width <= c.stride);
	assert(height <= c.lines);

	int iMaxPadWidth = gMin(c.stride, ((width + 15) >> 4) << 4);
	int iMaxPadHeight = gMin(c.lines, ((height + 31) >> 5) << 5);

	for (int i = 0; i < height; i++)
	{
		unsigned char* buffer = (second ? c.data2 : c.data) + i * c.stride;
//		int rsize = (int)fread(buffer, sizeof(unsigned char), width, file);
		for (int j = 0; j < width; j++)
		{
			buffer[j] = (unsigned char)piPicOrg[j + i * stride];
		}
/*		if (rsize != width)
		{
			return 1;
		}*/
		for (int xp = width; xp < iMaxPadWidth; xp++)
		{
			buffer[xp] = buffer[xp - 1];
		}
	}
	for (int yp = height; yp < iMaxPadHeight; yp++)
	{
		unsigned char* buffer = (second ? c.data2 : c.data) + yp * c.stride;
		unsigned char* bufferX = buffer - c.stride;
		for (int xp = 0; xp < c.stride; xp++)
		{
			buffer[xp] = bufferX[xp];
		}
	}
	return 0;
}

void
duplicateColorComponent(ColorComponent& c)
{
	memcpy(c.data2, c.data, c.stride * c.lines * sizeof(unsigned char));
}

void
combineTopAndBottomInColorComponent(ColorComponent& c, Bool bBotField)
{
	int            offs = (bBotField ? c.stride : 0);
	unsigned char* pDes = c.data + offs;
	unsigned char* pSrc = c.data2 + offs;
	for (int i = 0; i < c.lines / 2; i++, pDes += 2 * c.stride, pSrc += 2 * c.stride)
	{
		memcpy(pDes, pSrc, c.stride * sizeof(unsigned char));
	}
}

void
writeColorComponent(ColorComponent& c, FILE* file, int width, int height, bool second)
{
	assert(width <= c.stride);
	assert(height <= c.lines);

	for (int i = 0; i < height; i++)
	{
		unsigned char* buffer = (second ? c.data2 : c.data) + i * c.stride;
		int            wsize = (int)fwrite(buffer, sizeof(unsigned char), width, file);

		if (wsize != width)
		{
			fprintf(stderr, "\nERROR: while writing to output file!\n\n");
			exit(1);
		}
	}
}


void
createFrame(YuvFrame& f, int width, int height)
{
	createColorComponent(f.y, width, height);
	createColorComponent(f.u, width >> 1, height >> 1);
	createColorComponent(f.v, width >> 1, height >> 1);
}

void
deleteFrame(YuvFrame& f)
{
	deleteColorComponent(f.y);
	deleteColorComponent(f.u);
	deleteColorComponent(f.v);
}

int
readFrame(YuvFrame& f, TComPicYuv* &pcPicYuvOrg, int width, int height, bool second = false)
{
	ROTRS(readColorComponent(f.y, pcPicYuvOrg->getLumaAddr(),pcPicYuvOrg->getStride(), width, height, second), 1);
	ROTRS(readColorComponent(f.u, pcPicYuvOrg->getCbAddr(),pcPicYuvOrg->getCStride(), width >> 1, height >> 1, second), 1);
	ROTRS(readColorComponent(f.v, pcPicYuvOrg->getCrAddr(),pcPicYuvOrg->getCStride(), width >> 1, height >> 1, second), 1);
	return 0;
}

void
duplicateFrame(YuvFrame& f)
{
	duplicateColorComponent(f.y);
	duplicateColorComponent(f.u);
	duplicateColorComponent(f.v);
}

void
combineTopAndBottomInFrame(YuvFrame& f, Bool botField)
{
	combineTopAndBottomInColorComponent(f.y, botField);
	combineTopAndBottomInColorComponent(f.u, botField);
	combineTopAndBottomInColorComponent(f.v, botField);
}

void
writeFrame(YuvFrame& f, FILE* file, int width, int height, bool both = false)
{
	writeColorComponent(f.y, file, width, height, false);
	writeColorComponent(f.u, file, width >> 1, height >> 1, false);
	writeColorComponent(f.v, file, width >> 1, height >> 1, false);

	if (both)
	{
		writeColorComponent(f.y, file, width, height, true);
		writeColorComponent(f.u, file, width >> 1, height >> 1, true);
		writeColorComponent(f.v, file, width >> 1, height >> 1, true);
	}
}


void
print_usage_and_exit(int test, const char* name, const char* message = 0)
{
	if (test)
	{
		if (message)
		{
			fprintf(stderr, "\nERROR: %s\n", message);
		}
		fprintf(stderr, "\nUsage: %s <win> <hin> <in> <wout> <hout> <out> [<method> [<t> [<skip> [<frms>]]]] [[-crop <args>] [-phase <args>] [-resample_mode <arg>]]\n\n", name);
		fprintf(stderr, "  win     : input width  (luma samples)\n");
		fprintf(stderr, "  hin     : input height (luma samples)\n");
		fprintf(stderr, "  in      : input file\n");
		fprintf(stderr, "  wout    : output width  (luma samples)\n");
		fprintf(stderr, "  hout    : output height (luma samples)\n");
		fprintf(stderr, "  out     : output file\n");
		fprintf(stderr, "\n--------------------------- OPTIONAL ---------------------------\n\n");
		fprintf(stderr, "  method  : rescaling methods (default: 0)\n");
		fprintf(stderr, "            0: normative upsampling\n");
		fprintf(stderr, "               non-normative downsampling (JVT-R006)\n");
		fprintf(stderr, "            1: dyadic upsampling (AVC 6-tap (1/2 pel) on odd samples\n");
		fprintf(stderr, "               dyadic downsampling (MPEG-4 downsampling filter)\n");
		fprintf(stderr, "            2: crop only\n");
		fprintf(stderr, "            3: upsampling (Three-lobed Lanczos-windowed sinc)\n");
		fprintf(stderr, "            4: upsampling (JVT-O041: AVC 6-tap 1/2 pel + bilinear 1/4 pel)\n");
		fprintf(stderr, "  t       : number of temporal downsampling stages (default: 0)\n");
		fprintf(stderr, "  skip    : number of frames to skip at start (default: 0)\n");
		fprintf(stderr, "  frms    : number of frames wanted in output file (default: max)\n");
		fprintf(stderr, "\n-------------------------- OVERLOADED --------------------------\n\n");
		fprintf(stderr, " -crop  <type> <params>\n");
		fprintf(stderr, "   type   : 0: Sequence level,    1: Picture level\n");
		fprintf(stderr, "   params : IF Sequence level: <x_orig> <y_orig> <crop_width> <crop_height>\n");
		fprintf(stderr, "               cropping window origin (x,y) and dimensions (width and height)\n");
		fprintf(stderr, "            IF Picture level: <crop_file>\n");
		fprintf(stderr, "                 input file containing cropping window parameters.\n");
		fprintf(stderr, "                 each line has four integer numbers separated by a comma\n");
		fprintf(stderr, "                 as following: \"x_orig, y_orig, crop_width, crop_height\"\n");
		fprintf(stderr, "                 for each picture to be resampled;\n");
		fprintf(stderr, "\n");
		fprintf(stderr, " -phase <in_uv_ph_x> <in_uv_ph_y> <out_uv_ph_x> <out_uv_ph_y>\n");
		fprintf(stderr, "   in_uv_ph_x : input  chroma phase shift in horizontal direction (default:-1)\n");
		fprintf(stderr, "   in_uv_ph_y : input  chroma phase shift in vertical   direction (default: 0)\n");
		fprintf(stderr, "   out_uv_ph_x: output chroma phase shift in horizontal direction (default:-1)\n");
		fprintf(stderr, "   out_uv_ph_y: output chroma phase shift in vertical   direction (default: 0)\n");
		fprintf(stderr, "\n");
		fprintf(stderr, " -resample_mode <resample_mode>\n");
		fprintf(stderr, "   resample_mode : resampling modes, present when method==0 (default: 0)\n");
		fprintf(stderr, "                 0: low-res-frm  = progressive, high-res-frm = progressive\n");
		fprintf(stderr, "                 1: low-res-frm  = interlaced,  high-res-frm = interlaced\n");
		fprintf(stderr, "                 2: low-res-frm  = progressive  (top-coincided)\n");
		fprintf(stderr, "                    high-res-frm = interlaced\n");
		fprintf(stderr, "                 3: low-res-frm  = progressive  (bot-coincided)\n");
		fprintf(stderr, "                    high-res-frm = interlaced\n");
		fprintf(stderr, "                 4: low-res-frm  = interlaced   (top-first)\n");
		fprintf(stderr, "                    high-res-frm = progressive  (double frm rate)\n");
		fprintf(stderr, "                 5: low-res-frm  = interlaced   (bot-first)\n");
		fprintf(stderr, "                    high-res-frm = progressive  (double frm rate)\n");
		fprintf(stderr, "\n\n");
		exit(1);
	}
}


void
updateCropParametersFromFile(ResizeParameters& cRP, FILE* cropFile, int resamplingMethod, char* name)
{
	int crop_x0 = 0;
	int crop_y0 = 0;
	int crop_w = 0;
	int crop_h = 0;
	if (fscanf(cropFile, "%d,%d,%d,%d\n", &crop_x0, &crop_y0, &crop_w, &crop_h) == 4)
	{
		cRP.m_iLeftFrmOffset = crop_x0;
		cRP.m_iTopFrmOffset = crop_y0;
		cRP.m_iScaledRefFrmWidth = crop_w;
		cRP.m_iScaledRefFrmHeight = crop_h;
	}
	print_usage_and_exit(cRP.m_iLeftFrmOffset & 1 || cRP.m_iTopFrmOffset & 1, name, "cropping parameters must be even values");
	print_usage_and_exit(cRP.m_iScaledRefFrmWidth & 1 || cRP.m_iScaledRefFrmHeight & 1, name, "cropping parameters must be even values");
	print_usage_and_exit(resamplingMethod == 2 && cRP.m_iScaledRefFrmWidth != gMin(cRP.m_iRefLayerFrmWidth, cRP.m_iFrameWidth), name, "crop dimensions must be the same as the minimal dimensions");
	print_usage_and_exit(resamplingMethod == 2 && cRP.m_iScaledRefFrmHeight != gMin(cRP.m_iRefLayerFrmHeight, cRP.m_iFrameHeight), name, "crop dimensions must be the same as the minimal dimensions");
	print_usage_and_exit(cRP.m_iScaledRefFrmWidth  > gMax(cRP.m_iRefLayerFrmWidth, cRP.m_iFrameWidth), name, "wrong crop window size");
	print_usage_and_exit(cRP.m_iScaledRefFrmHeight > gMax(cRP.m_iRefLayerFrmHeight, cRP.m_iFrameHeight), name, "wrong crop window size");
	print_usage_and_exit(cRP.m_iScaledRefFrmWidth  < gMin(cRP.m_iRefLayerFrmWidth, cRP.m_iFrameWidth), name, "wrong crop window size");
	print_usage_and_exit(cRP.m_iScaledRefFrmHeight < gMin(cRP.m_iRefLayerFrmHeight, cRP.m_iFrameHeight), name, "wrong crop window size");
	print_usage_and_exit(cRP.m_iLeftFrmOffset + cRP.m_iScaledRefFrmWidth  > gMax(cRP.m_iRefLayerFrmWidth, cRP.m_iFrameWidth), name, "wrong crop window size and origin");
	print_usage_and_exit(cRP.m_iTopFrmOffset + cRP.m_iScaledRefFrmHeight > gMax(cRP.m_iRefLayerFrmHeight, cRP.m_iFrameHeight), name, "wrong crop window size and origin");
}


void
resampleFrame(YuvFrame&          rcFrame,
DownConvert&       rcDownConvert,
ResizeParameters&  rcRP,
int                resamplingMethod,
int                resamplingMode,
bool               resampling,
bool               upsampling,
bool               bSecondInputFrame)
{
	//===== cropping only =====
	if (!resampling && !upsampling)
	{
		rcDownConvert.cropping(rcFrame.y.data, rcFrame.y.stride, rcFrame.u.data, rcFrame.u.stride, rcFrame.v.data, rcFrame.v.stride, &rcRP);
		return;
	}

	//===== upsampling ====
	if (upsampling)
	{
		ResizeParameters cRP = rcRP;
		{
			Int iRefVerMbShift = (cRP.m_bRefLayerFrameMbsOnlyFlag ? 4 : 5);
			Int iScaledVerShift = (cRP.m_bFrameMbsOnlyFlag ? 1 : 2);
			Int iHorDiv = (cRP.m_iRefLayerFrmWidth << 1);
			Int iVerDiv = (cRP.m_iRefLayerFrmHeight << iScaledVerShift);
			Int iRefFrmW = ((cRP.m_iRefLayerFrmWidth + (1 << 4) - 1) >> 4) << 4;  // round to next multiple of 16
			Int iRefFrmH = ((cRP.m_iRefLayerFrmHeight + (1 << iRefVerMbShift) - 1) >> iRefVerMbShift) << iRefVerMbShift;  // round to next multiple of 16 or 32 (for interlaced)
			Int iScaledRefFrmW = ((cRP.m_iScaledRefFrmWidth  * iRefFrmW + (iHorDiv >> 1)) / iHorDiv) << 1;  // scale and round to next multiple of  2
			Int iScaledRefFrmH = ((cRP.m_iScaledRefFrmHeight * iRefFrmH + (iVerDiv >> 1)) / iVerDiv) << iScaledVerShift;  // scale and round to next multiple of  2 or  4 (for interlaced)
			cRP.m_iRefLayerFrmWidth = iRefFrmW;
			cRP.m_iRefLayerFrmHeight = iRefFrmH;
			cRP.m_iScaledRefFrmWidth = iScaledRefFrmW;
			cRP.m_iScaledRefFrmHeight = iScaledRefFrmH;
		}
		if (resamplingMethod == 1)
		{
			rcDownConvert.upsamplingDyadic(rcFrame.y.data, rcFrame.y.stride, rcFrame.u.data, rcFrame.u.stride, rcFrame.v.data, rcFrame.v.stride, &rcRP);
			return;
		}
		if (resamplingMethod == 3)
		{
			rcDownConvert.upsamplingLanczos(rcFrame.y.data, rcFrame.y.stride, rcFrame.u.data, rcFrame.u.stride, rcFrame.v.data, rcFrame.v.stride, &cRP);
			return;
		}
		if (resamplingMethod == 4)
		{
			rcDownConvert.upsampling6tapBilin(rcFrame.y.data, rcFrame.y.stride, rcFrame.u.data, rcFrame.u.stride, rcFrame.v.data, rcFrame.v.stride, &cRP);
			return;
		}
		if (resamplingMode < 4)
		{
			rcDownConvert.upsamplingSVC(rcFrame.y.data, rcFrame.y.stride, rcFrame.u.data, rcFrame.u.stride, rcFrame.v.data, rcFrame.v.stride, &cRP, resamplingMode == 3);
			return;
		}
	{
		duplicateFrame(rcFrame);
		cRP.m_bRefLayerBotFieldFlag = (resamplingMode == 5);
		rcDownConvert.upsamplingSVC(rcFrame.y.data, rcFrame.y.stride, rcFrame.u.data, rcFrame.u.stride, rcFrame.v.data, rcFrame.v.stride, &cRP);
		cRP.m_bRefLayerBotFieldFlag = (resamplingMode != 5);
		rcDownConvert.upsamplingSVC(rcFrame.y.data2, rcFrame.y.stride, rcFrame.u.data2, rcFrame.u.stride, rcFrame.v.data2, rcFrame.v.stride, &cRP);
		return;
	}
	}

	//===== downsampling =====
	ResizeParameters cRP = rcRP;
	{
		Int iRefVerMbShift = (cRP.m_bRefLayerFrameMbsOnlyFlag ? 4 : 5);
		Int iScaledVerShift = (cRP.m_bFrameMbsOnlyFlag ? 1 : 2);
		Int iHorDiv = (cRP.m_iFrameWidth << 1);
		Int iVerDiv = (cRP.m_iFrameHeight << iScaledVerShift);
		Int iRefFrmW = ((cRP.m_iFrameWidth + (1 << 4) - 1) >> 4) << 4;        // round to next multiple of 16
		Int iRefFrmH = ((cRP.m_iFrameHeight + (1 << iRefVerMbShift) - 1) >> iRefVerMbShift) << iRefVerMbShift;        // round to next multiple of 16 or 32 (for interlaced)
		Int iScaledRefFrmW = ((cRP.m_iScaledRefFrmWidth  * iRefFrmW + (iHorDiv >> 1)) / iHorDiv) << 1;  // scale and round to next multiple of  2
		Int iScaledRefFrmH = ((cRP.m_iScaledRefFrmHeight * iRefFrmH + (iVerDiv >> 1)) / iVerDiv) << iScaledVerShift;  // scale and round to next multiple of  2 or  4 (for interlaced)
		cRP.m_iFrameWidth = iRefFrmW;
		cRP.m_iFrameHeight = iRefFrmH;
		cRP.m_iScaledRefFrmWidth = iScaledRefFrmW;
		cRP.m_iScaledRefFrmHeight = iScaledRefFrmH;
	}
	if (resamplingMethod == 1)
	{
		rcDownConvert.downsamplingDyadic(rcFrame.y.data, rcFrame.y.stride, rcFrame.u.data, rcFrame.u.stride, rcFrame.v.data, rcFrame.v.stride, &rcRP);
		return;
	}
	if (resamplingMode < 4)
	{
		rcDownConvert.downsamplingSVC(rcFrame.y.data, rcFrame.y.stride, rcFrame.u.data, rcFrame.u.stride, rcFrame.v.data, rcFrame.v.stride, &cRP, resamplingMode == 3);
		return;
	}
	if (!bSecondInputFrame)
	{
		cRP.m_bRefLayerBotFieldFlag = (resamplingMode == 5);
		rcDownConvert.downsamplingSVC(rcFrame.y.data, rcFrame.y.stride, rcFrame.u.data, rcFrame.u.stride, rcFrame.v.data, rcFrame.v.stride, &cRP);
	}
	else
	{
		cRP.m_bRefLayerBotFieldFlag = (resamplingMode == 4);
		rcDownConvert.downsamplingSVC(rcFrame.y.data2, rcFrame.y.stride, rcFrame.u.data2, rcFrame.u.stride, rcFrame.v.data2, rcFrame.v.stride, &cRP);
		combineTopAndBottomInFrame(rcFrame, resamplingMode == 4);
	}
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void decode_fun(int argc, char* argv[], int Thread_num, char *file_bin)
{
	TAppEncTop *cTAppEncTop = NULL;
	cTAppEncTop = new TAppEncTop[Thread_num];
	for (int ii = 0; ii < Thread_num; ii++)
	{
		kk_framesToBeEncoded = cTAppEncTop[ii].init_encode(argc, argv, ii);
		cTAppEncTop[ii].dResult = 0.0;
	}
	TAppDecTop  cTAppDecTop;
	cTAppDecTop.create();
	int deargc = 3;
	char* deargv[3] = { 0 };
	char deargv0[100] = { 0 };
	char deargv1[100] = { 0 };
	char deargv2[100] = { 0 };
	deargv[0] = deargv0;
	deargv[1] = deargv1;
	deargv[2] = deargv2;

	strcpy(deargv[1], "-b");
	strcpy(deargv[2], file_bin);
	if (!cTAppDecTop.parseCfg(deargc, deargv))
	{
		cTAppDecTop.destroy();
		return;
	}
	TComList<TComPic*>* pcListPic = NULL;
	Int poc = 0;

	ifstream mybitstreamFile(cTAppDecTop.get_m_pchBitstreamFile(), ifstream::in | ifstream::binary);
	//	cout <<"m_pchDBitstreamFile = "<<cTAppDecTop.get_m_pchBitstreamFile()<< endl;
	if (!mybitstreamFile)
	{
		fprintf(stderr, "\nfailed to open bitstream file '%s' for reading\n", cTAppDecTop.get_m_pchBitstreamFile());
		exit(EXIT_FAILURE);
	}
	InputByteStream bytestream(mybitstreamFile);
	// create & initialize internal classes
	cTAppDecTop.xCreateDecLib();
	cTAppDecTop.xInitDecLib();
	cTAppDecTop.m_iPOCLastDisplay += cTAppDecTop.get_m_iSkipFrame();      // set the last displayed POC correctly for skip forward.
	// main decoder loop
	Bool openedReconFile = false; // reconstruction file not yet opened. (must be performed after SPS is seen)
	Bool loopFiltered = false;

	int codedframenun = 0;
	bool mystop = true;
	while (!!mybitstreamFile && mystop)
	{
		streampos location = mybitstreamFile.tellg();
		AnnexBStats stats = AnnexBStats();

		vector<uint8_t> nalUnit;
		InputNALUnit nalu;
		byteStreamNALUnit(bytestream, nalUnit, stats);

		// call actual decoding function
		Bool bNewPicture = false;

		if (nalUnit.empty())
		{
			fprintf(stderr, "Warning: Attempt to decode an empty NAL unit\n");
		}
		else
		{
			read(nalu, nalUnit);
			if ((cTAppDecTop.get_m_iMaxTemporalLayer() >= 0 && nalu.m_temporalId > cTAppDecTop.get_m_iMaxTemporalLayer()) || !cTAppDecTop.isNaluWithinTargetDecLayerIdSet(&nalu))
			{
				bNewPicture = false;
			}
			else
			{
				//bNewPicture = m_cTDecTop.decode(nalu, m_iSkipFrame, m_iPOCLastDisplay);
				//WJ修改成如下代码  
				Int my_m_iSkipFrame = cTAppDecTop.get_m_iSkipFrame();
				bNewPicture = cTAppDecTop.m_cTDecTop.decode(nalu, my_m_iSkipFrame, cTAppDecTop.m_iPOCLastDisplay);
				if (bNewPicture)
				{
					mybitstreamFile.clear();
					mybitstreamFile.seekg(location - streamoff(3));
					bytestream.reset();
				}
			}
		}
		if (bNewPicture || !mybitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS)
		{
			if (!loopFiltered)
			{
				cTAppDecTop.m_cTDecTop.executeLoopFilters(poc, pcListPic);
			}
			loopFiltered = (nalu.m_nalUnitType == NAL_UNIT_EOS);
		}
		if (pcListPic)
		{
			if (cTAppDecTop.get_m_pchReconFile() && !openedReconFile)
			{
				if (!cTAppDecTop.get_m_outputBitDepthY()) { cTAppDecTop.set_m_outputBitDepthY(g_bitDepthY); }
				if (!cTAppDecTop.get_m_outputBitDepthC()) { cTAppDecTop.set_m_outputBitDepthC(g_bitDepthC); }
				cTAppDecTop.m_cTVideoIOYuvReconFile.open(cTAppDecTop.get_m_pchReconFile(), true, cTAppDecTop.get_m_outputBitDepthY(), cTAppDecTop.get_m_outputBitDepthC(), g_bitDepthY, g_bitDepthC); // write mode
				openedReconFile = true;
			}
			if (bNewPicture)
			{
				cTAppDecTop.xWriteOutput(pcListPic, nalu.m_temporalId);
			}
			if ((bNewPicture || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA) && cTAppDecTop.m_cTDecTop.getNoOutputPriorPicsFlag())
			{
				cTAppDecTop.m_cTDecTop.checkNoOutputPriorPics(pcListPic);
				cTAppDecTop.m_cTDecTop.setNoOutputPriorPicsFlag(false);
			}
			if (bNewPicture &&
				(nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL
				|| nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP
				|| nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP
				|| nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL
				|| nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP))
			{
				cTAppDecTop.xFlushOutput(pcListPic);
			}
			if (nalu.m_nalUnitType == NAL_UNIT_EOS)
			{
				cTAppDecTop.xWriteOutput(pcListPic, nalu.m_temporalId);
			}
			if (!bNewPicture && nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL_N && nalu.m_nalUnitType <= NAL_UNIT_RESERVED_VCL31)
			{
				cTAppDecTop.xWriteOutput(pcListPic, nalu.m_temporalId);
			}
		}
		if (bNewPicture || !mybitstreamFile)
		{
			if (pcListPic->empty())
			{
				return;
			}

			TComList<TComPic*>::iterator iterPic = pcListPic->end();
			iterPic--;
			pcListPicDec.push_back(*(iterPic));
			TComList<TComPic*>::iterator iter = pcListPicDec.end();
			iter--;
			mypcPic = *(iter);
#if	RRT
			depth_all_Dec = new int *[4 * 8];
			for (int i = 0; i < 4 * 8; i++)
			{
				depth_all_Dec[i] = new int[7 * 8];
			}
#endif
#if resolution_ratio_transcoding
			TComPicYuv* pcPicYuvOrg = mypcPic->getPicYuvRec();
			ResizeParameters *cRP = new ResizeParameters[Thread_num];
#endif
			for (int jj = 0; jj < Thread_num; jj++)
			{
#if resolution_ratio_transcoding
				//===== set standard resize parameters =====
				cRP[jj].m_bRefLayerFrameMbsOnlyFlag = true;
				cRP[jj].m_bFrameMbsOnlyFlag =  true;
				cRP[jj].m_bRefLayerFieldPicFlag = false;
				cRP[jj].m_bFieldPicFlag = false;
				cRP[jj].m_bRefLayerBotFieldFlag = false;
				cRP[jj].m_bBotFieldFlag = false;
				cRP[jj].m_bRefLayerIsMbAffFrame = false;
				cRP[jj].m_bIsMbAffFrame = false;
				cRP[jj].m_iRefLayerChromaPhaseX = -1;
				cRP[jj].m_iRefLayerChromaPhaseY = 0;
				cRP[jj].m_iChromaPhaseX = -1;
				cRP[jj].m_iChromaPhaseY = 0;
				cRP[jj].m_iRefLayerFrmWidth = 0;
				cRP[jj].m_iRefLayerFrmHeight = 0;
				cRP[jj].m_iScaledRefFrmWidth = 0;
				cRP[jj].m_iScaledRefFrmHeight = 0;
				cRP[jj].m_iFrameWidth = 0;
				cRP[jj].m_iFrameHeight = 0;
				cRP[jj].m_iLeftFrmOffset = 0;
				cRP[jj].m_iTopFrmOffset = 0;
				cRP[jj].m_iExtendedSpatialScalability = 0;
				cRP[jj].m_iLevelIdc = 0;

				//===== init parameters =====
				FILE* inputFile = 0;
				FILE* outputFile = 0;
				FILE* croppingParametersFile = 0;
				int   resamplingMethod = 0;
				int   resamplingMode = 0;
				bool  croppingInitialized = false;
				bool  phaseInitialized = false;
				bool  methodInitialized = false;
				bool  resampling = false;
				bool  upsampling = false;
				int   numSpatialDyadicStages = 0;
				int   skipBetween = 0;
				int   skipAtStart = 0;
				int   maxNumOutputFrames = 0;

				//===== read input parameters =====
				cRP[jj].m_iRefLayerFrmWidth = pcPicYuvOrg->getWidth();
				cRP[jj].m_iRefLayerFrmHeight = pcPicYuvOrg->getHeight();
				cRP[jj].m_iFrameWidth = cTAppEncTop[jj].get_m_iSourceWidth();
				cRP[jj].m_iFrameHeight = cTAppEncTop[jj].get_m_iSourceHeight();
				if (!methodInitialized)
				{
					resampling = true;
					upsampling = (cRP[jj].m_iRefLayerFrmWidth < cRP[jj].m_iFrameWidth) || (cRP[jj].m_iRefLayerFrmHeight < cRP[jj].m_iFrameHeight);
				}
				if (!croppingInitialized)
				{
					if (resamplingMethod == 2)
					{
						cRP[jj].m_iScaledRefFrmWidth = gMin(cRP[jj].m_iRefLayerFrmWidth, cRP[jj].m_iFrameWidth);
						cRP[jj].m_iScaledRefFrmHeight = gMin(cRP[jj].m_iRefLayerFrmHeight, cRP[jj].m_iFrameHeight);
					}
					else
					{
						cRP[jj].m_iScaledRefFrmWidth = gMax(cRP[jj].m_iRefLayerFrmWidth, cRP[jj].m_iFrameWidth);
						cRP[jj].m_iScaledRefFrmHeight = gMax(cRP[jj].m_iRefLayerFrmHeight, cRP[jj].m_iFrameHeight);
					}
				}

				//===== set basic parameters for resampling control =====
				if (resamplingMethod == 0)
				{
					if (resamplingMode == 1)
					{
						cRP[jj].m_bRefLayerFrameMbsOnlyFlag = false;
						cRP[jj].m_bFrameMbsOnlyFlag = false;
					}
					else if (resamplingMode == 2 || resamplingMode == 3)
					{
						cRP[jj].m_bFrameMbsOnlyFlag = false;
						if (!upsampling)
						{
							cRP[jj].m_bFieldPicFlag = true;
							cRP[jj].m_bBotFieldFlag = (resamplingMode == 3);
						}
					}
					else if (resamplingMode == 4 || resamplingMode == 5)
					{
						cRP[jj].m_bRefLayerFrameMbsOnlyFlag = false;
						cRP[jj].m_bRefLayerFieldPicFlag = true;
					}
				}
				//===== initialize classes =====
				YuvFrame    cFrame;
				DownConvert cDownConvert;
				{
					int maxWidth = gMax(cRP[jj].m_iRefLayerFrmWidth, cRP[jj].m_iFrameWidth);
					int maxHeight = gMax(cRP[jj].m_iRefLayerFrmHeight, cRP[jj].m_iFrameHeight);
					int minWidth = gMin(cRP[jj].m_iRefLayerFrmWidth, cRP[jj].m_iFrameWidth);
					int minHeight = gMin(cRP[jj].m_iRefLayerFrmHeight, cRP[jj].m_iFrameHeight);
					int minWRnd16 = ((minWidth + 15) >> 4) << 4;
					int minHRnd32 = ((minHeight + 31) >> 5) << 5;
					maxWidth = ((maxWidth  * minWRnd16 + (minWidth << 4) - 1) / (minWidth << 4)) << 4;
					maxHeight = ((maxHeight * minHRnd32 + (minHeight << 4) - 1) / (minHeight << 4)) << 4;
					createFrame(cFrame, maxWidth, maxHeight);
					cDownConvert.init(maxWidth, maxHeight);
				}
//				printf("Resampler\n\n");
				//===== loop over frames =======
				int skip = skipAtStart;
				int writtenFrames = 0;
				int   numInputFrames = (resamplingMode >= 4 && !upsampling ? 2 : 1);
				int   numOutputFrames = (resamplingMode >= 4 && upsampling ? 2 : 1);
				bool  bFinished = false;
				bFinished = (readFrame(cFrame, pcPicYuvOrg, cRP[jj].m_iRefLayerFrmWidth, cRP[jj].m_iRefLayerFrmHeight) != 0);
				skip = skipBetween;
				/*
				if (cRP[jj].m_iExtendedSpatialScalability == 2 && !bFinished)
				{
//					updateCropParametersFromFile();
				}*/
				//===== set resampling parameter =====
				if (resamplingMethod != 0 &&
					cRP[jj].m_iScaledRefFrmWidth == gMin(cRP[jj].m_iRefLayerFrmWidth, cRP[jj].m_iFrameWidth) &&
					cRP[jj].m_iScaledRefFrmHeight == gMin(cRP[jj].m_iRefLayerFrmHeight, cRP[jj].m_iFrameHeight))
				{
					resampling = false;
				}
				else
				{
					resampling = true;
				}
				//===== resample input frame =====
				if (!bFinished)
				{
					resampleFrame(cFrame, cDownConvert, cRP[jj], resamplingMethod, resamplingMode, resampling, upsampling, false);
				}
#else
				YuvFrame    cFrame;
#endif
				long lBefore = clock();
				cTAppEncTop[jj].encode(cFrame);
				cTAppEncTop[jj].dResult += (double)(clock() - lBefore) / CLOCKS_PER_SEC;

#if resolution_ratio_transcoding
				deleteFrame(cFrame);
#endif
			}
#if resolution_ratio_transcoding
			delete[] cRP;
#endif
			codedframenun++;
			if (codedframenun < kk_framesToBeEncoded)
			{
				mystop = true;
			}
			else
			{
				mystop = false;
			}
			printf("\n");
		}
	}
	for (int kk = 0; kk < Thread_num; kk++)
	{
		printf("\n Total Time: %12.3f sec.\n", cTAppEncTop[kk].dResult);
		cTAppEncTop[kk].destroy_encode();
	}
#if RRT
	for (int i = 0; i < 4 * 8; i++)
	{
		delete[] depth_all_Dec[i];
	}
	delete[] depth_all_Dec;
#endif
	delete[]cTAppEncTop;
	cTAppDecTop.m_cTDecTop.deletePicBuffer();
	cTAppDecTop.xDestroyDecLib();
	cTAppDecTop.destroy();
}

int main(int argc, char* argv[])
{
	int Thread_num = atoi(argv[1]);
	double dResult = 0.0;
	long lBefore = clock();
	decode_fun(argc, argv, Thread_num, argv[2]);
	dResult = (double)(clock() - lBefore) / CLOCKS_PER_SEC;
	printf("\n  Together Total Time: %12.3f sec.\n", dResult);
	mypcPic = NULL;

//	_CrtDumpMemoryLeaks();
	return 0;
}

//! \}
