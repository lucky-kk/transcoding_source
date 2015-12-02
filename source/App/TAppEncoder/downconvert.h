#ifndef _DOWN_CONVERT_
#define _DOWN_CONVERT_

#include "DC_typedefs.h"
#include "DC_macros.h"
//#include "ResizeParameters.h"

#define DOWN_CONVERT_STATIC 1

#ifdef DOWN_CONVERT_STATIC

#ifndef  gMax
#define  gMax(x,y)   ((x)>(y)?(x):(y))
#define  gMin(x,y)   ((x)<(y)?(x):(y))
#endif

#else

H264AVC_NAMESPACE_BEGIN

enum MbMapEntry
{
	INVALID_ENTRY = 0x00,
	BASE_MODE_ALLOWED = 0x01,
	INTRA_UPS_ALLOWED = 0x02
};

#endif

__inline Int CeilLog2(Int i)
{
	Int s = 0; i--;
	while (i > 0)
	{
		s++;
		i >>= 1;
	}
	return s;
}

typedef struct
{
	int            stride;
	int            lines;
	unsigned char* data;
	unsigned char* data2;
} ColorComponent;

typedef struct
{
	ColorComponent y;
	ColorComponent u;
	ColorComponent v;
} YuvFrame;

class ResizeParameters
{
public:
	ResizeParameters()
		: m_iExtendedSpatialScalability(0)
		, m_iLevelIdc(0)
		, m_bFrameMbsOnlyFlag(true)
		, m_bFieldPicFlag(false)
		, m_bBotFieldFlag(false)
		, m_bIsMbAffFrame(false)
		, m_iFrameWidth(0)
		, m_iFrameHeight(0)
		, m_iWidthInSamples(0)
		, m_iHeightInSamples(0)
		, m_iChromaPhaseX(0)
		, m_iChromaPhaseY(0)
		, m_iScaledRefFrmWidth(0)
		, m_iScaledRefFrmHeight(0)
		, m_iLeftFrmOffset(0)
		, m_iTopFrmOffset(0)
		, m_iRefLayerChromaPhaseX(0)
		, m_iRefLayerChromaPhaseY(0)
		, m_bRefLayerFrameMbsOnlyFlag(true)
		, m_bRefLayerFieldPicFlag(false)
		, m_bRefLayerBotFieldFlag(false)
		, m_bRefLayerIsMbAffFrame(false)
		, m_iRefLayerFrmWidth(0)
		, m_iRefLayerFrmHeight(0)
		, m_iRefLayerWidthInSamples(0)
		, m_iRefLayerHeightInSamples(0)
	{
	};
public:
	//===== parameters of current layer =====
	Int   m_iExtendedSpatialScalability;
	Int   m_iLevelIdc;
	Bool  m_bFrameMbsOnlyFlag;
	Bool  m_bFieldPicFlag;
	Bool  m_bBotFieldFlag;
	Bool  m_bIsMbAffFrame;
	Int   m_iFrameWidth;
	Int   m_iFrameHeight;
	Int   m_iWidthInSamples;
	Int   m_iHeightInSamples;
	Int   m_iChromaPhaseX;
	Int   m_iChromaPhaseY;
	Int   m_iScaledRefFrmWidth;     // also in PictureParameters
	Int   m_iScaledRefFrmHeight;    // also in PictureParameters
	Int   m_iLeftFrmOffset;         // also in PictureParameters
	Int   m_iTopFrmOffset;          // also in PictureParameters
	Int   m_iRefLayerChromaPhaseX;  // also in PictureParameters
	Int   m_iRefLayerChromaPhaseY;  // also in PictureParameters

	//===== parameters for base layer =====
	Bool  m_bRefLayerFrameMbsOnlyFlag;
	Bool  m_bRefLayerFieldPicFlag;
	Bool  m_bRefLayerBotFieldFlag;
	Bool  m_bRefLayerIsMbAffFrame;
	Int   m_iRefLayerFrmWidth;
	Int   m_iRefLayerFrmHeight;
	Int   m_iRefLayerWidthInSamples;
	Int   m_iRefLayerHeightInSamples;
};

class DownConvert
{
public:
	//========================
	// general main functions
	//========================
	DownConvert();
	~DownConvert();
	bool  init(int iMaxWidth, int iMaxHeight, int iMaxMargin = 0);
	void  destroy();

#ifdef  DOWN_CONVERT_STATIC
	//=====================================
	// main functions for DownConvert Tool
	//=====================================
	void  cropping(unsigned char*          pucBufferY, int   iStrideY,
		unsigned char*          pucBufferU, int   iStrideU,
		unsigned char*          pucBufferV, int   iStrideV,
		ResizeParameters*       pcParameters);
	void  upsamplingDyadic(unsigned char*          pucBufferY, int   iStrideY,
		unsigned char*          pucBufferU, int   iStrideU,
		unsigned char*          pucBufferV, int   iStrideV,
		ResizeParameters*       pcParameters);
	void  upsamplingLanczos(unsigned char*          pucBufferY, int   iStrideY,
		unsigned char*          pucBufferU, int   iStrideU,
		unsigned char*          pucBufferV, int   iStrideV,
		ResizeParameters*       pcParameters);
	void  upsampling6tapBilin(unsigned char*          pucBufferY, int   iStrideY,
		unsigned char*          pucBufferU, int   iStrideU,
		unsigned char*          pucBufferV, int   iStrideV,
		ResizeParameters*       pcParameters);
	void  upsamplingSVC(unsigned char*          pucBufferY, int   iStrideY,
		unsigned char*          pucBufferU, int   iStrideU,
		unsigned char*          pucBufferV, int   iStrideV,
		ResizeParameters*       pcParameters, bool  bBotCoincided = false);
	void  downsamplingDyadic(unsigned char*          pucBufferY, int   iStrideY,
		unsigned char*          pucBufferU, int   iStrideU,
		unsigned char*          pucBufferV, int   iStrideV,
		ResizeParameters*       pcParameters);
	void  downsamplingSVC(unsigned char*          pucBufferY, int   iStrideY,
		unsigned char*          pucBufferU, int   iStrideU,
		unsigned char*          pucBufferV, int   iStrideV,
		ResizeParameters*       pcParameters, bool  bBotCoincided = false);
#else
	//====================================
	// main functions for encoder/decoder
	//====================================
	void  intraUpsampling(Frame*                  pcFrame,
		Frame*                  pcBaseFrame,
		Frame*                  pcTempFrame,
		Frame*                  pcTempBaseFrame,
		ResizeParameters*       pcParameters,
		MbDataCtrl*             pcMbDataCtrlBase,
		MbDataCtrl*             pcMbDataCtrlPredFrm,
		MbDataCtrl*             pcMbDataCtrlPredFld,
		ReconstructionBypass*   pcReconstructionBypass,
		Bool*                   pabBaseModeAllowedFlagArrayFrm,
		Bool*                   pabBaseModeAllowedFlagArrayFld,
		Bool                    bConstrainedIntraResamplingFlag);
	void  residualUpsampling(Frame*                  pcFrame,
		Frame*                  pcBaseFrame,
		ResizeParameters*       pcParameters,
		MbDataCtrl*             pcMbDataCtrlBase);
#endif


	//==========================
	// general helper functions
	//==========================
	//--- delete buffers ---
	void  xDestroy();
	//--- general clipping ---
	int   xClip(int                     iValue,
		int                     imin,
		int                     imax);
	//--- SVC normative intra upsampling ---
	void  xCompIntraUpsampling(ResizeParameters*       pcParameters,
		bool                    bChroma,
		bool                    bBotFlag,
		bool                    bVerticalInterpolation,
		bool                    bFrameMb,
		int                     iMargin = 0);
	void  xVertIntraUpsampling(int  iBaseW, int  iBaseH,
		int  iLOffset, int  iTOffset, int  iROffset, int  iBOffset,
		int  iYBorder, bool bBotFlag, bool bFrameMb, bool bChromaFilter);
	void  xBasicIntraUpsampling(int  iBaseW, int  iBaseH, int  iCurrW, int  iCurrH,
		int  iLOffset, int  iTOffset, int  iROffset, int  iBOffset,
		int  iShiftX, int  iShiftY, int  iScaleX, int  iScaleY,
		int  iOffsetX, int  iOffsetY, int  iAddX, int  iAddY,
		int  iDeltaX, int  iDeltaY, int  iYBorder, bool bChromaFilter, int iMargin);

#ifdef  DOWN_CONVERT_STATIC
	//=======================================
	// helper functions for DownConvert Tool
	//=======================================
	//--- initialization ---
	void  xInitLanczosFilter();
	//--- place to and get from image buffer ---
	void  xCopyToImageBuffer(unsigned char*        pucSrc,
		int                   iWidth,
		int                   iHeight,
		int                   iStride);
	void  xCopyFromImageBuffer(unsigned char*        pucDes,
		int                   iWidth,
		int                   iHeight,
		int                   iStride);
	void  xInitializeWithValue(unsigned char*        pucBuffer,
		int                   iWidth,
		int                   iHeight,
		int                   iStride,
		unsigned char         cValue);
	//--- dyadic upsampling ---
	void  xCompUpsamplingDyadic(int                   iBaseW,
		int                   iBaseH,
		bool                  bChroma);
	//--- Lanczos upsampling ---
	void  xCompUpsamplingLanczos(ResizeParameters*     pcParameters,
		bool                  bChroma);
	void  xUpsamplingDataLanczos(int                   iInLength,
		int                   iOutLength,
		long                  spos);
	void  xGetNumDenomLanczos(int                   iInWidth,
		int                   iOutWidth,
		int&                  riNumerator,
		int&                  riDenominator);
	long  xGetFilterLanczos(long                  x);
	//--- 6-tap + bilinear upsampling ---
	void  xCompUpsampling6tapBilin(ResizeParameters*     pcParameters,
		bool                  bChroma);
	void  xUpsamplingData6tapBilin(int                   iInLength,
		int                   iOutLength);
	//--- dyadic downsampling ---
	void  xCompDownsamplingDyadic(int                   iCurrW,
		int                   iCurrH);
	//--- SVC non-normative downsampling ---
	void  xCompDownsampling(ResizeParameters*     pcParameters,
		bool                  bChroma,
		bool                  bBotFlag,
		bool                  bVerticalDownsampling);
	void  xVertDownsampling(int                   iBaseW,
		int                   iBaseH,
		bool                  bBotFlag);
	void  xBasicDownsampling(int  iBaseW, int  iBaseH, int  iCurrW, int  iCurrH,
		int  iLOffset, int  iTOffset, int  iROffset, int  iBOffset,
		int  iShiftX, int  iShiftY, int  iScaleX, int  iScaleY,
		int  iAddX, int  iAddY, int  iDeltaX, int  iDeltaY);
#else
	//======================================
	// helper functions for encoder/decoder
	//======================================
	//--- place to and get from image buffer ---
	void  xCopyToImageBuffer(const short*          psSrc,
		int                   iWidth,
		int                   iHeight,
		int                   iStride,
		int                   iMargin = 0);
	void  xCopyFromImageBuffer(short*                psDes,
		int                   iWidth,
		int                   iHeight,
		int                   iStride);
	void  xInitializeWithValue(short*                psBuffer,
		int                   iWidth,
		int                   iHeight,
		int                   iStride,
		short                 iValue);
	//--- cropping ---
	void  xCrop(Frame*                pcFrame,
		Frame*                pcBaseFrame,
		ResizeParameters*     pcParameters,
		short                 iValue);
	//--- SVC intra upsampling ---
	void  xIntraUpsampling(Frame*                pcFrame,
		Frame*                pcBaseFrame,
		ResizeParameters*     pcParameters);
	void  xInitSliceIdList(MyList<unsigned int>& rcSliceIdList,
		ResizeParameters*     pcParameters,
		MbDataCtrl*           pcMbDataCtrl);
	void  xInitBaseModeAllowedFlags(ResizeParameters*     pcParameters,
		bool*                 pabBaseModeAllowedFlagArrayFrm,
		bool*                 pabBaseModeAllowedFlagArrayFld);
	void  xUpdateBaseModeAllowedFlags(ResizeParameters*     pcParameters,
		bool*                 pabBaseModeAllowedFlagArrayFrm,
		bool*                 pabBaseModeAllowedFlagArrayFld);
	void  xInitIntraUpsAvailFlags(ResizeParameters*     pcParameters);
	void  xUpdateIntraUpsAvailFlags(ResizeParameters*     pcParameters);
	void  xUpdateBaseModeFlagsIntraUps(ResizeParameters*     pcParameters,
		MbDataCtrl*           pcMbDataCtrlPredFrm,
		MbDataCtrl*           pcMbDataCtrlPredFld,
		bool*                 pabBaseModeAllowedFlagArrayFrm,
		bool*                 pabBaseModeAllowedFlagArrayFld);
	void  xGenerateMbMapsForSliceId(ResizeParameters*     pcParameters,
		MbDataCtrl*           pcMbDataCtrlBase,
		MbDataCtrl*           pcMbDataCtrlPredFrm,
		MbDataCtrl*           pcMbDataCtrlPredFld,
		unsigned int          uiCurrentSliceId);
	void  xInitMbMaps(ResizeParameters*     pcParameters,
		bool                  bFrm,
		bool                  bTop,
		bool                  bBot);
	void  xUpdateMbMapForSliceId(ResizeParameters*     pcParameters,
		bool                  bChroma,
		bool                  bFieldMb,
		MbDataCtrl*           pcMbDataCtrlBase,
		MbDataCtrl*           pcMbDataCtrlPredFrm,
		MbDataCtrl*           pcMbDataCtrlPredFld,
		unsigned int          uiCurrentSliceId);
	void  xUpdateIntraPredFrame(Frame*                pcDesFrame,
		Frame*                pcSrcFrame,
		ResizeParameters*     pcParameters);
	//--- SVC residual upsampling ---
	void  xResidualUpsampling(Frame*              pcFrame,
		Frame*              pcBaseFrame,
		ResizeParameters*   pcParameters,
		MbDataCtrl*         pcMbDataCtrlBase);
	void  xDetermineTransBlkIdcs(int                 iBaseW,
		int                 iBaseH,
		bool                bChroma,
		bool                bBotField,
		ResizeParameters*   pcRP,
		MbDataCtrl*         pcMbDataCtrlBase);
	void  xCompResidualUpsampling(ResizeParameters*   pcParameters,
		bool                bChroma,
		bool                bBotFlag,
		bool                bVerticalInterpolation,
		MbDataCtrl*         pcMbDataCtrlBase);
	void  xVertResidualUpsampling(int  iBaseW, int  iBaseH,
		int  iLOffset, int  iTOffset, int  iROffset, int  iBOffset,
		int  iYBorder, bool bBotFlag);
	void  xBasicResidualUpsampling(int  iBaseW, int  iBaseH, int  iCurrW, int  iCurrH,
		int  iLOffset, int  iTOffset, int  iROffset, int  iBOffset,
		int  iShiftX, int  iShiftY, int  iScaleX, int  iScaleY,
		int  iOffsetX, int  iOffsetY, int  iAddX, int  iAddY,
		int  iDeltaX, int  iDeltaY, int  iYBorder);
#endif

private:
	//===== member variables =====
	int         m_iImageStride;
	int*        m_paiImageBuffer;
	int*        m_paiTmp1dBuffer;
#ifdef  DOWN_CONVERT_STATIC
	long*       m_padFilter;
	int*        m_aiTmp1dBufferInHalfpel;
	int*        m_aiTmp1dBufferInQ1pel;
	int*        m_aiTmp1dBufferInQ3pel;
	int*        m_paiTmp1dBufferOut;
#else
	int*        m_paiTransBlkIdc;
	int         m_iMbMapStride;
	MbMapEntry* m_paeMbMapFrm;
	MbMapEntry* m_paeMbMapFld;
	bool*       m_pabIntraUpsAvailableFrm;
	bool*       m_pabIntraUpsAvailableFld;
#endif
};


#ifdef DOWN_CONVERT_STATIC
#else
H264AVC_NAMESPACE_END
#endif


#endif // _DOWN_CONVERT_

