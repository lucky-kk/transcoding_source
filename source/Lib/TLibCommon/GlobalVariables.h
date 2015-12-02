#ifndef __GLOBAL_VARIABLES__
#define __GLOBAL_VARIABLES__

#include "TLibCommon/TComSlice.h"
#include "TLibCommon/TComPicYuv.h"
#include "TLibCommon/TComMv.h"

extern TComList<TComPic*>  pcListPicDec;
extern TComList<TComPic*>  pcListPicRec;
extern TComPic* mypcPic;
extern TComPic* pcPicDec;
extern TComPic* pcPicRec;
extern int kk_framesToBeEncoded;
extern int myuiPartUnitIdx;
extern int **depth_all_Dec;

#endif // __GLOBAL_VARIABLES__
