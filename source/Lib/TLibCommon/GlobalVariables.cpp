#include "GlobalVariables.h"

TComList<TComPic*> pcListPicDec;
TComList<TComPic*> pcListPicRec;
TComPic* mypcPic = NULL;
TComPic* pcPicDec = NULL;
TComPic* pcPicRec = NULL;
int kk_framesToBeEncoded = 0;
int myuiPartUnitIdx = 0;
int **depth_all_Dec = NULL;