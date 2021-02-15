#include <vector>
#include "impro_util.h"

int FuncTryCamFocusExposure(int argc, char ** argv)
{
	std::vector<double> result;
	tryOpcvCapFocusExposure(result);
	printf("The final set is:\n");
	printf("Camera %d: Focus:%7.2f, Exposure : %7.2f, Width : % 4d, Height : % 4d.\n",
		(int)result[0],
		result[1], result[2],
		(int)result[3], (int)result[4]); 
	return 0;
}
