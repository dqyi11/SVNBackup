#include

void _init()
{
    printf("Initialization of test shared object\n");
}

void _fini()
{
    printf("Clean up of test shared object\n");
}

extern "C" int* function(int x, int y)
{
	int* information = new int[10];
	for(int k=0;k<10;k++)
	{
	    information[k] = k;
	}
	return information;
}