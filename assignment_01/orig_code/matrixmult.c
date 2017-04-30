#include <stdio.h>
#include "Timer.h"

#define MAXSIZE 128

void matMult(int mat1[MAXSIZE][MAXSIZE], int mat2[MAXSIZE][MAXSIZE], int prod[MAXSIZE][MAXSIZE], int size);

int main(int argc, char** argv)
{
	int size;
	if (argc < 2)
		size = 16;	// default option
	else
		size = atoi(argv[1]);
	if (size > 128) {
		printf("Max size exceeded, size set to 128\n");
		size = 128;
	}

    Timer totalTime;
    initTimer(&totalTime, "Total Time");

	int mat1[MAXSIZE][MAXSIZE], mat2[MAXSIZE][MAXSIZE], prod[MAXSIZE][MAXSIZE];
	int i, j;
	
	for (i = 0;i < size; i++)
	{
		for (j = 0; j < size; j++)
		{
			mat1[i][j] = i+j*2;
		}
	}
	
	for(i = 0; i < size; i++)
	{
		for (j = 0; j < size; j++)
		{
			mat2[i][j] = i+j*3;
		}
	}

    startTimer(&totalTime);
	matMult(mat1,mat2,prod,size);
    stopTimer(&totalTime);
    printTimer(&totalTime);	

	for (i = 0;i < size; i++)
	{
		printf("\n");
		for (j = 0; j < size; j++)
		{
			printf("\t%d ", prod[i][j]);
		}
	}
	
	printf("\nDone !!! \n");
	return 0;
}

void matMult(int mat1[MAXSIZE][MAXSIZE], int mat2[MAXSIZE][MAXSIZE], int prod[MAXSIZE][MAXSIZE], int size)
{
	int i, j, k;
	for (i = 0;i < size; i++)
	{
		for (j = 0; j < size; j++)
		{
			prod[i][j]=0;
			for(k=0;k<size;k++)
				prod[i][j] = prod[i][j]+mat1[i][k] * mat2[k][j];
		}
	}
}
