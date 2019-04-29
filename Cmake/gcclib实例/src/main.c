#include "addnum.h"
#include<stdio.h>
int main(int argc, char ** argv)
{
	if(argc<3)
	{
		printf("please input two string\n");
		return 1;
	}
	int a=10;
	int b=5;
	int sum = add(a,b);
	printf("%s+%s=%d\n",argv[1],argv[2],sum);
	return 0;
}
