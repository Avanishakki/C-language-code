#include<stdio.h>
#include<stdlib.h>
int *test(int);
int main()
{
	int x=10;
	int *ptr;
	ptr=test(x);
	printf("ptr=%p\n",ptr);
	return 0;
}	
int *test(int x){	
	int y;
	int *ptr=(int *)malloc(sizeof(int)); //allot memory on heap

	*ptr=&y;
	y=x*x;

	return ptr; 
}
