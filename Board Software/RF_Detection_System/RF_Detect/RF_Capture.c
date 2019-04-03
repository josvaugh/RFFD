#include <stdio.h>
#include <stdlib.h>

int main()
{
	char buffer[6];
	FILE *fptr;
	while(1)
	{

		system("sh ~/Desktop/RF_Detect/rfdetect");

		if((fptr = fopen("rf_strength", "r")) == NULL)
		{
			printf("Error Opening File!");
			exit(1);
		}

		fscanf(fptr, "%[^\n]", buffer);
		fclose(fptr);
	}
	return 0;
}
