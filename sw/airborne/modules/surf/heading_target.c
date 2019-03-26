#include <stdio.h>
#include <stdlib.h>

int i;

float xxx[8] = {1, 2, 3, 4, 5, 5, 6, 7};
float lol = 0;

void main()
{
  for (i = 0; i < sizeof(xxx)/sizeof(xxx[0]); i++)
  {
    lol += xxx[i];
  }
  float xpos_av = lol/(sizeof(xxx)/sizeof(xxx[0]));


  printf("%f", xpos_av);
}