#include <iostream>
#include <string>
#include <cstring>

double* convertStrtoArr4x4(char* str)
{
  // Initialize Array
  double array1[4][4];

  // Iterator to access array
  int i = 0, j = 0;


  char s[2] = ",";
  char *token;

  // get the first token
  token = strtok(str, s);

  // Iterates through all tokens
  while ( token != NULL ) {
    array1[j][i] = std::stof(token);

    // iterates through the array
    i += 1;
    if (i == 4) {
      i = 0;
      j += 1;
    }

    // Goes to next token
    token = strtok(NULL, s);
  }

  std::cout << "arr[] = ";
  for (int m = 0; m <= 3; m++) {
    for (int n = 0; n <= 3; n++) {
       std::cout << array1[m][n] << " ";
    }
  }

  return *array1;
}

int main()
{ 
  char str[] ="1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16";
  double *out_arr = convertStrtoArr4x4(str);
  return 0;
}
