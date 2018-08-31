#include <cstdio>
#include <cstdlib>
#include <vector>

//g++ -g vectorbounds.cpp


int main(void) {
  int a;
  int b;
  std::vector<double> first_vector;
  std::vector<double> second_vector;

  printf("What is the length of the first vector\n");
  scanf("%i", &a);

  printf("What is the length of the second vector\n");
  scanf("%i", &b);

  first_vector.resize(a);
  second_vector.resize(b);

  printf("first vector: ");
  for (int idx=0; idx < a; idx++) {
    first_vector[idx] = idx;
    printf("%i ", idx);
  }
  printf("\n");

  printf("Second vector: ");
  for (int idx=0; idx < a; idx++) {
    second_vector[idx] = 10*rand();
    printf("%3.2f ", second_vector[idx]);
  }
  printf("\n");

  printf("Summing\n");

  int sum=0;
  for (int idx=0; idx <= a; idx++) {
    sum += first_vector[idx];
  }
  printf("sum of first vector is %i\n", sum);

  sum = 0;
  for (int idx=0; idx <= a; idx++) {
    sum+= second_vector[idx];
  }
  printf("sum of second vector is %i\n", sum);
}
