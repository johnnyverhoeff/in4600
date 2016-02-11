#define MAX_DIMENSION 8

uint8_t start_chip = 0;

uint8_t matrix[MAX_DIMENSION][MAX_DIMENSION];

void print_matrix() {
  Serial.println("*******");
  for (int i = 0; i < MAX_DIMENSION; i++) {
    for (int j = 0; j < MAX_DIMENSION; j++) {
      Serial.print(matrix[i][j]);
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println("*******");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Begun!");

  matrix[0][0] = start_chip;


  // inspiration taken from: 
  http://stackoverflow.com/questions/18604659/hadamard-matrix-code
  
  for (int i = 2; i <= MAX_DIMENSION; i *= 2) {
    for (int x = 0; x < (i / 2); x++) {
      for (int y = i / 2; y < i; y++) {
        matrix[x][y] = matrix[x][y - i / 2];
      }
    }

    for (int y = 0; y < (i / 2); y++) {
      for (int x = i / 2; x < i; x++) {
        matrix[x][y] = matrix[x - (i / 2)][y];
      }
    }

    for (int x = i / 2; x < i; x++) {
      for (int y = i / 2; y < i; y++){
        matrix[x][y] = !matrix[x - i / 2][y - i / 2];

      }
    }
  }

  print_matrix();

  for (int i = 0; i < MAX_DIMENSION; i++) {
    Serial.print(matrix[MAX_DIMENSION - 3][i]);
    Serial.print(" ");
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
