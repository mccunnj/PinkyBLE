int values[72];
char output[216];
int valuesout[72];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial)
    delay(1000);
  for(int i = 0; i < (3*3*8); i++){
    values[i] = random(50,700);
  }
  for(int i = 0; i < (3*3*8); i++){
    Serial.print(i);
    Serial.print(": ");
    Serial.println(values[i]);
    //Convert array of ints into array of chars
    output[i*3] = ((values[i]/100) % 10) + '0';
    output[i*3+1] = ((values[i]/10) % 10) + '0';
    output[i*3+2] = (values[i] % 10) + '0';
  }
  for (int i = 0; i < 216; i++){
    Serial.print(output[i]);
  }
  for (int i = 0; i < 72; i++){
  valuesout[i] = ((output[i*3] - '0') * 100) + ((output[i*3+1] - '0') * 10) + (output[i * 3 + 2] - '0');
  Serial.println(valuesout[i]);
  //Serial.println((output[i*3] - '0') * 100);
  }
  
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
