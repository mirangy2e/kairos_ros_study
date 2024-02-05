void setup() {
   // put your setup code here, to run once:
   Serial.begin(115200);
   pinMode(13, OUTPUT);}
void loop() {
   // put your main code here, to run repeatedly:
   float temp = random(10, 40)/1.0;
   Serial.println(temp);
   delay(1000);}
