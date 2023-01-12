#define echoPin 2
#define trigPin 3
#define LED 4

long duration;
int distance;
int delayPeriod;

void setup()
{
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
  Serial.print("Sanity check");
}

void loop()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.016;
  Serial.print(distance);
  Serial.println("cm");
  digitalWrite(LED, HIGH);
  delayPeriod = duration;
  Serial.println(delayPeriod);
  delayMicroseconds(delayPeriod);
  digitalWrite(LED, LOW);
  delay(100);
}