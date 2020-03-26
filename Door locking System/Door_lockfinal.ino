#include <Keypad.h>
#include <LiquidCrystal.h>
#include <Servo.h>

Servo myservo;

LiquidCrystal lcd(A0, A1, A2, A3, A4, A5);

#define Password_Length 6

int pos;   

char New[Password_Length];
char Original[Password_Length] = "12345";
byte new_count = 0, original_count = 0;

char customKey;
int buzzer=13;
int dooropen=11;
int dopen;

const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1', '2', '3','+'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};
bool door = true;

byte rowPins[ROWS] = {2, 3, 4, 5};
byte colPins[COLS] = {6, 7, 8,11};

Keypad customKeypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup()
{
  myservo.attach(9);
  ServoClose();
  lcd.begin(16, 2);
  lcd.print(" GIT ");
  lcd.setCursor(0, 1);
  lcd.print("--HACKATHON--");
  delay(4000);
  lcd.clear();
  pinMode(buzzer,OUTPUT);
  pinMode(dooropen,INPUT);


}

void loop()
{
  if (door == 0)
  {
    customKey = customKeypad.getKey();
  
    if (customKey == '#')

    {
      lcd.clear();
      ServoClose();
      lcd.print("  Door is close");
      delay(3000);
      door = 1;
    }
  }
 
  else if (customKey=='+')
  {
      lcd.clear();
      ServoOpen();
      lcd.print("  Door is Open");
      digitalWrite(12,HIGH);
      delay(2000);
      digitalWrite(12,LOW);
      door = 0;
  }

  else
    Open();
}

void clearData()
{
  while (new_count != 0)
  {
    New[new_count--] = 0;
  }
  return;
}

void ServoOpen()
{
  for (pos = 180; pos >= 0; pos -= 5) {
   
    myservo.write(pos);             
    delay(15);                       
  }
}

void ServoClose()
{
  for (pos = 0; pos <= 180; pos += 5) { 
    myservo.write(pos);              
    delay(15);                      
  }
}

void Open()
{
  lcd.setCursor(0, 0);
  lcd.print(" Enter Password");
  
  customKey = customKeypad.getKey();
  if (customKey)
  {
    New[new_count] = customKey; 
    lcd.setCursor(new_count, 1);
    lcd.print(New[new_count]); 
    new_count++;
  }

  if (new_count == Password_Length - 1)
  {
    if (!strcmp(New, Original))
    {
      lcd.clear();
      ServoOpen();
      lcd.print("  Door is Open");
      digitalWrite(12,HIGH);
      delay(2000);
      digitalWrite(12,LOW);
      door = 0;
      
    }
    else
    {
      lcd.clear();
      lcd.print("  Wrong Password");
      delay(1000);
      digitalWrite(buzzer,HIGH);
      delay(2000);
      digitalWrite(buzzer,LOW);
      door = 1;
      
    }
    clearData();
  }
}
