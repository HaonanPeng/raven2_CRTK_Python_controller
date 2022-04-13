#include <LS7366.h>
#include <SPI.h>

LS7366 myLS7366_1(SS);  //

LS7366 myLS7366_2(8);  //

LS7366 myLS7366_3(7);  //

void setup() {
  
  Serial.begin(115200);
  myLS7366_1.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX4);
  myLS7366_1.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_4 );
  myLS7366_1.clear_counter();
  myLS7366_1.clear_status_register();
  myLS7366_1.write_data_register(4);

  myLS7366_2.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX4);
  myLS7366_2.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_4 );
  myLS7366_2.clear_counter();
  myLS7366_2.clear_status_register();
  myLS7366_2.write_data_register(4);

  myLS7366_3.write_mode_register_0(FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX4);
  myLS7366_3.write_mode_register_1(NO_FLAGS | EN_CNTR | BYTE_4 );
  myLS7366_3.clear_counter();
  myLS7366_3.clear_status_register();
  myLS7366_3.write_data_register(4);
}

void loop() {
  long count_1 = myLS7366_1.read_counter();
  long count_2 = myLS7366_2.read_counter();
  long count_3 = myLS7366_3.read_counter();
  
  //Serial.print("Count: ");
  Serial.print(count_1, DEC);
  Serial.print('_');
  Serial.print(count_2, DEC);
  Serial.print('_');
  Serial.print(count_3, DEC);
  //Serial.print(" Status: ");
  //print_binary(myLS7366.read_status_register());
  Serial.print("\n");
  delay(10);
}

//Function to print out one byte in a readable, left-padded binary format

void print_binary(byte val)
{
byte i=0;
for (i=0;i<8;i++)
    {
        if (val & (0x01 << (7-i)))
        {
             Serial.print("1");
        }
        else
        {
             Serial.print("0");
        }
        if (i==3) Serial.print("_");
    }
}
