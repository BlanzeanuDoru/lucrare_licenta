#include <at86rf2xx.h>

// the setup function runs once when you press reset or power the board
void setup() {
  at86rf2xx.init();
  at86rf2xx.set_chan(26); 
}

// the loop function runs over and over again forever
void loop() {
  uint8_t data[] = {'w', 'o', 'r', 'k', 's'};
  at86rf2xx.send(data, 5);
  delay(5000);
}

