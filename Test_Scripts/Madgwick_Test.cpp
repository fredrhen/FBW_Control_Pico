
#include <stdio.h>
#include "pico/stdlib.h"
//#include "math.h"
#include "../libs/Madgwick/MadgwickAHRS.h"
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

int main() {
    stdio_init_all();

    while(1){
       printf("Helloworlds\n");
       sleep_ms(1000);
    }

    return 0;
}

#pragma clang diagnostic pop

