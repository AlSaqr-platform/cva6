#include "uart.h"
#include "spi.h"
#include "sd.h"
#include "gpt.h"

int main()
{
    init_uart(40000000, 76800);
    print_uart("Hello World!\r\n");

    int res = gpt_find_boot_partition((uint8_t *)0x80000000UL, 2 * 16384);

    return 0;
}

void handle_trap(void)
{
    // print_uart("trap\r\n");
}
