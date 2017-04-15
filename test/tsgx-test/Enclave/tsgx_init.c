#include <stdio.h>
#include <stdint.h>

extern void __bss_start;
extern void _end;

void tsgx_init()
{
    //uint64_t *addr = &tsgx_init;
    uint64_t addr = (uint64_t)tsgx_init & 0xfffffffffff00000;
    uint64_t bcc_start, bcc_end;
    unsigned access;
    int i;

    //printf("text addr: %lx\n", addr);
    // Code pages
    for (i = 0; i < 30; i++) {
      access = *(uint64_t *)(addr);
      addr += 4096;
    }

    bcc_start = (uint64_t)&__bss_start & 0xfffffffffffff000;
    bcc_end = (uint64_t)&_end & 0xfffffffffffff000;
    //printf("bcc addr: %lx\n", bcc_start);
    // Uninitialized data pages
    while (bcc_start <= bcc_end) {
      access = *(uint64_t *)(bcc_start);
      *(uint64_t *)(bcc_start) = 1;
      bcc_start += 4096;
    }

    // stack
    addr = (uint64_t)&addr & 0xfffffffffffff000;
    //printf("stack addr: %lx\n", addr);
    for (i = 0; i < 64; i++) {
      access = *(uint64_t *)(addr);
      *(uint64_t *)(addr) = 1;
      addr -= 4096;
    }

#if 0 // Seems we do not need to touch heap memory.
    void *heap_base = get_heap_base();
    printf("heap addr: %lx\n", (uint64_t)heap_base);

    void *ptr;
    ptr = malloc(8192);
    printf("malloc addr: %lx\n", (uint64_t)ptr);
#endif
}
