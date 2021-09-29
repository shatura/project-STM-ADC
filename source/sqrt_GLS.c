//
// Файл для построения графика
//

#include <stdint.h>
//-----------------------------------------------
uint16_t sqrt_GLS (uint32_t x)
{
    uint32_t m, y, b;
    uint16_t m16, y16, b16, x16;
    if (x & 0xffff0000)
    {
        if (x & 0xff000000)
        {
            m = 0x40000000;
        }
        else
        {
            m = 0x00400000;
        }
        y = 0;
        do               // Do 12-16 times.
        {
            b = y | m;
            y >>= 1;
            if (x >= b)
            {
                x -= b;
                y |=  m;
            }
            m >>= 2;
        }
        while (m);
        if(y < x) ++y;
        return y;
    }
    else
    {
        if(x & 0x0000ff00)
        {
            m16 = 0x4000;
        }
        else
        {
            m16 = 0x0040;
        }
        y16 = 0;
        x16 = x;
        do              // Do 4-8 times.
        {
            b16 = y16 | m16;
            y16 >>=  1;
            if (x16 >= b16)
            {
                x16 -= b16;
                y16 |= m16;
            }
            m16 >>= 2;
        }
        while (m16);
        if(y16 < x16) ++y16;
        return y16;
    }
}