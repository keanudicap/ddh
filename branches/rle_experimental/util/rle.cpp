#include "rle.h"
#include <iostream>

warthog::arraylist< warthog::rle::rle_run >*
warthog::rle::compress(const char* str, uint32_t strlen)
{
    if(strlen >= (1 << 24))
    {
        std::cerr << "err; warthog::rle::compress input string is too large.\n";
        exit(EXIT_FAILURE);
    }

    warthog::arraylist< warthog::rle::rle_run >* str_rle = new
        warthog::arraylist< warthog::rle::rle_run >();
    for(uint32_t i = 0, j = 0; i < strlen; i = j) 
    {
        while(str[i] == str[++j]) {}
        if(j >= strlen) { j = strlen; }
        str_rle->push_back(warthog::rle::rle_run(i, str[i]));
    }

    return str_rle;
}
