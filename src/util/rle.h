#ifndef WARTHOG_RLE_H
#define WARTHOG_RLE_H

// rle.h
//
// Utilities for compressing strings with run-length encoding.
// The current implementation is limited to strings whose length 
// is less than 2^{24}.
//
// @author: dharabor
// @created: 2014-10-01
//

#include "arraylist.h"
#include <cassert>

namespace warthog
{

namespace rle
{

class rle_run
{
    public:
        rle_run() : label_(0) { }
        rle_run(uint32_t index, uint8_t label)
        {
            assert(index < (1 << 24));
            label_ = (index << 8) | label;
        }

        rle_run(const rle_run& other)
        {
            this->label_ = other.label_;
        }

        warthog::rle::rle_run&
        operator=(const warthog::rle::rle_run& other)
        {
            this->label_ = other.label_;
            return *this;
        }

        void
        update(uint32_t index, uint8_t label)
        {
            label_ = (index << 8) | label;
        }
        
        inline uint32_t 
        get_run_label() { return label_ & 255; }

        inline uint32_t
        get_run_index() { return (label_ >> 8); }
    
    private:
        uint32_t label_;
};

// Compress @param str using run-length-encoding.
// @return a pointer to an rle-compressed version of @param str
//
// Note that we store the start of the run and its label
// (not the length and its label)
warthog::arraylist<warthog::rle::rle_run>*
compress(const char* str, uint32_t strlen)
{
    if(strlen >= (1 << 24))
    {
        std::cerr << "err; warthog::rle::compress input string is too large.\n";
        exit(EXIT_FAILURE);
    }

    warthog::arraylist<warthog::rle::rle_run>* str_rle = new
        warthog::arraylist<warthog::rle::rle_run>();
    for(uint32_t i = 0, j = 0; i < strlen; i = j) 
    {
        while(str[i] == str[++j]) {}
        if(j >= strlen) { j = strlen; }
        str_rle->push_back(warthog::rle::rle_run(i, str[i]));
    }

    return str_rle;
}

}
}

#endif

