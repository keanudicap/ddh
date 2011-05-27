/**
 * $Id: edgewidth.h,v 1.3 2006/09/18 06:22:14 nathanst Exp $
 *
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * HOG File
 *
 * Lookup table for the edge widths.
 *
 * The edges are as follows:
 *
 *  ---------
 *  |   |   |
 *  ---------
 *  |  -|-  |
 *  ---------
 *
 * To cross the bottom middle edge.
 *
 * Tile types:
 *   Empty: 0
 *   Forward slash: 1
 *   Backslash: 2
 *
 *
 * To translate into the lookup table index, multiply
 * tile 1's type by 27, multiply tile 2's type by 9,
 * multiply tile 3's type by 3, and add all the values together,
 * along with tile 4.
 *
 * So you end up with: 27*T1 + 9*T2 + 3*T3 + T4
 *
 * For example,
 *
 *  ---------
 *  | \ | / |
 *  ---------
 *  |   | \ |
 *  ---------
 *
 * 27*2 + 9*1 + 3*0 + 2 = 65
 *
 * The edge width will be found in index 65.
 *
 */

#include "constants.h"

#ifndef EDGEWIDTH_H
#define EDGEWIDTH_H
 
extern "C" {
 
  float edgewidth[81] = {
    TWO,
    ROOT_TWO,
    ONE,
    ONE,
    ONE,
		
    ONE_OVER_ROOT_TWO,
    ROOT_TWO,
    ROOT_TWO,
    ONE,
    ONE,
		
    ONE,
    ONE,
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
		
    ONE,
    ONE,
    ONE,
    ROOT_TWO,
    ROOT_TWO,
		
    ONE,
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
    ROOT_TWO,
		
    ROOT_TWO,
    ONE,
    ROOT_TWO,
    ROOT_TWO,
    ONE,
		
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
    ROOT_TWO,
    ROOT_TWO,
		
    ONE,
    ONE,
    ONE,
    ONE,
    ONE,
		
    ONE,
    ONE_OVER_ROOT_TWO,
    ONE,
    ONE_OVER_ROOT_TWO,
    ONE,
		
    ROOT_TWO,
    ROOT_TWO,
    ONE,
    ONE,
    ONE,
		
    ONE_OVER_ROOT_TWO,
    ROOT_TWO,
    ROOT_TWO,
    ONE,
    ONE,
		
    ONE,
    ONE,
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
		
    ONE,
    ONE,
    ONE,
    ONE,
    ONE,
		
    ONE,
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
    ONE,
		
    ONE,
    ONE,
    ONE,
    ONE,
    ONE,
		
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
    ONE,
    ONE,
		
    ONE
  };
}


#endif
