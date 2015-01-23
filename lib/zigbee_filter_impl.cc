/* -*- c++ -*- */
/* 
 * Copyright 2015 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "zigbee_filter_impl.h"

namespace gr {
  namespace zigbee {

    zigbee_filter::sptr
    zigbee_filter::make(int filter_num)
    {
      return gnuradio::get_initial_sptr
        (new zigbee_filter_impl(filter_num));
    }

    /*
     * The private constructor
     */
    zigbee_filter_impl::zigbee_filter_impl(int filter_num)
      : gr::sync_block("zigbee_filter",
              gr::io_signature::make(1, 1, sizeof(1)),
              gr::io_signature::make(1, 1, sizeof(1))),
      d_filter_num(filter_num)
    {
        set_history(64);
    }

    /*
     * Our virtual destructor.
     */
    zigbee_filter_impl::~zigbee_filter_impl()
    {

    }

    int
    zigbee_filter_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
        const float *in = (const float *) input_items[0];
        float *out = (float *) output_items[0];

        if(d_filter_num ==10){
          for(int i=0; i<noutput_items; i++){
            out[i] = -in[i]-in[i+1]-in[i+2]-in[i+3]-in[i+4]-in[i+5]+in[i+6]+in[i+7]-in[i+8]-in[i+9]-in[i+10]-in[i+11]+in[i+12]+in[i+13]+in[i+14]+in[i+15]-in[i+16]-in[i+17]-in[i+18]-in[i+19]-in[i+20]-in[i+21]+in[i+22]+in[i+23]+in[i+24]+in[i+25]+in[i+26]+in[i+27]+in[i+28]+in[i+29]+in[i+30]+in[i+31]+in[i+32]+in[i+33]-in[i+34]-in[i+35]-in[i+36]-in[i+37]-in[i+38]-in[i+39]+in[i+40]+in[i+41]-in[i+42]-in[i+43]-in[i+44]-in[i+45]-in[i+46]-in[i+47]-in[i+48]-in[i+49]+in[i+50]+in[i+51]-in[i+52]-in[i+53]+in[i+54]+in[i+55]-in[i+56]-in[i+57]-in[i+58]-in[i+59]-in[i+60]-in[i+61]+in[i+62]+in[i+63];
          }
        }
        else if(d_filter_num ==7){
          for(int i=0; i<noutput_items; i++){
            out[i] = -in[i]-in[i+1]-in[i+2]-in[i+3]-in[i+4]-in[i+5]-in[i+6]-in[i+7]-in[i+8]-in[i+9]+in[i+10]+in[i+11]+in[i+12]+in[i+13]+in[i+14]+in[i+15]-in[i+16]-in[i+17]+in[i+18]+in[i+19]+in[i+20]+in[i+21]+in[i+22]+in[i+23]+in[i+24]+in[i+25]-in[i+26]-in[i+27]+in[i+28]+in[i+29]-in[i+30]-in[i+31]+in[i+32]+in[i+33]+in[i+34]+in[i+35]+in[i+36]+in[i+37]-in[i+38]-in[i+39]-in[i+40]-in[i+41]+in[i+42]+in[i+43]+in[i+44]+in[i+45]-in[i+46]-in[i+47]+in[i+48]+in[i+49]+in[i+50]+in[i+51]-in[i+52]-in[i+53]-in[i+54]-in[i+55]+in[i+56]+in[i+57]+in[i+58]+in[i+59]+in[i+60]+in[i+61]-in[i+62]-in[i+63];
          }
        }
        else if(d_filter_num ==0){
          for(int i=0; i<noutput_items; i++){
            out[i] = -in[i]-in[i+1]+in[i+2]+in[i+3]+in[i+4]+in[i+5]-in[i+6]-in[i+7]-in[i+8]-in[i+9]-in[i+10]-in[i+11]-in[i+12]-in[i+13]-in[i+14]-in[i+15]-in[i+16]-in[i+17]+in[i+18]+in[i+19]+in[i+20]+in[i+21]+in[i+22]+in[i+23]-in[i+24]-in[i+25]+in[i+26]+in[i+27]+in[i+28]+in[i+29]+in[i+30]+in[i+31]+in[i+32]+in[i+33]-in[i+34]-in[i+35]+in[i+36]+in[i+37]-in[i+38]-in[i+39]+in[i+40]+in[i+41]+in[i+42]+in[i+43]+in[i+44]+in[i+45]-in[i+46]-in[i+47]-in[i+48]-in[i+49]+in[i+50]+in[i+51]+in[i+52]+in[i+53]-in[i+54]-in[i+55]+in[i+56]+in[i+57]+in[i+58]+in[i+59]-in[i+60]-in[i+61]-in[i+62]-in[i+63];
          }
        }
        // Do <+signal processing+>

        // Tell runtime system how many output items we produced.
        return noutput_items;
    }

  } /* namespace zigbee */
} /* namespace gr */

