/* -*- c++ -*- */
/* 
 * Copyright 2014 <+YOU OR YOUR COMPANY+>.
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

#ifndef INCLUDED_ZIGBEE_NONCOHERENT_DETECTOR_IMPL_H
#define INCLUDED_ZIGBEE_NONCOHERENT_DETECTOR_IMPL_H

#include <zigbee/noncoherent_detector.h>

namespace gr {
  namespace zigbee {

    class noncoherent_detector_impl : public noncoherent_detector
    {
     private:
      int d_N; //the frame length calculated; 
      int d_Q; //the spreading factor ; 
      int d_state; // the processing state; 0: searching for sync, 1:synced, looking for d_N by demod, 2: synced, output the N symbols. 
      int d_remaining;//the remaining bits to be processed at the beginning state 2.
      int d_process;// the bits to be processed in state 2.
      char buf[128];
      char *buf_index = buf;
      std::vector<gr_complex> d_symbol_table; //the 16-ary symbol table used in demodulation.
     public:
      noncoherent_detector_impl(const int Q, const std::vector<gr_complex> &symbol_table);
      ~noncoherent_detector_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);
      int demodulator(const gr_complex input[]);
      int general_work(int noutput_items,
		       gr_vector_int &ninput_items,
		       gr_vector_const_void_star &input_items,
		       gr_vector_void_star &output_items);
    };

  } // namespace zigbee
} // namespace gr

#endif /* INCLUDED_ZIGBEE_NONCOHERENT_DETECTOR_IMPL_H */

