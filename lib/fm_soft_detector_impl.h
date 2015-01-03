/* -*- c++ -*- */
/* 
 * Copyright 2014 Zhe Feng, Achilleas Anastasopoulos.
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

#ifndef INCLUDED_ZIGBEE_FM_SOFT_DETECTOR_IMPL_H
#define INCLUDED_ZIGBEE_FM_SOFT_DETECTOR_IMPL_H

#include <zigbee/fm_soft_detector.h>
#include <gnuradio/filter/fir_filter.h>
namespace gr {
  namespace zigbee {

    class fm_soft_detector_impl : public fm_soft_detector
    {
     private:
      int d_N; //the frame length calculated; 
      int d_Q; //the spreading factor ; 
      int d_preset_N;
      int d_state; // the processing state; 0: searching for sync, 1:synced, looking for d_N by demod, 2: synced, output the N symbols. 
      int d_remaining;//the remaining bits to be processed at the beginning state 2.
      char d_buf[128];
      char *buf_index = &d_buf[0];
      std::vector<float> d_symbol_table; //the 16-ary symbol table used in demodulation.

     public:
      fm_soft_detector_impl(const int Q, const std::vector<float> &symbol_table, int preset_N);
      ~fm_soft_detector_impl();

      // Where all the action really happens
      int demodulator(const float input[]);
      int general_work(int noutput_items,
		       gr_vector_int &ninput_items,
		       gr_vector_const_void_star &input_items,
		       gr_vector_void_star &output_items);
    };

  } // namespace zigbee
} // namespace gr

#endif /* INCLUDED_ZIGBEE_FM_SOFT_DETECTOR_IMPL_H */

