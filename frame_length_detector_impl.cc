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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "frame_length_detector_impl.h"
#include <stdio.h>
#include <volk/volk.h>

namespace gr {
  namespace zigbee {

    frame_length_detector::sptr
    frame_length_detector::make(const int Q, const std::vector<float> &symbol_table)
    {
      return gnuradio::get_initial_sptr
        (new frame_length_detector_impl(Q, symbol_table));
    }

    /*
     * The private constructor
     */
    frame_length_detector_impl::frame_length_detector_impl(const int Q, const std::vector<float> &symbol_table)
      : gr::block("frame_length_detector",
              gr::io_signature::make2(2, 2, sizeof(float), sizeof(char)),
              gr::io_signature::make(1, 1, sizeof(float))),
	d_Q(Q),
	d_symbol_table(symbol_table),
	d_state(0),
	d_filter(new filter::kernel::fir_filter_fff(d_Q,d_symbol_table))
    {}

    /*
     * Our virtual destructor.
     */
    frame_length_detector_impl::~frame_length_detector_impl()
    {
	delete d_filter;
    }

    void
    frame_length_detector_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
        ninput_items_required[0] = noutput_items*d_Q;
	ninput_items_required[1] = noutput_items*d_Q;
    }

    int
    frame_length_detector_impl::demodulator(const float input[])
    {
	std::vector<float> d_pulse;
	size_t align = volk_get_alignment();
	float *d_matched_out = (float*) volk_malloc(2*sizeof(float),align);
	std::vector<float> max_out;
	std::vector<int> max_index;
	for(int k = 0; k < 2; k++){
	  max_out.push_back(-INFINITY);
	  max_index.push_back(0);
	}

	for(int j = 0; j < 16; j++){
	  for(int k=d_Q-1; k>=0; k--){
	    d_pulse.push_back(d_symbol_table[j*d_Q+k]);
	  }//for k, set value for d_pulse;
	  d_filter->set_taps(d_pulse);
	  d_filter->filterNdec(d_matched_out, &d_frame_length[0], 2, d_Q);
	  //printf("d_matched filter output are %f and %f:\n",d_matched_out[0],d_matched_out[1]);
	  for(int l=0; l<2; l++){
	  if(d_matched_out[l] > max_out[l]){
	    max_out[l] = d_matched_out[l];
	    max_index[l] = j;
	    }
	  }
	  d_pulse.clear();
	}// for j=0~16; 
	//printf("indeces with max output is %d and %d\n", max_index[0], max_index[1]);
	int N = max_index[0]+max_index[1]*16;//compute the demodulated result here;
	//printf("demodulated result is %d\n",N);
	return N;
    }

    int
    frame_length_detector_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
        const float *in_data = (const float *) input_items[0];
        const char *in_flag = (const char *) input_items[1];
        float *out = (float *) output_items[0];

	int i = 0;
	int k = 0;
	int ki = 0; 
	while(i < ninput_items[1]){
	  if(d_state == 0){
	    //flags has not been detected yet
	    if(in_flag[i]==0){
	      d_state = 0;
	      i++;
	    }
	    else
	      d_state = 1;
	   }
	   else if(d_state==1){
	     for(int j = 0; j < 2*d_Q; j++){
		if(in_flag[i+j]==1){
		  d_state =1;
		  i = i+j;
		  j=0;
	          //printf("current pointer is %d\n",i);//for debug
		  d_frame_length.clear();
		}//if flag
		d_frame_length.push_back(in_data[i+j]);
	      }//for loop
	      d_N = demodulator(&d_frame_length[0]);
	      printf("frame length is %d\n",d_N);
              d_frame_length.clear();
	      d_state = 2;
	      i=i+2*d_Q;//move pointer
	   }//else d_state ==1

	   else if(d_state == 2){
	     for(int j = 0 ;j<2*d_N*d_Q; j++){
		if(in_flag[i+j] ==1){
		  d_state = 1;
		  i=i+j;
		  j=0;
		  d_frame_length.clear();
		  printf("BREAK IS CALLED!");
		  break;
		}
		d_frame_length.push_back(in_data[i+j]);
		if(d_frame_length.size()==2*d_Q){
		  int kk=j/(2*d_Q);
		  d_result.push_back(demodulator(&d_frame_length[0]));
		  d_frame_length.clear();
		  printf("the %dth result is %d\n", kk,d_result[kk]);
		  out[ki+kk]=d_result[kk];
		}
	     }// for j
	     ki=ki+d_N;
	     d_result.clear();
	     d_state=0;
	     i=i+2*d_N*d_Q;
	     k++;
	    }//*/
	}//while statement
	printf("noutput_items is %d\n",noutput_items);
        // Tell runtime system how many input items we consumed on
        // each input stream.
        consume_each (i);
	printf("the input length consumed %d\n",i);
	printf("number of packets processed per output is %d\n",k);
        // Tell runtime system how many output items we produced.
        return d_N*k;
    }

  } /* namespace zigbee */
} /* namespace gr */

