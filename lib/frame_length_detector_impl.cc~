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
	d_remaining(0),
	d_process(0),
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
	std::vector<float> d_pulse; //contains the matched filter tap for demodulation.
	size_t align = volk_get_alignment();
	float *d_matched_out = (float*) volk_malloc(2*sizeof(float),align);//decimated matched filter output
	std::vector<float> max_out; //the max matched filter output among 16 pulses. 
	std::vector<int> max_index;// the index of max matched filter output. 
	for(int k = 0; k < 2; k++){
	  max_out.push_back(-INFINITY);
	  max_index.push_back(0);
	}
	//16-ary demodulation
	for(int dj = 0; dj < 16; dj++){
	  for(int dk = d_Q-1; dk >= 0; dk--){
	    d_pulse.push_back(d_symbol_table[dj*d_Q+dk]);
	  }//set matched filter taps.
	  d_filter->set_taps(d_pulse);
	  d_filter->filterNdec(d_matched_out, & d_byte_in_process[0], 2, d_Q);
	  //printf("d_matched filter output are %f and %f:\n",d_matched_out[0],d_matched_out[1]);
	  for(int dl=0; dl<2; dl++){
	  if(d_matched_out[dl] > max_out[dl]){
	    max_out[dl] = d_matched_out[dl];
	    max_index[dl] = dj;
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

	int i = 0;//input index;
	int k = 0;
	int ko = 0; 
	int l = 0; //state 0 index;
	int j=0;
	int ni=std::min(ninput_items[0],ninput_items[1]);

	  if(d_state == 0){
	    for(l = 0; l < ni; l++)
	    //flags have not been detected yet
	    if(in_flag[l] != 0){
	      d_state = 1;
	      printf("In state 0 consumed %d\n",l);
	      consume_each(l);
	      //printf("after state 0 consume: in_data is %f,in_flag is %d\n",in_data[l],in_flag[l]);
	      //printf("ni is %d,%d,%d\n",ni,ninput_items[0],ninput_items[1]);
	      return 0;
	    }
	      consume_each(ni);
	      return 0;
	   }
	   else if(d_state==1){
	     for(j = 0; j < 2*d_Q; j++){
		 d_byte_in_process.push_back(in_data[j]);
	        //printf("in_data are %f\n",in_data[j]);
	      }//for loop
	      //printf("the current flag is %d\n",in_flag[0]);
	      d_N = demodulator(& d_byte_in_process[0]);
	      if(d_N > 127){// Since 7 bits is used for frame length, the frame length is wrong if it's greater than 127.
		d_state = 0;	        //so we enter state 0 to search the next flags. 
                 d_byte_in_process.clear();
		consume_each(1); // the input pointer is updated by 1.
		return 0;
		printf("detected frame length is out of range, research a flag");		
	       }
	      else{
	        d_remaining = 2*d_N*d_Q; // when d_N is smaller than 127, the frame length detected might form a packet. the remaining bits to be processed in state 2.
	        //printf("d_N %d and d_remaining %d\n",2*d_N*d_Q, d_remaining);
	        printf("frame length is %d\n",d_N);
                 d_byte_in_process.clear();
	        consume_each(2*d_Q);
	        //printf("end of state 1 input pointer updated %d\n",j);
	        //printf("after state 1, the current input data and flags are: in_data is %f,in_flag is %d\n",in_data[j],in_flag[j]);
	        d_state = 2;
	        return 0;
	      }
	   }//else d_state ==1

	   else if(d_state == 2){
	     //printf("d_remaining is %d\n",d_remaining);
	     if(d_remaining > ni){ // if d_remaining is greater than ni, we only process ni bits with the current buffer.
		printf("ni is %d, d_remaining is %d\n",ni, d_remaining);
		d_process = ni; 
		d_remaining = d_remaining -d_process;
		printf("d_remaining lager than ni d_process: %d and d_remaining:%d\n", d_process, d_remaining);
	      }
	     else{
		printf("ni is %d, and d_remaining is %d\n",ni, d_remaining);
		d_process = d_remaining;
		d_remaining = 0;
		printf("d_remaining smaller than ni d_process: %d and d_remaining:%d\n", d_process, d_remaining);
	     }


	     for(j =0 ;j < d_process; j++){
		 d_byte_in_process.push_back(in_data[j]);
		if( d_byte_in_process.size()%(2*d_Q)==0){
		  no=j/(2*d_Q);
		  //printf("the last data in 128 to be demod is %f\n",in_data[j]);
		  d_result.push_back(demodulator(& d_byte_in_process[0]));
		   d_byte_in_process.clear();
		  printf("the %dth result is %d\n", no+1,d_result[no]);
		  out[no]=d_result[no];
		}
	     }

	     if(d_remaining ==0){
       	       d_state = 0;
	     }
	     else{
	       d_state = 2;
	       printf("continuing state =2, d_remaining is %d, processed is %d\n",d_remaining, d_process);
	     }
	     d_result.clear();
	     //printf("the input pointer update after state 2 is %d\n",j);
	     consume_each(d_process);
	     //printf("In state 2 it consumed %d samples: the current in_data is %f,in_flag is %d\n",d_process,in_data[j],in_flag[j]);
	     return no+1;
	    }//d_state =2*/
    }

  } /* namespace zigbee */
} /* namespace gr */

