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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "fm_soft_detector_impl.h"
#include <stdio.h>
#include <volk/volk.h>
#include <sys/syscall.h>
namespace gr {
  namespace zigbee {

    fm_soft_detector::sptr
    fm_soft_detector::make(const int spc, const std::vector<float> &symbol_table, int preset_N)
    {
      return gnuradio::get_initial_sptr
        (new fm_soft_detector_impl(spc, symbol_table, preset_N));
    }

    /*
     * The private constructor
     */
    fm_soft_detector_impl::fm_soft_detector_impl(const int spc, const std::vector<float> &symbol_table, int preset_N)
      : gr::block("fm_soft_detector",
              gr::io_signature::make2(2, 2, sizeof(float), sizeof(char)),
              gr::io_signature::make(0, 0, 0)),
	d_Q(32*spc),
	d_symbol_table(symbol_table),
        d_preset_N(preset_N),
	d_state(0),
	d_remaining(0),
	d_nfilters(16)
    {
      message_port_register_out(pmt::mp("msg_out"));
      d_filters = std::vector<filter::kernel::fir_filter_fff*>(d_nfilters);
      std::vector<float> fil_taps;
      for(int i = 0; i < d_nfilters; i++) {
        for(int dk = d_Q-1; dk >= 0; dk--){
	  fil_taps.push_back(d_symbol_table[i*d_Q+dk]);
        }
	d_filters[i] = new filter::kernel::fir_filter_fff(1, fil_taps);
        fil_taps.clear();
      }       
    }

    /*
     * Our virtual destructor.
     */
    fm_soft_detector_impl::~fm_soft_detector_impl()
    {
      for(int i = 0; i < d_nfilters; i++) {
        delete d_filters[i];
      }

    }

    int
    fm_soft_detector_impl::demodulator(const float input[])
    {
	float max_out = -INFINITY; //the max matched filter output among 16 pulses. 
        int max_index =0; 
	//16-ary demodulation
        int d_align = volk_get_alignment();
        float *d_output = (float*)volk_malloc(1*sizeof(float), d_align);
        for(int dj = 0; dj < 16; dj++){
          //float *d_taps = &d_symbol_table[dj*d_Q];
	  /*float matched_out =0;
          for(int dk = 0; dk < d_Q; dk++){
            matched_out += d_symbol_table[dj*d_Q+dk]*input[dk]; 
          }//*/
          //volk_32f_x2_dot_prod_32f_a(d_output,input, d_taps, d_Q );
          d_filters[dj]->filterNdec(d_output, input, 1, d_Q);
	  //printf("matched_out is %d, %f\n", dj, *d_output);

          if(*d_output> max_out){
            max_out = *d_output;
            max_index = dj;
          }
        }// for j=0~16; 
	//printf("demodulated result is %f and %d\n",max_out, max_index);
        volk_free(d_output);
        return max_index;
    }

    int
    fm_soft_detector_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
      static int first=0;
        pid_t pid;

        if(first<1) {
          /* get the process id */
          if ((pid = syscall(SYS_gettid)) < 0) {
            perror("Unable to get pid/tid\n");
          } else {
            printf("fm_soft-->thread id is %d\n", pid);
          }
          first++;
        }

        const float *in_data = (const float *) input_items[0];
        const char *in_flag = (const char *) input_items[1];
        //char *out = (char *) output_items[0];
	int j=0;
	int ni=std::min(ninput_items[0],ninput_items[1]);
        int no = int(ni/(2*d_Q)); 
        int process;
        char out[128];
	//printf("At the beginning: ni is %d, %d, %d\n",ni,ninput_items[0],ninput_items[1]);


        if(d_state == 0){
          for(int l = 0; l < ni; l++){
	    //flags have not been detected yet
            if(in_flag[l] != 0){
              d_state = 1;
              consume_each(l+1);
	      return 0;
	    }
          }
	  consume_each(ni);
	  return 0;
        }
        else if(d_state==1){
          if(ni<d_Q*2) {//not enough input samples for length demodulation
            //printf("Not enough smaples for length demodulation\n");
            return 0;
          }
          int d_length[2];
          for(int jj =0; jj<2; jj++){
            d_length[jj] = demodulator(&in_data[jj*d_Q]);
          }
          d_N = d_length[0]+d_length[1]*16;
          if(d_preset_N > 0){
            d_N = d_preset_N;
          }
          if(d_N > 127){// Since 7 bits is used for frame length, the frame length is wrong if it's greater than 127.
            d_state = 0;	        //so we enter state 0 to search the next flags. 
            //printf("detected frame length is out of range, research a flag");
          }
          else{
            d_remaining = d_N; // when d_N is smaller than 127, the frame length detected might form a packet. the remaining bits to be processed in state 2.
            //printf("The frame length is %d\n", d_N);
            consume_each(2*d_Q);
            d_state = 2;
            return 0;
          }
        }//else d_state ==1
        else if(d_state == 2){
          if(d_remaining > no){ // if d_remaining is greater than ni, we only process ni bits with the current buffer.
            process = no; 
            d_remaining = d_remaining -process;
            //printf("d_remaining lager than noutput_items process: %d and d_remaining:%d\n", process, d_remaining);
          }
          else{
            process = d_remaining;
            d_remaining = 0;
            //printf("d_remaining smaller than ni process: %d and d_remaining:%d\n", process, d_remaining);
          }

          int d_byte_out[2];
          for(j=0; j<process; j++){
            d_byte_out[0] = demodulator(&in_data[(2*j)*d_Q]);
	    d_byte_out[1] = demodulator(&in_data[(2*j+1)*d_Q]);
            int result = d_byte_out[0]+ 16*d_byte_out[1];
	    //out[j] = result;
            *(d_buf_index +j) = result;
          }
          //std::memcpy(d_buf_index, out, process);
          d_buf_index = d_buf_index + process;
          if(d_remaining ==0){
            d_state = 0;
            pmt::pmt_t meta = pmt::make_dict();
            meta = pmt::dict_add(meta, pmt::mp("frame_length"), pmt::from_long(d_N));
            pmt::pmt_t payload = pmt::make_blob(&d_buf[0], d_N);
            d_buf_index = &d_buf[0];
            message_port_pub(pmt::mp("msg_out"), cons(meta,payload)); 
          }
          else{
            d_state = 2;
	    //printf("continuing state =2, d_remaining is %d, processed is %d\n",d_remaining, process);
          }
          consume_each(process*2*d_Q);
	  return process;
        }//d_state =2*/
    }//work

  } /* namespace zigbee */
} /* namespace gr */

