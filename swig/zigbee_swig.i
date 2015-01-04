/* -*- c++ -*- */

#define ZIGBEE_API
#define FILTER_API
%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "zigbee_swig_doc.i"

%{
#include "zigbee/fm_soft_detector.h"
#include "gnuradio/filter/fir_filter.h"
#include "zigbee/noncoherent_detector.h"
%}
%include "gnuradio/filter/fir_filter.h"
%include "zigbee/fm_soft_detector.h"
GR_SWIG_BLOCK_MAGIC2(zigbee, fm_soft_detector);
%include "zigbee/noncoherent_detector.h"
GR_SWIG_BLOCK_MAGIC2(zigbee, noncoherent_detector);

