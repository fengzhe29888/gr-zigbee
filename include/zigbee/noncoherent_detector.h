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


#ifndef INCLUDED_ZIGBEE_NONCOHERENT_DETECTOR_H
#define INCLUDED_ZIGBEE_NONCOHERENT_DETECTOR_H

#include <zigbee/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace zigbee {

    /*!
     * \brief <+description of block+>
     * \ingroup zigbee
     *
     */
    class ZIGBEE_API noncoherent_detector : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<noncoherent_detector> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of zigbee::noncoherent_detector.
       *
       * To avoid accidental use of raw pointers, zigbee::noncoherent_detector's
       * constructor is in a private implementation
       * class. zigbee::noncoherent_detector::make is the public interface for
       * creating new instances.
       */
      static sptr make(const int Q, const std::vector<gr_complex> &symbol_table, int preset_N);
    };

  } // namespace zigbee
} // namespace gr

#endif /* INCLUDED_ZIGBEE_NONCOHERENT_DETECTOR_H */

