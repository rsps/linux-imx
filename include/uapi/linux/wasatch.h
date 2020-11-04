/*
 * include/linux/misc/wasatch.h
 *
 *  Userspace header for
 *  RSP custom FPGA to control Wasatch
 *
 *  Copyright (c) 2018 Markku Nivala
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef UAPI_WASATCH_H
#define UAPI_WASATCH_H

#define CAPTURE_START       0x01
#define CAPTURE_STOP        0x02
#define CAPTURE_READ        0x03
#define CAPTURE_BINNING_ON  0x04
#define CAPTURE_BINNING_OFF 0x05

#define STATUS_CAPTURE      0x01
#define STATUS_FRAME_AVAIL  0x02
#define STATUS_FRAME_RDY    0x04
#define STATUS_BINNING      0x08

#define CDS_CTRL_DISABLE    0x01
#define CDS_CTRL_ACT_VALUE  0x02

#define STANDBY_ON          0x0F
#define STANDBY_OFF         0xF1


#endif /* UAPI_WASATCH_H */
