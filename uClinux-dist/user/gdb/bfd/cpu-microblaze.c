/* BFD library support routines for Xilinx's microblaze architecture
   Copyright (C) 1993, 1999 Free Software Foundation, Inc.

This file is part of BFD, the Binary File Descriptor library.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  */

/*
 * Copyright (c) 2001 Xilinx, Inc.  All rights reserved. 
 *
 * Redistribution and use in source and binary forms are permitted
 * provided that the above copyright notice and this paragraph are
 * duplicated in all such forms and that any documentation,
 * advertising materials, and other materials related to such
 * distribution and use acknowledge that the software was developed
 * by Xilinx, Inc.  The name of the Company may not be used to endorse 
 * or promote products derived from this software without specific prior 
 * written permission.
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *	Xilinx, Inc.
 */


#include "bfd.h"
#include "sysdep.h"
#include "libbfd.h"

const bfd_arch_info_type bfd_microblaze_arch =
{
  32,		  		/* 32 bits in a word */
  32,		  		/* 32 bits in an address */
  8,		  		/* 8 bits in a byte */
  bfd_arch_microblaze, 		/* Architecture */
  0,		  		/* Machine number - 0 for now */
  "microblaze",	  		/* Architecture name */
  "MicroBlaze",	  		/* Printable name */
  3,		  		/* Section align power */
  TRUE,		  		/* Is this the default architecture ? */
  bfd_default_compatible,	/* Architecture comparison function */
  bfd_default_scan,	   	/* String to architecture conversion */
  NULL			   	/* Next in list */
};
