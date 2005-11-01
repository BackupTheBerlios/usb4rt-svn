/***************************************************************************
 *   Copyright (C) 2005 by Jörg Langenberg                                 *
 *   joerg.langenberg@gmx.net                                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef RT_UHCI_HUB_H
#define RT_UHCI_HUB_H

#include <host/rt_uhci.h>

/* BITs im USB-IO PORTSC[0,1]-Register */
#define PORTSC_CCS          0x0001    /* Current Connect Status */
#define PORTSC_CSC          0x0002    /* Connect Status Change */
#define PORTSC_PE           0x0004    /* Port Enabled / Disabled */
#define PORTSC_PEC          0x0008    /* Port Enable/Disable Change [R/W] */
#define PORTSC_DPLUS        0x0010    /* Line Status D+ */
#define PORTSC_DMINUS       0x0020    /* Line Status D- */
#define PORTSC_RD           0x0040    /* Resume Detect */
#define PORTSC_LSDA         0x0100    /* Low Speed Device Attached */
#define PORTSC_PR           0x0200    /* Port Reset */
#define PORTSC_OC           0x0400    /* Over-current Active */
#define PORTSC_OCC          0x0800    /* Over-current Change */
#define PORTSC_SUSPEND      0x1000    /* Suspend */

#endif
