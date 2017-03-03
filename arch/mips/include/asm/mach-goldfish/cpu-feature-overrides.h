/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2003, 2004 Chris Dearman
 * Copyright (C) 2005 Ralf Baechle (ralf@linux-mips.org)
 */
#ifndef __ASM_MACH_MIPS_CPU_FEATURE_OVERRIDES_H
#define __ASM_MACH_MIPS_CPU_FEATURE_OVERRIDES_H

/* QEMU does not emulate cache, so no data cache aliases
 * This is important to HIGHMEM support
 */
#define cpu_has_dc_aliases		0

#endif /* __ASM_MACH_MIPS_CPU_FEATURE_OVERRIDES_H */
