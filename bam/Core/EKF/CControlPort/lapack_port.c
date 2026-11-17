/**
 * @file lapack_port.c
 * @brief Embedded settings for CControl's Fortran-translated LAPACK sources.
 */
#include "CLapack/Routines/f2c.h"

/** @brief Suppress optional Fortran diagnostic output. */
static integer s_wsfe(cilist *io)
{
    (void)io;
    return 0;
}

/** @brief Suppress optional Fortran diagnostic output. */
static integer do_fio(integer *count, char *data, ftnlen length)
{
    (void)count;
    (void)data;
    (void)length;
    return 0;
}

/** @brief End the suppressed Fortran diagnostic output. */
static integer e_wsfe(void)
{
    return 0;
}

#include "CLapack/Routines/dlamch.c"

/* The selected LAPACK callers concatenate into separate buffers. */
#define NO_OVERWRITE
#include "CLapack/Tools/s_cat.c"
