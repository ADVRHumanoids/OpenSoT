/*
 * Copyright 2008, 2009, 2010,
 *
 * Francois Keith
 * Olivier Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */

#ifndef PORTABILITY_GETTIMEOFDAY_HH
#define PORTABILITY_GETTIMEOFDAY_HH

// This deals with gettimeofday portability issues.
#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#else
#ifdef WIN32
#include <Windows.h>
#include <time.h>

struct timezone {
  int tz_minuteswest; /* minutes W of Greenwich */
  int tz_dsttime;     /* type of dst correction */
};

int gettimeofday(struct timeval *tv, struct timezone *tz);

#else  // WIN32
#error "gettimeof day does not seem to be supported on your platform."
#endif  // WIN32
#endif  //! HAVE_SYS_TIME_H
#endif  //! JRL_WALKGEN_PORTABILITY_GETTIMEOFDAY_HH
