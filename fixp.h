/*
 * fixp.h
 *
 *  Created on: Feb 8, 2015
 *      Author: shaun
 */

#ifndef FIXP_H_
#define FIXP_H_

#ifdef ECLIPSE_INDEXER
#define ACC_LITERAL(num) num
#define FRACT_LITERAL(num) num
#define _Accum float
#define _Fract float
#define _Sat
#else
#define ACC_LITERAL(num) num##k
#define FRACT_LITERAL(num) num##r
#endif

#endif /* FIXP_H_ */
