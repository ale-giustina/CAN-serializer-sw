/**
 * @file serializer.h
 * @brief Entry points called from main.c.
 */

#ifndef SERIALIZER_H
#define SERIALIZER_H

/** Call once after the CubeMX MX_*_Init() functions. */
void serializer_init(void);

/** Call continuously from the main loop. */
void serializer_loop(void);

#endif /* SERIALIZER_H */
