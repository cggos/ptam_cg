#ifndef __COMMON_H
#define __COMMON_H

#include <stdio.h>

#ifdef __USER_DEBUG__
#if __cplusplus < 201103L
#define DEBUG_LOG(format,...) printf("File: %s, Line: %05d: "format"\n", __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define DEBUG_LOG(format,...) printf("File: %s, Line: %05d: " format "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#endif
#else
#define DEBUG_LOG(format,...)
#endif

#endif //__COMMON_H